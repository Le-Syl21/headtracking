//! Lockbar detector — YOLOv11n-OBB inference via `tract`.
//!
//! Trained on ~100 hand-annotated pinball-cabinet photos covering
//! Kinect v2 / Kinect v1 / webcam viewpoints plus assorted community
//! cabinet shots (matte black, chrome, brushed, vintage finishes).
//! Runs the resulting model purely in Rust; no native ONNX Runtime
//! to package alongside the plugin.
//!
//! Model: YOLOv11n-OBB, single class `lockbar`, 640×640 input.
//! Output tensor shape `[1, 6, 8400]` where the 6 channels are
//! `[cx, cy, w, h, score, angle]` and 8400 = total anchor count
//! across the three FPN scales. Coordinates are in model-input
//! pixels (0..640) and `angle` is in radians, range `(-π/4, 3π/4)`
//! for Ultralytics' OBB head.
//!
//! Post-processing: confidence threshold, single-class NMS on rotated
//! boxes (we use axis-aligned bbox IoU as a cheap proxy — fine when
//! the cab's lockbar is the only object), pick the highest-confidence
//! detection, convert `(cx, cy, w, h, angle)` to four corners, then
//! letterbox-unmap into the input frame's pixel coordinates.

use std::io::Cursor;
use std::sync::Arc;

use image::imageops::FilterType;
use image::{ImageBuffer, Rgb};
use tract_onnx::prelude::*;

const MODEL_BYTES: &[u8] =
    include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/models/lockbar.onnx"));

const MODEL_SIDE: usize = 640;
const NUM_ANCHORS: usize = 8_400;
const NUM_CHANNELS: usize = 6;

const DEFAULT_SCORE_THRESHOLD: f32 = 0.25;
const DEFAULT_NMS_IOU_THRESHOLD: f32 = 0.50;

/// One detected lockbar in the original frame's pixel coordinates.
///
/// `corners` is `[top_left, top_right, bottom_right, bottom_left]`,
/// in image coordinates (Y points down). The two top corners share
/// the smaller Y; rotation is recovered from the OBB head.
#[derive(Debug, Clone, Copy)]
pub struct LockbarObb {
    pub corners: [(f32, f32); 4],
    pub confidence: f32,
    /// Slope of the top edge in degrees (image Y down → positive
    /// means right-end-lower).
    pub slope_deg: f32,
    /// Mean vertical separation between the top and bottom edges,
    /// the pixel "thickness" of the lockbar.
    pub thickness_px: f32,
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("model load failed: {0}")]
    ModelLoad(String),
}

type RunModel = SimplePlan<TypedFact, Box<dyn TypedOp>, Graph<TypedFact, Box<dyn TypedOp>>>;

pub struct LockbarDetector {
    model: Arc<RunModel>,
    score_threshold: f32,
    nms_iou_threshold: f32,
}

impl LockbarDetector {
    pub fn new() -> Result<Self, Error> {
        let mut cursor = Cursor::new(MODEL_BYTES);
        let infered = tract_onnx::onnx()
            .model_for_read(&mut cursor)
            .map_err(|e| Error::ModelLoad(format!("model_for_read: {e}")))?
            .with_input_fact(0, f32::fact([1, 3, MODEL_SIDE, MODEL_SIDE]).into())
            .map_err(|e| Error::ModelLoad(format!("with_input_fact: {e}")))?;
        let typed = infered
            .into_optimized()
            .map_err(|e| Error::ModelLoad(format!("into_optimized: {e}")))?;
        let runnable = typed
            .into_runnable()
            .map_err(|e| Error::ModelLoad(format!("into_runnable: {e}")))?;
        Ok(Self {
            model: Arc::new(runnable),
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            nms_iou_threshold: DEFAULT_NMS_IOU_THRESHOLD,
        })
    }

    pub fn set_score_threshold(&mut self, t: f32) {
        self.score_threshold = t.clamp(0.0, 1.0);
    }

    pub fn set_nms_threshold(&mut self, t: f32) {
        self.nms_iou_threshold = t.clamp(0.0, 1.0);
    }

    /// Run inference on an RGB888 image. Returns the highest-confidence
    /// lockbar detection after NMS, or `None` if nothing passes the
    /// score threshold.
    pub fn detect(&self, rgb888: &[u8], width: u32, height: u32) -> Option<LockbarObb> {
        if width == 0 || height == 0 {
            return None;
        }
        let expected = (width as usize) * (height as usize) * 3;
        if rgb888.len() != expected {
            return None;
        }

        let (input_tensor, lb) = letterbox_to_chw(rgb888, width, height);
        let tensor: Tensor =
            tract_ndarray::Array4::from_shape_vec((1, 3, MODEL_SIDE, MODEL_SIDE), input_tensor)
                .expect("shape matches len")
                .into();

        let outputs = match self.model.run(tvec!(tensor.into())) {
            Ok(o) => o,
            Err(e) => {
                tracing::warn!(?e, "lockbar ONNX inference failed");
                return None;
            }
        };

        let view = outputs.first()?.to_array_view::<f32>().ok()?;
        let flat = view.as_slice()?;
        if flat.len() != NUM_CHANNELS * NUM_ANCHORS {
            tracing::warn!(
                got = flat.len(),
                expected = NUM_CHANNELS * NUM_ANCHORS,
                "lockbar ONNX output: unexpected length"
            );
            return None;
        }

        // The tensor is `[1, 6, 8400]`, channel-major. Each channel is a
        // contiguous block of 8400 floats: [cx … | cy … | w … | h … | score … | angle …].
        let cx = &flat[0..NUM_ANCHORS];
        let cy = &flat[NUM_ANCHORS..2 * NUM_ANCHORS];
        let bw = &flat[2 * NUM_ANCHORS..3 * NUM_ANCHORS];
        let bh = &flat[3 * NUM_ANCHORS..4 * NUM_ANCHORS];
        let score = &flat[4 * NUM_ANCHORS..5 * NUM_ANCHORS];
        let angle = &flat[5 * NUM_ANCHORS..6 * NUM_ANCHORS];

        // First pass: collect anchors above the score threshold.
        let mut kept: Vec<Detection> = Vec::with_capacity(16);
        for i in 0..NUM_ANCHORS {
            if score[i] < self.score_threshold {
                continue;
            }
            kept.push(Detection {
                cx: cx[i],
                cy: cy[i],
                w: bw[i],
                h: bh[i],
                angle: angle[i],
                score: score[i],
            });
        }
        if kept.is_empty() {
            return None;
        }
        // Sort by score descending for greedy NMS.
        kept.sort_by(|a, b| {
            b.score
                .partial_cmp(&a.score)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let best = nms_pick_best(&kept, self.nms_iou_threshold);
        let corners_model = obb_to_corners(&best);
        let corners_img = corners_model.map(|(x, y)| lb.unmap_xy(x, y));
        let corners_ordered = order_quad_tl_tr_br_bl(corners_img);

        // Slope of the top edge (top-left → top-right), in degrees.
        let (tlx, tly) = corners_ordered[0];
        let (trx, try_) = corners_ordered[1];
        let slope_rad = (try_ - tly).atan2(trx - tlx);
        let slope_deg = slope_rad.to_degrees();
        // Thickness = mean vertical distance between top and bottom edges.
        let (blx, bly) = corners_ordered[3];
        let (brx, bry) = corners_ordered[2];
        let mid_top_y = (tly + try_) * 0.5;
        let mid_bot_y = (bly + bry) * 0.5;
        // The lockbar is rotated, so vertical distance under-estimates
        // thickness when the slope is steep. Project the bottom-edge
        // midpoint onto the normal of the top edge to get true height.
        let dx = trx - tlx;
        let dy = try_ - tly;
        let edge_len = (dx * dx + dy * dy).sqrt().max(1e-3);
        let nx = -dy / edge_len;
        let ny = dx / edge_len;
        let mid_top_x = (tlx + trx) * 0.5;
        let mid_bot_x = (blx + brx) * 0.5;
        let thickness_px = ((mid_bot_x - mid_top_x) * nx + (mid_bot_y - mid_top_y) * ny).abs();

        let _ = (mid_top_y, mid_bot_y);
        Some(LockbarObb {
            corners: corners_ordered,
            confidence: best.score,
            slope_deg,
            thickness_px,
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct Detection {
    cx: f32,
    cy: f32,
    w: f32,
    h: f32,
    angle: f32,
    score: f32,
}

impl Detection {
    /// Axis-aligned bounding box of the rotated quad.
    fn aabb(&self) -> (f32, f32, f32, f32) {
        let corners = obb_to_corners(self);
        let (mut xmin, mut ymin) = (f32::INFINITY, f32::INFINITY);
        let (mut xmax, mut ymax) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
        for (x, y) in corners {
            if x < xmin {
                xmin = x;
            }
            if y < ymin {
                ymin = y;
            }
            if x > xmax {
                xmax = x;
            }
            if y > ymax {
                ymax = y;
            }
        }
        (xmin, ymin, xmax, ymax)
    }
}

/// Greedy NMS over the rotated detections, using axis-aligned IoU as
/// a cheap proxy. `dets` must already be sorted by descending score.
/// Returns the single highest-scoring detection that survives.
fn nms_pick_best(dets: &[Detection], iou_threshold: f32) -> Detection {
    // With descending sort, dets[0] is the global max. NMS only matters
    // if downstream callers want the full filtered list — we return
    // the top-1 either way, but walking the list lets us log whether
    // a competing detection survived.
    let mut survivors: Vec<Detection> = Vec::with_capacity(dets.len());
    for &d in dets {
        let dropped = survivors
            .iter()
            .any(|s| aabb_iou(&d.aabb(), &s.aabb()) > iou_threshold);
        if !dropped {
            survivors.push(d);
        }
    }
    survivors[0]
}

fn aabb_iou(a: &(f32, f32, f32, f32), b: &(f32, f32, f32, f32)) -> f32 {
    let ix1 = a.0.max(b.0);
    let iy1 = a.1.max(b.1);
    let ix2 = a.2.min(b.2);
    let iy2 = a.3.min(b.3);
    let iw = (ix2 - ix1).max(0.0);
    let ih = (iy2 - iy1).max(0.0);
    let inter = iw * ih;
    let area_a = (a.2 - a.0).max(0.0) * (a.3 - a.1).max(0.0);
    let area_b = (b.2 - b.0).max(0.0) * (b.3 - b.1).max(0.0);
    let union = area_a + area_b - inter;
    if union <= 0.0 { 0.0 } else { inter / union }
}

/// Convert YOLO OBB `(cx, cy, w, h, angle)` to four corners (model
/// coordinates). Corner order follows Ultralytics' convention: walk
/// the rectangle clockwise from `(+w/2, -h/2)` rotated by `angle`.
fn obb_to_corners(d: &Detection) -> [(f32, f32); 4] {
    let (sa, ca) = d.angle.sin_cos();
    let hw = d.w * 0.5;
    let hh = d.h * 0.5;
    let local: [(f32, f32); 4] = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)];
    let mut out = [(0.0, 0.0); 4];
    for (i, (lx, ly)) in local.iter().enumerate() {
        let rx = lx * ca - ly * sa;
        let ry = lx * sa + ly * ca;
        out[i] = (d.cx + rx, d.cy + ry);
    }
    out
}

/// Reorder 4 arbitrary corners into TL, TR, BR, BL. Assumes a roughly
/// rectangular quad oriented with the long edges near-horizontal.
fn order_quad_tl_tr_br_bl(pts: [(f32, f32); 4]) -> [(f32, f32); 4] {
    let mut by_y = pts;
    by_y.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    let (mut tl, mut tr) = (by_y[0], by_y[1]);
    if tl.0 > tr.0 {
        std::mem::swap(&mut tl, &mut tr);
    }
    let (mut bl, mut br) = (by_y[2], by_y[3]);
    if bl.0 > br.0 {
        std::mem::swap(&mut bl, &mut br);
    }
    [tl, tr, br, bl]
}

#[derive(Debug, Clone, Copy)]
struct Letterbox {
    /// Scale applied (model_pixel / orig_pixel).
    scale: f32,
    /// Padding offset inside the 640×640 model frame (in model px).
    pad_x: f32,
    pad_y: f32,
}

impl Letterbox {
    /// Inverse map from model coords back to the original image's
    /// pixel coordinates.
    fn unmap_xy(self, mx: f32, my: f32) -> (f32, f32) {
        let x = (mx - self.pad_x) / self.scale;
        let y = (my - self.pad_y) / self.scale;
        (x, y)
    }
}

/// Letterbox-resize the RGB image to MODEL_SIDE×MODEL_SIDE, return
/// the CHW float32 tensor data (range 0..1) plus the letterbox map
/// used to undo the transform on detected coordinates.
fn letterbox_to_chw(rgb888: &[u8], width: u32, height: u32) -> (Vec<f32>, Letterbox) {
    let scale = (MODEL_SIDE as f32 / width as f32).min(MODEL_SIDE as f32 / height as f32);
    let new_w = ((width as f32) * scale).round() as u32;
    let new_h = ((height as f32) * scale).round() as u32;
    let pad_x = ((MODEL_SIDE as u32 - new_w) / 2) as f32;
    let pad_y = ((MODEL_SIDE as u32 - new_h) / 2) as f32;

    let buf: ImageBuffer<Rgb<u8>, Vec<u8>> =
        ImageBuffer::from_raw(width, height, rgb888.to_vec()).expect("size validated by caller");
    let resized = image::imageops::resize(&buf, new_w, new_h, FilterType::Triangle);

    // Build the model input with grey padding (114, the Ultralytics
    // default). Tract takes CHW float32 [0..1].
    const PAD: f32 = 114.0 / 255.0;
    let plane = MODEL_SIDE * MODEL_SIDE;
    let mut chw = vec![PAD; 3 * plane];
    let pad_x_i = pad_x as u32;
    let pad_y_i = pad_y as u32;
    for y in 0..new_h {
        for x in 0..new_w {
            let p = resized.get_pixel(x, y);
            let dst_idx = ((y + pad_y_i) as usize) * MODEL_SIDE + (x + pad_x_i) as usize;
            chw[dst_idx] = p[0] as f32 / 255.0;
            chw[plane + dst_idx] = p[1] as f32 / 255.0;
            chw[2 * plane + dst_idx] = p[2] as f32 / 255.0;
        }
    }
    (
        chw,
        Letterbox {
            scale,
            pad_x,
            pad_y,
        },
    )
}
