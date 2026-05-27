//! Playfield-U detector — YOLOv11n-seg inference via `tract`.
//!
//! Trained on ~90 hand-annotated pinball-cabinet photos where the
//! playfield opening (the "U": two side rails + the lockbar at the
//! bottom) is traced as a single-class `u` polygon. Pure-Rust ONNX,
//! no native ONNX Runtime to ship — same paradigm as [`lockbar-onnx`].
//!
//! Model: YOLOv11n-seg, single class `u`, 640×640 input, two outputs:
//!   - `output0` `[1, 37, 8400]` — per-anchor `[cx, cy, w, h, score,
//!     m0..m31]`, channel-major. 8400 = anchors across the 3 FPN
//!     scales. Box coords are in model-input pixels (0..640).
//!   - `output1` `[1, 32, 160, 160]` — 32 mask prototypes at ¼ model
//!     resolution.
//!
//! A detection's instance mask is `sigmoid(Σ_k coeff_k · proto_k)`,
//! cropped to its box. Post-processing: score threshold, single-class
//! NMS on the axis-aligned boxes, keep the survivors (some frames show
//! two cabinets, hence a `Vec`). Corner/PnP geometry lives downstream;
//! this crate stops at the mask + box.

use std::io::Cursor;
use std::sync::Arc;

use image::imageops::FilterType;
use image::{ImageBuffer, Rgb};
use tract_onnx::prelude::*;

const MODEL_BYTES: &[u8] = include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/models/u.onnx"));

pub const MODEL_SIDE: usize = 640;
const NUM_ANCHORS: usize = 8_400;
/// 4 box + 1 score + 32 mask coefficients.
const DET_CHANNELS: usize = 37;
const NUM_MASK: usize = 32;
pub const PROTO_SIDE: usize = 160;
/// Model pixels per proto pixel (640 / 160).
const PROTO_STRIDE: f32 = (MODEL_SIDE / PROTO_SIDE) as f32;

const DEFAULT_SCORE_THRESHOLD: f32 = 0.25;
const DEFAULT_NMS_IOU_THRESHOLD: f32 = 0.50;
const DEFAULT_MASK_THRESHOLD: f32 = 0.50;
/// At most this many U instances per frame (a couple of frames show two
/// cabinets / a reflection).
const MAX_DETECTIONS: usize = 5;

/// Letterbox transform between original-image pixels and the 640×640
/// model input. Public so downstream geometry can map mask/box coords
/// back to the source frame.
#[derive(Debug, Clone, Copy)]
pub struct Letterbox {
    /// Scale applied (model_pixel / orig_pixel).
    pub scale: f32,
    /// Padding offset inside the 640×640 model frame (model px).
    pub pad_x: f32,
    pub pad_y: f32,
}

impl Letterbox {
    /// Model coords → original-image pixels.
    #[must_use]
    pub fn unmap_xy(self, mx: f32, my: f32) -> (f32, f32) {
        (
            (mx - self.pad_x) / self.scale,
            (my - self.pad_y) / self.scale,
        )
    }

    /// Original-image pixels → model coords.
    #[must_use]
    pub fn map_xy(self, x: f32, y: f32) -> (f32, f32) {
        (x * self.scale + self.pad_x, y * self.scale + self.pad_y)
    }
}

/// One detected U instance.
#[derive(Debug, Clone)]
pub struct UDetection {
    pub confidence: f32,
    /// Axis-aligned box in ORIGINAL image pixels `(x0, y0, x1, y1)`.
    pub bbox: (f32, f32, f32, f32),
    /// Mask probabilities (sigmoid, 0..1) on the 160×160 proto grid,
    /// in MODEL space (÷4). Zeroed outside the detection's box. Index
    /// `py * 160 + px`.
    pub proto_mask: Vec<f32>,
    /// Letterbox used for this frame — maps proto/model ↔ original.
    pub letterbox: Letterbox,
}

impl UDetection {
    /// Sample the mask probability at an original-image pixel.
    #[must_use]
    pub fn mask_prob_at(&self, x: f32, y: f32) -> f32 {
        let (mx, my) = self.letterbox.map_xy(x, y);
        let px = (mx / PROTO_STRIDE).round() as isize;
        let py = (my / PROTO_STRIDE).round() as isize;
        if px < 0 || py < 0 || px >= PROTO_SIDE as isize || py >= PROTO_SIDE as isize {
            return 0.0;
        }
        self.proto_mask[py as usize * PROTO_SIDE + px as usize]
    }

    /// Render the instance mask as a full-frame binary image (1 byte per
    /// pixel, 0 or 255) at the original resolution, thresholded at `thr`.
    #[must_use]
    pub fn mask_image(&self, img_w: u32, img_h: u32, thr: f32) -> Vec<u8> {
        let mut out = vec![0u8; (img_w as usize) * (img_h as usize)];
        for y in 0..img_h {
            for x in 0..img_w {
                if self.mask_prob_at(x as f32, y as f32) >= thr {
                    out[(y * img_w + x) as usize] = 255;
                }
            }
        }
        out
    }

    /// Map a proto-grid cell `(px, py)` (each in `0..PROTO_SIDE`) to its
    /// centre in original-image pixel coordinates, undoing the letterbox.
    /// Downstream geometry (turning the U mask into a lockbar quad / a
    /// vanishing-point fit) walks the proto grid and needs this bridge.
    #[must_use]
    pub fn proto_to_image(&self, px: usize, py: usize) -> (f32, f32) {
        self.letterbox
            .unmap_xy(px as f32 * PROTO_STRIDE, py as f32 * PROTO_STRIDE)
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("model load failed: {0}")]
    ModelLoad(String),
}

type RunModel = SimplePlan<TypedFact, Box<dyn TypedOp>, Graph<TypedFact, Box<dyn TypedOp>>>;

pub struct UDetector {
    model: Arc<RunModel>,
    score_threshold: f32,
    nms_iou_threshold: f32,
    mask_threshold: f32,
}

impl UDetector {
    pub fn new() -> Result<Self, Error> {
        let mut cursor = Cursor::new(MODEL_BYTES);
        let infered = tract_onnx::onnx()
            .model_for_read(&mut cursor)
            .map_err(|e| Error::ModelLoad(format!("model_for_read: {e}")))?
            .with_input_fact(0, f32::fact([1, 3, MODEL_SIDE, MODEL_SIDE]).into())
            .map_err(|e| Error::ModelLoad(format!("with_input_fact: {e}")))?;
        let runnable = infered
            .into_optimized()
            .map_err(|e| Error::ModelLoad(format!("into_optimized: {e}")))?
            .into_runnable()
            .map_err(|e| Error::ModelLoad(format!("into_runnable: {e}")))?;
        Ok(Self {
            model: Arc::new(runnable),
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            nms_iou_threshold: DEFAULT_NMS_IOU_THRESHOLD,
            mask_threshold: DEFAULT_MASK_THRESHOLD,
        })
    }

    pub fn set_score_threshold(&mut self, t: f32) {
        self.score_threshold = t.clamp(0.0, 1.0);
    }

    pub fn set_nms_threshold(&mut self, t: f32) {
        self.nms_iou_threshold = t.clamp(0.0, 1.0);
    }

    pub fn set_mask_threshold(&mut self, t: f32) {
        self.mask_threshold = t.clamp(0.0, 1.0);
    }

    /// Run inference on an RGB888 image. Returns the surviving U
    /// detections after NMS, highest confidence first (empty if none).
    #[must_use]
    pub fn detect(&self, rgb888: &[u8], width: u32, height: u32) -> Vec<UDetection> {
        if width == 0 || height == 0 || rgb888.len() != (width as usize) * (height as usize) * 3 {
            return Vec::new();
        }

        let (input, lb) = letterbox_to_chw(rgb888, width, height);
        let tensor: Tensor =
            tract_ndarray::Array4::from_shape_vec((1, 3, MODEL_SIDE, MODEL_SIDE), input)
                .expect("shape matches len")
                .into();

        let outputs = match self.model.run(tvec!(tensor.into())) {
            Ok(o) => o,
            Err(e) => {
                tracing::warn!(?e, "U-seg ONNX inference failed");
                return Vec::new();
            }
        };

        // Two outputs, order not guaranteed — locate each by element count.
        let mut det_idx = None;
        let mut proto_idx = None;
        for (i, out) in outputs.iter().enumerate() {
            if let Ok(view) = out.to_array_view::<f32>() {
                match view.len() {
                    n if n == DET_CHANNELS * NUM_ANCHORS => det_idx = Some(i),
                    n if n == NUM_MASK * PROTO_SIDE * PROTO_SIDE => proto_idx = Some(i),
                    _ => {}
                }
            }
        }
        let (Some(det_idx), Some(proto_idx)) = (det_idx, proto_idx) else {
            tracing::warn!("U-seg ONNX output: could not locate det/proto tensors");
            return Vec::new();
        };
        // Bind the views to locals so their backing slices outlive the
        // decode below (a slice borrowed from a temporary view wouldn't).
        let det_view = outputs[det_idx]
            .to_array_view::<f32>()
            .expect("checked above");
        let proto_view = outputs[proto_idx]
            .to_array_view::<f32>()
            .expect("checked above");
        let (Some(det), Some(proto)) = (det_view.as_slice(), proto_view.as_slice()) else {
            tracing::warn!("U-seg ONNX output: tensors not contiguous");
            return Vec::new();
        };

        // output0 `[1, 37, 8400]`, channel-major: channel c is the
        // contiguous block `det[c*8400 .. (c+1)*8400]`.
        let ch = |c: usize| &det[c * NUM_ANCHORS..(c + 1) * NUM_ANCHORS];
        let (cx, cy, bw, bh, score) = (ch(0), ch(1), ch(2), ch(3), ch(4));

        let mut kept: Vec<Candidate> = Vec::with_capacity(16);
        for i in 0..NUM_ANCHORS {
            if score[i] < self.score_threshold {
                continue;
            }
            let (hw, hh) = (bw[i] * 0.5, bh[i] * 0.5);
            let mut coeffs = [0.0f32; NUM_MASK];
            for (k, c) in coeffs.iter_mut().enumerate() {
                *c = det[(5 + k) * NUM_ANCHORS + i];
            }
            kept.push(Candidate {
                xyxy: (cx[i] - hw, cy[i] - hh, cx[i] + hw, cy[i] + hh),
                score: score[i],
                coeffs,
            });
        }
        if kept.is_empty() {
            return Vec::new();
        }
        kept.sort_by(|a, b| {
            b.score
                .partial_cmp(&a.score)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let survivors = nms(&kept, self.nms_iou_threshold);
        survivors
            .into_iter()
            .take(MAX_DETECTIONS)
            .map(|c| self.build_detection(c, proto, lb))
            .collect()
    }

    /// Reconstruct the instance mask for a surviving candidate:
    /// `sigmoid(Σ_k coeff_k · proto_k)`, then zero everything outside
    /// the (proto-space) box. Box is mapped back to original pixels.
    fn build_detection(&self, c: Candidate, proto: &[f32], lb: Letterbox) -> UDetection {
        let plane = PROTO_SIDE * PROTO_SIDE;
        // Box in proto coords, clamped to the grid.
        let to_proto = |v: f32| (v / PROTO_STRIDE).round();
        let px0 = to_proto(c.xyxy.0).max(0.0) as usize;
        let py0 = to_proto(c.xyxy.1).max(0.0) as usize;
        let px1 = (to_proto(c.xyxy.2) as usize).min(PROTO_SIDE - 1);
        let py1 = (to_proto(c.xyxy.3) as usize).min(PROTO_SIDE - 1);

        let mut mask = vec![0.0f32; plane];
        for py in py0..=py1.max(py0) {
            for px in px0..=px1.max(px0) {
                let mut acc = 0.0f32;
                let cell = py * PROTO_SIDE + px;
                for (k, &coeff) in c.coeffs.iter().enumerate() {
                    acc += coeff * proto[k * plane + cell];
                }
                mask[cell] = sigmoid(acc);
            }
        }

        let (x0, y0) = lb.unmap_xy(c.xyxy.0, c.xyxy.1);
        let (x1, y1) = lb.unmap_xy(c.xyxy.2, c.xyxy.3);
        UDetection {
            confidence: c.score,
            bbox: (x0, y0, x1, y1),
            proto_mask: mask,
            letterbox: lb,
        }
    }

    #[must_use]
    pub fn mask_threshold(&self) -> f32 {
        self.mask_threshold
    }
}

#[derive(Debug, Clone)]
struct Candidate {
    /// Box in model (640) coords, `(x0, y0, x1, y1)`.
    xyxy: (f32, f32, f32, f32),
    score: f32,
    coeffs: [f32; NUM_MASK],
}

#[inline]
fn sigmoid(x: f32) -> f32 {
    1.0 / (1.0 + (-x).exp())
}

/// Greedy NMS on axis-aligned boxes; `dets` must be sorted by
/// descending score. Returns the survivors in score order.
fn nms(dets: &[Candidate], iou_threshold: f32) -> Vec<Candidate> {
    let mut survivors: Vec<Candidate> = Vec::with_capacity(dets.len());
    for d in dets {
        if !survivors
            .iter()
            .any(|s| iou(d.xyxy, s.xyxy) > iou_threshold)
        {
            survivors.push(d.clone());
        }
    }
    survivors
}

fn iou(a: (f32, f32, f32, f32), b: (f32, f32, f32, f32)) -> f32 {
    let ix1 = a.0.max(b.0);
    let iy1 = a.1.max(b.1);
    let ix2 = a.2.min(b.2);
    let iy2 = a.3.min(b.3);
    let inter = (ix2 - ix1).max(0.0) * (iy2 - iy1).max(0.0);
    let area_a = (a.2 - a.0).max(0.0) * (a.3 - a.1).max(0.0);
    let area_b = (b.2 - b.0).max(0.0) * (b.3 - b.1).max(0.0);
    let union = area_a + area_b - inter;
    if union <= 0.0 { 0.0 } else { inter / union }
}

/// Letterbox-resize the RGB image to 640×640, return the CHW float32
/// tensor (range 0..1) plus the transform used to undo it. Identical
/// scheme to `lockbar-onnx` (grey 114 padding, Ultralytics default).
fn letterbox_to_chw(rgb888: &[u8], width: u32, height: u32) -> (Vec<f32>, Letterbox) {
    let scale = (MODEL_SIDE as f32 / width as f32).min(MODEL_SIDE as f32 / height as f32);
    let new_w = ((width as f32) * scale).round() as u32;
    let new_h = ((height as f32) * scale).round() as u32;
    let pad_x = ((MODEL_SIDE as u32 - new_w) / 2) as f32;
    let pad_y = ((MODEL_SIDE as u32 - new_h) / 2) as f32;

    let buf: ImageBuffer<Rgb<u8>, Vec<u8>> =
        ImageBuffer::from_raw(width, height, rgb888.to_vec()).expect("size validated by caller");
    let resized = image::imageops::resize(&buf, new_w, new_h, FilterType::Triangle);

    const PAD: f32 = 114.0 / 255.0;
    let plane = MODEL_SIDE * MODEL_SIDE;
    let mut chw = vec![PAD; 3 * plane];
    let (pad_x_i, pad_y_i) = (pad_x as u32, pad_y as u32);
    for y in 0..new_h {
        for x in 0..new_w {
            let p = resized.get_pixel(x, y);
            let dst = ((y + pad_y_i) as usize) * MODEL_SIDE + (x + pad_x_i) as usize;
            chw[dst] = p[0] as f32 / 255.0;
            chw[plane + dst] = p[1] as f32 / 255.0;
            chw[2 * plane + dst] = p[2] as f32 / 255.0;
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
