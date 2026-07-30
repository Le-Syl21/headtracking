//! Head detector — single-class YOLOv8/v11 "head" ONNX via `tract`.
//!
//! Replaces the frontal **face** detector (crate `face`) with a **head**
//! detector that finds the head as a *shape*, so it keeps tracking when the
//! player leans over the playfield and the camera only sees the top of the
//! skull — precisely where a face detector drops out.
//!
//! The model is a stock Ultralytics YOLOv8/v11 *detect* graph trained on a
//! single `head` class (e.g. SCUT-HEAD or CrowdHuman-head). Exported with
//! `nms=False`, so output0 is the **raw** prediction tensor
//! `[1, 5, 8400]` — channel-major `[cx, cy, w, h, score]`, box coords in
//! model-input pixels (0..640). That is `u.onnx`'s layout minus the 32 mask
//! channels, so the decode below is `u-onnx`'s, truncated to 5 channels; the
//! risky ops that broke MoveNet in tract (`GatherND`/`NonMaxSuppression`)
//! are absent — NMS is done here in Rust.
//!
//! Like `face`/`u-onnx`, the default model is **embedded** (`Detector::new`,
//! `include_bytes!`) so a shipped plugin `.so` pins exactly one model version —
//! no separate model file to keep in sync at deploy time. `from_path` /
//! `from_reader` stay available to load a different model (e.g. a heavier
//! variant) at runtime without a recompile.

use std::io::Read;
use std::path::Path;
use std::sync::Arc;

use image::imageops::FilterType;
use image::{ImageBuffer, Rgb};
use tract_onnx::prelude::*;

/// Square model input side. YOLOv8 is fully convolutional, but an Ultralytics
/// ONNX export bakes the input size and the anchor-grid reshape in, so this
/// must match the export's `imgsz` (see [`NUM_ANCHORS`]). Default 128: on a
/// pincab the player's head fills much of the frame, so the accuracy cost of
/// the smallest input is negligible, and 128 keeps a full 60 Hz *even with the
/// detect overhead* — ≈10 ms inference (≈13.5 ms full detect) on the cab CPU
/// (160 ≈15, 224 ≈30, 320 ≈61 ms). Run on its own worker thread it delivers a
/// fresh head well within every 60 Hz frame.
pub const MODEL_SIDE: usize = 128;
/// Anchors emitted across the 3 FPN scales (strides 8/16/32), derived from
/// [`MODEL_SIDE`]: `(S/8)² + (S/16)² + (S/32)²` — 8400 at 640, 2100 at 320,
/// 1029 at 224, 525 at 160, 336 at 128.
const NUM_ANCHORS: usize =
    (MODEL_SIDE / 8).pow(2) + (MODEL_SIDE / 16).pow(2) + (MODEL_SIDE / 32).pow(2);
/// Per-anchor channels of a single-class detect head: `[cx, cy, w, h, score]`.
const DET_CHANNELS: usize = 5;

const DEFAULT_SCORE_THRESHOLD: f32 = 0.25;
const DEFAULT_NMS_IOU_THRESHOLD: f32 = 0.50;
/// At most this many heads per frame kept after NMS (a pincab has one
/// player; the slack absorbs a reflection or a passer-by).
const MAX_DETECTIONS: usize = 8;

/// Letterbox transform between original-image pixels and the 640×640 model
/// input (grey-114 padding, Ultralytics default). Same scheme as `u-onnx`.
#[derive(Debug, Clone, Copy)]
pub struct Letterbox {
    /// Scale applied (`model_pixel / orig_pixel`).
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

/// One detected head, in the input image's pixel coordinates.
///
/// This is the model-independent hand-off the trackers consume: the Kinect
/// path uses `(cx, cy)` as the region centre and reads depth from the IR
/// sensor there; the webcam path turns `width` into distance via the
/// lockbar-calibrated focal length (`z = fx · real_head_width / width`).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct HeadAnchor {
    /// Box centre X (original image px).
    pub cx: f32,
    /// Box centre Y (original image px).
    pub cy: f32,
    /// Box width (original image px).
    pub width: f32,
    /// Box height (original image px).
    pub height: f32,
    /// Detection score, 0..1.
    pub confidence: f32,
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("model load failed: {0}")]
    ModelLoad(String),
    #[error("model read failed: {0}")]
    ModelIo(#[from] std::io::Error),
}

type RunModel = TypedRunnableModel;

pub struct Detector {
    model: Arc<RunModel>,
    score_threshold: f32,
    nms_iou_threshold: f32,
}

/// The bundled default head model — SCUT-HEAD `nano.pt`
/// (`Abcfsa/YOLOv8_head_detector`), re-exported with `nms=False` to the raw
/// `[1, 5, 8400]` detect output. Embedded so one shipped `.so` pins exactly one
/// model version (no separate file to keep in sync at deploy time).
const MODEL_BYTES: &[u8] = include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/models/head.onnx"));

impl Detector {
    /// Load the embedded default head model. Equivalent to `from_reader` over
    /// the bundled bytes — the plugin's normal entry point.
    pub fn new() -> Result<Self, Error> {
        Self::from_reader(&mut std::io::Cursor::new(MODEL_BYTES))
    }

    /// Load a head-detection ONNX from a file path.
    pub fn from_path(path: impl AsRef<Path>) -> Result<Self, Error> {
        let mut f = std::fs::File::open(path)?;
        Self::from_reader(&mut f)
    }

    /// Load a head-detection ONNX from any reader. The graph must be a
    /// single-class YOLOv8/v11 *detect* export (raw `[1, 5, 8400]` output,
    /// `nms=False`); the input is bound to `[1, 3, 640, 640]` f32.
    pub fn from_reader(reader: &mut dyn Read) -> Result<Self, Error> {
        let infered = tract_onnx::onnx()
            .model_for_read(reader)
            .map_err(|e| Error::ModelLoad(format!("model_for_read: {e}")))?
            .with_input_fact(0, f32::fact([1, 3, MODEL_SIDE, MODEL_SIDE]).into())
            .map_err(|e| Error::ModelLoad(format!("with_input_fact: {e}")))?;
        let runnable = infered
            .into_optimized()
            .map_err(|e| Error::ModelLoad(format!("into_optimized: {e}")))?
            .into_runnable()
            .map_err(|e| Error::ModelLoad(format!("into_runnable: {e}")))?;
        Ok(Self {
            model: runnable,
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

    /// Run inference on an RGB888 image. Returns the surviving heads after
    /// NMS, highest confidence first (empty if none / bad input).
    #[must_use]
    pub fn detect(&self, rgb888: &[u8], width: u32, height: u32) -> Vec<HeadAnchor> {
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
                tracing::warn!(?e, "head ONNX inference failed");
                return Vec::new();
            }
        };

        // Single output `[1, 5, 8400]`; locate it by element count so a
        // stray metadata output doesn't matter.
        let det_view = outputs.iter().find_map(|o| {
            let v = o.to_plain_array_view::<f32>().ok()?;
            (v.len() == DET_CHANNELS * NUM_ANCHORS).then_some(v)
        });
        let Some(det_view) = det_view else {
            tracing::warn!("head ONNX output: no `[1,5,8400]` tensor found");
            return Vec::new();
        };
        let Some(det) = det_view.as_slice() else {
            tracing::warn!("head ONNX output: tensor not contiguous");
            return Vec::new();
        };

        decode_anchors(det, lb, self.score_threshold, self.nms_iou_threshold)
    }
}

/// Box in model (640) coords, `(x0, y0, x1, y1)`, plus score.
#[derive(Debug, Clone, Copy)]
struct Candidate {
    xyxy: (f32, f32, f32, f32),
    score: f32,
}

/// Decode the raw `[1, 5, 8400]` detect tensor into head anchors in
/// original-image pixels: threshold, single-class NMS, letterbox-undo.
///
/// Channel-major: channel `c` is the contiguous block
/// `det[c*8400 .. (c+1)*8400]` — `[cx, cy, w, h, score]`. Pulled out as a
/// free function so it is unit-testable without a model.
fn decode_anchors(
    det: &[f32],
    lb: Letterbox,
    score_threshold: f32,
    nms_iou_threshold: f32,
) -> Vec<HeadAnchor> {
    if det.len() != DET_CHANNELS * NUM_ANCHORS {
        return Vec::new();
    }
    let ch = |c: usize| &det[c * NUM_ANCHORS..(c + 1) * NUM_ANCHORS];
    let (cx, cy, bw, bh, score) = (ch(0), ch(1), ch(2), ch(3), ch(4));

    let mut kept: Vec<Candidate> = Vec::with_capacity(16);
    for i in 0..NUM_ANCHORS {
        if score[i] < score_threshold {
            continue;
        }
        let (hw, hh) = (bw[i] * 0.5, bh[i] * 0.5);
        kept.push(Candidate {
            xyxy: (cx[i] - hw, cy[i] - hh, cx[i] + hw, cy[i] + hh),
            score: score[i],
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

    nms(&kept, nms_iou_threshold)
        .into_iter()
        .take(MAX_DETECTIONS)
        .map(|c| {
            let (x0, y0) = lb.unmap_xy(c.xyxy.0, c.xyxy.1);
            let (x1, y1) = lb.unmap_xy(c.xyxy.2, c.xyxy.3);
            HeadAnchor {
                cx: (x0 + x1) * 0.5,
                cy: (y0 + y1) * 0.5,
                width: (x1 - x0).max(0.0),
                height: (y1 - y0).max(0.0),
                confidence: c.score,
            }
        })
        .collect()
}

/// Greedy NMS on axis-aligned boxes; `dets` must be sorted by descending
/// score. Returns survivors in score order.
fn nms(dets: &[Candidate], iou_threshold: f32) -> Vec<Candidate> {
    let mut survivors: Vec<Candidate> = Vec::with_capacity(dets.len());
    for &d in dets {
        if !survivors
            .iter()
            .any(|s| iou(d.xyxy, s.xyxy) > iou_threshold)
        {
            survivors.push(d);
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

/// Letterbox-resize the RGB image to 640×640, return the CHW float32 tensor
/// (range 0..1) plus the transform used to undo it. Identical scheme to
/// `u-onnx` (grey-114 padding, Ultralytics default).
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
            chw[dst] = f32::from(p[0]) / 255.0;
            chw[plane + dst] = f32::from(p[1]) / 255.0;
            chw[2 * plane + dst] = f32::from(p[2]) / 255.0;
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Identity letterbox: model coords == original coords.
    const ID_LB: Letterbox = Letterbox {
        scale: 1.0,
        pad_x: 0.0,
        pad_y: 0.0,
    };

    #[test]
    fn detector_new_loads_embedded_model() {
        // The embedded SCUT-HEAD nano model parses + optimises through tract,
        // and a blank frame yields no heads (no false positives on grey).
        let det = Detector::new().expect("embedded head.onnx loads through tract");
        let blank = vec![0u8; 320 * 240 * 3];
        assert!(det.detect(&blank, 320, 240).is_empty());
    }

    /// Build a raw `[5, 8400]` (channel-major) buffer and set one anchor.
    fn one_anchor(idx: usize, cx: f32, cy: f32, w: f32, h: f32, score: f32) -> Vec<f32> {
        let mut det = vec![0.0f32; DET_CHANNELS * NUM_ANCHORS];
        for (c, v) in [cx, cy, w, h, score].into_iter().enumerate() {
            det[c * NUM_ANCHORS + idx] = v;
        }
        det
    }

    #[test]
    fn decode_single_anchor_maps_to_original_pixels() {
        let det = one_anchor(0, 320.0, 300.0, 100.0, 120.0, 0.9);
        let heads = decode_anchors(&det, ID_LB, 0.25, 0.5);
        assert_eq!(heads.len(), 1);
        let h = heads[0];
        assert!((h.cx - 320.0).abs() < 1e-3);
        assert!((h.cy - 300.0).abs() < 1e-3);
        assert!((h.width - 100.0).abs() < 1e-3);
        assert!((h.height - 120.0).abs() < 1e-3);
        assert!((h.confidence - 0.9).abs() < 1e-6);
    }

    #[test]
    fn decode_respects_letterbox_undo() {
        // Half-scale + 40px x-pad: original = (model - pad) / scale.
        let lb = Letterbox {
            scale: 0.5,
            pad_x: 40.0,
            pad_y: 0.0,
        };
        let det = one_anchor(7, 240.0, 100.0, 50.0, 60.0, 0.8);
        let heads = decode_anchors(&det, lb, 0.25, 0.5);
        assert_eq!(heads.len(), 1);
        // cx: (240-40)/0.5 = 400 ; cy: (100-0)/0.5 = 200 ; w: 50/0.5 = 100.
        assert!((heads[0].cx - 400.0).abs() < 1e-3);
        assert!((heads[0].cy - 200.0).abs() < 1e-3);
        assert!((heads[0].width - 100.0).abs() < 1e-3);
    }

    #[test]
    fn decode_thresholds_out_low_scores() {
        let det = one_anchor(0, 320.0, 320.0, 80.0, 80.0, 0.10);
        assert!(decode_anchors(&det, ID_LB, 0.25, 0.5).is_empty());
    }

    #[test]
    fn nms_suppresses_overlapping_lower_score() {
        // Two near-identical boxes, different scores → one survivor.
        let mut det = one_anchor(0, 320.0, 320.0, 100.0, 100.0, 0.9);
        let other = one_anchor(1, 322.0, 321.0, 100.0, 100.0, 0.6);
        for c in 0..DET_CHANNELS {
            det[c * NUM_ANCHORS + 1] = other[c * NUM_ANCHORS + 1];
        }
        let heads = decode_anchors(&det, ID_LB, 0.25, 0.5);
        assert_eq!(heads.len(), 1);
        assert!((heads[0].confidence - 0.9).abs() < 1e-6);
    }

    #[test]
    fn iou_self_is_one_disjoint_is_zero() {
        let a = (0.0, 0.0, 10.0, 10.0);
        assert!((iou(a, a) - 1.0).abs() < 1e-6);
        let b = (100.0, 0.0, 110.0, 10.0);
        assert_eq!(iou(a, b), 0.0);
    }
}
