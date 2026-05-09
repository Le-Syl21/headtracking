//! Face & hand detection over vendored ONNX models run through
//! `tract`. Pure Rust, no native deps.
//!
//! The crate is named `face` for historical reasons; with the
//! BlazePalm scaffold under [`hand`] it now hosts both detectors.
//! Renaming to `detect` is on the table once the hand path stabilises.
//!
//! We tried YuNet first — bigger landmark coverage, but tract 0.21
//! doesn't analyse its first stride-2 Conv correctly (`Failed analyse for
//! node Conv_0 ConvHir`, shape unification rejects 320×320 → 320×320 vs
//! the model's declared 160×160). The Ultra-Light-Fast-Generic-Face-
//! Detector RFB-320 is the next best thing — slightly fewer features
//! (no landmarks, just bbox + score) but the tract path is verified.
//!
//! Reference port: <https://github.com/sgasse/infercam_onnx>.
//! Model file: <https://github.com/onnx/models/blob/main/validated/vision/body_analysis/ultraface/models/version-RFB-320.onnx>
//!
//! Outputs of RFB-320:
//! * `scores`: `[1, N, 2]` post-softmax classes (background, face).
//! * `boxes`:  `[1, N, 4]` `(x1, y1, x2, y2)` in normalised [0, 1] coords.
//!
//! The `N=4420` anchors are baked into the graph, so all the post-
//! processing we owe is: filter by score, scale to image pixels, NMS.

pub mod hand;

use std::cmp::Ordering;
use std::io::Cursor;
use std::sync::Arc;

use image::imageops::FilterType;
use image::{ImageBuffer, Rgb};
use tract_onnx::prelude::*;

const MODEL_BYTES: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/models/version-RFB-320.onnx"
));

/// Ultraface RFB-320's expected input. Width × height (note the 4:3 aspect).
const MODEL_W: usize = 320;
const MODEL_H: usize = 240;

/// Mean / scale used at training time. Output = (pixel - MEAN) / SCALE.
const PIXEL_MEAN: f32 = 127.0;
const PIXEL_SCALE: f32 = 128.0;

const DEFAULT_SCORE_THRESHOLD: f32 = 0.7;
const DEFAULT_NMS_THRESHOLD: f32 = 0.3;

/// One detected face in the input image's pixel coordinates.
///
/// Ultraface only emits a bounding box + score. We fake the five
/// anatomical landmark fields with bbox-relative anchor points so the
/// downstream consumers (headtracking-demo, the plugin webcam tracker) can
/// keep using the same struct as YuNet would have produced.
#[derive(Debug, Clone, Copy, Default)]
pub struct FaceDetection {
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
    pub confidence: f32,
    pub right_eye_x: f32,
    pub right_eye_y: f32,
    pub left_eye_x: f32,
    pub left_eye_y: f32,
    pub nose_x: f32,
    pub nose_y: f32,
    pub mouth_right_x: f32,
    pub mouth_right_y: f32,
    pub mouth_left_x: f32,
    pub mouth_left_y: f32,
}

type RunModel = SimplePlan<TypedFact, Box<dyn TypedOp>, Graph<TypedFact, Box<dyn TypedOp>>>;

pub struct Detector {
    model: Arc<RunModel>,
    scores_idx: usize,
    boxes_idx: usize,
    score_threshold: f32,
    nms_threshold: f32,
}

impl Detector {
    pub fn new() -> Result<Self, Error> {
        let mut cursor = Cursor::new(MODEL_BYTES);
        let infered = tract_onnx::onnx()
            .model_for_read(&mut cursor)
            .map_err(|e| Error::ModelLoad(format!("model_for_read: {e}")))?
            .with_input_fact(0, f32::fact([1, 3, MODEL_H, MODEL_W]).into())
            .map_err(|e| Error::ModelLoad(format!("with_input_fact: {e}")))?;
        let typed = infered
            .into_optimized()
            .map_err(|e| Error::ModelLoad(format!("into_optimized: {e}")))?;
        let (scores_idx, boxes_idx) = locate_outputs(&typed)?;
        let runnable = typed
            .into_runnable()
            .map_err(|e| Error::ModelLoad(format!("into_runnable: {e}")))?;
        Ok(Self {
            model: Arc::new(runnable),
            scores_idx,
            boxes_idx,
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            nms_threshold: DEFAULT_NMS_THRESHOLD,
        })
    }

    pub fn set_score_threshold(&mut self, threshold: f32) {
        self.score_threshold = threshold.clamp(0.0, 1.0);
    }

    pub fn set_nms_threshold(&mut self, threshold: f32) {
        self.nms_threshold = threshold.clamp(0.0, 1.0);
    }

    pub fn detect(&self, rgb888: &[u8], width: u32, height: u32) -> Vec<FaceDetection> {
        if width == 0 || height == 0 {
            return Vec::new();
        }
        let expected = (width as usize) * (height as usize) * 3;
        if rgb888.len() != expected {
            return Vec::new();
        }

        // Resize the input to the model's 320×240 grid. Triangle filter is
        // the cheap quality/speed compromise.
        let buffer: ImageBuffer<Rgb<u8>, Vec<u8>> =
            ImageBuffer::from_raw(width, height, rgb888.to_vec()).expect("size validated above");
        let resized = image::imageops::resize(
            &buffer,
            MODEL_W as u32,
            MODEL_H as u32,
            FilterType::Triangle,
        );

        // HWC bytes → CHW float32 in normalised input space.
        let mut chw = vec![0.0_f32; 3 * MODEL_H * MODEL_W];
        let plane = MODEL_H * MODEL_W;
        for (idx, pixel) in resized.pixels().enumerate() {
            chw[idx] = (pixel[0] as f32 - PIXEL_MEAN) / PIXEL_SCALE;
            chw[plane + idx] = (pixel[1] as f32 - PIXEL_MEAN) / PIXEL_SCALE;
            chw[2 * plane + idx] = (pixel[2] as f32 - PIXEL_MEAN) / PIXEL_SCALE;
        }
        let input: Tensor = tract_ndarray::Array4::from_shape_vec((1, 3, MODEL_H, MODEL_W), chw)
            .expect("shape matches len")
            .into();

        let outputs = match self.model.run(tvec!(input.into())) {
            Ok(o) => o,
            Err(e) => {
                tracing::warn!(?e, "ultraface inference failed");
                return Vec::new();
            }
        };

        let scores_view = match outputs[self.scores_idx].to_array_view::<f32>() {
            Ok(t) => t,
            Err(_) => return Vec::new(),
        };
        let boxes_view = match outputs[self.boxes_idx].to_array_view::<f32>() {
            Ok(t) => t,
            Err(_) => return Vec::new(),
        };
        let scores = match scores_view.as_slice() {
            Some(s) => s,
            None => return Vec::new(),
        };
        let boxes = match boxes_view.as_slice() {
            Some(b) => b,
            None => return Vec::new(),
        };
        // scores: [1, N, 2] → flat 2N
        // boxes:  [1, N, 4] → flat 4N
        let n = scores.len() / 2;
        if boxes.len() != 4 * n {
            return Vec::new();
        }

        let mut detections: Vec<FaceDetection> = Vec::with_capacity(8);
        let img_w = width as f32;
        let img_h = height as f32;
        for i in 0..n {
            let face_score = scores[i * 2 + 1];
            if face_score < self.score_threshold {
                continue;
            }
            let x1 = boxes[i * 4] * img_w;
            let y1 = boxes[i * 4 + 1] * img_h;
            let x2 = boxes[i * 4 + 2] * img_w;
            let y2 = boxes[i * 4 + 3] * img_h;
            let bw = (x2 - x1).max(0.0);
            let bh = (y2 - y1).max(0.0);
            if bw < 1.0 || bh < 1.0 {
                continue;
            }
            // Anatomically-inspired anchor points. They aren't measured
            // landmarks (Ultraface doesn't predict them) but they let the
            // headtracking-demo overlays, IOD-based depth estimation, and any
            // downstream consumer keep the same data layout YuNet would
            // have produced. Eyes ≈ 35 % of the way down, mouth ≈ 75 %.
            let cx = x1 + bw * 0.5;
            let eye_y = y1 + bh * 0.35;
            let mouth_y = y1 + bh * 0.75;
            let nose_y = y1 + bh * 0.55;
            let right_eye_x = x1 + bw * 0.32;
            let left_eye_x = x1 + bw * 0.68;
            let mouth_right_x = x1 + bw * 0.36;
            let mouth_left_x = x1 + bw * 0.64;
            detections.push(FaceDetection {
                x: x1,
                y: y1,
                width: bw,
                height: bh,
                confidence: face_score,
                right_eye_x,
                right_eye_y: eye_y,
                left_eye_x,
                left_eye_y: eye_y,
                nose_x: cx,
                nose_y,
                mouth_right_x,
                mouth_right_y: mouth_y,
                mouth_left_x,
                mouth_left_y: mouth_y,
            });
        }

        nms(detections, self.nms_threshold)
    }
}

fn nms(mut detections: Vec<FaceDetection>, threshold: f32) -> Vec<FaceDetection> {
    detections.sort_by(|a, b| {
        b.confidence
            .partial_cmp(&a.confidence)
            .unwrap_or(Ordering::Equal)
    });
    let mut keep: Vec<FaceDetection> = Vec::new();
    for det in detections {
        if keep.iter().all(|kept| iou(&det, kept) < threshold) {
            keep.push(det);
        }
    }
    keep
}

fn iou(a: &FaceDetection, b: &FaceDetection) -> f32 {
    let x1 = a.x.max(b.x);
    let y1 = a.y.max(b.y);
    let x2 = (a.x + a.width).min(b.x + b.width);
    let y2 = (a.y + a.height).min(b.y + b.height);
    let inter_w = (x2 - x1).max(0.0);
    let inter_h = (y2 - y1).max(0.0);
    let inter = inter_w * inter_h;
    let union = a.width * a.height + b.width * b.height - inter;
    if union <= 0.0 { 0.0 } else { inter / union }
}

/// Find the slot of the `scores` and `boxes` outputs by name. ONNX models
/// are free to declare them in either order; we shouldn't hard-code.
fn locate_outputs(model: &TypedModel) -> Result<(usize, usize), Error> {
    let mut scores_idx = None;
    let mut boxes_idx = None;
    for (slot, out) in model.outputs.iter().enumerate() {
        match model.node(out.node).name.as_str() {
            "scores" => scores_idx = Some(slot),
            "boxes" => boxes_idx = Some(slot),
            _ => {}
        }
    }
    match (scores_idx, boxes_idx) {
        (Some(s), Some(b)) => Ok((s, b)),
        _ => Err(Error::ModelLoad(format!(
            "expected outputs `scores` and `boxes`, got: {:?}",
            model
                .outputs
                .iter()
                .map(|o| model.node(o.node).name.clone())
                .collect::<Vec<_>>()
        ))),
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("model load failed: {0}")]
    ModelLoad(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detector_constructs_from_embedded_model() {
        let det = Detector::new().expect("Ultraface model parses through tract");
        assert!(det.score_threshold > 0.0);
        assert!(det.nms_threshold > 0.0);
    }

    #[test]
    fn detector_returns_no_faces_on_blank_image() {
        let det = Detector::new().expect("ok");
        let blank = vec![0u8; 320 * 240 * 3];
        let faces = det.detect(&blank, 320, 240);
        assert!(
            faces.is_empty(),
            "blank image yielded {} face(s)",
            faces.len()
        );
    }

    #[test]
    fn iou_self_is_one() {
        let a = FaceDetection {
            x: 10.0,
            y: 20.0,
            width: 50.0,
            height: 50.0,
            ..Default::default()
        };
        assert!((iou(&a, &a) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn iou_disjoint_is_zero() {
        let a = FaceDetection {
            x: 0.0,
            y: 0.0,
            width: 10.0,
            height: 10.0,
            ..Default::default()
        };
        let b = FaceDetection {
            x: 100.0,
            y: 0.0,
            width: 10.0,
            height: 10.0,
            ..Default::default()
        };
        assert_eq!(iou(&a, &b), 0.0);
    }

    #[test]
    fn nms_keeps_highest_score_among_overlap() {
        let high = FaceDetection {
            x: 0.0,
            y: 0.0,
            width: 100.0,
            height: 100.0,
            confidence: 0.9,
            ..Default::default()
        };
        let low = FaceDetection {
            x: 5.0,
            y: 5.0,
            width: 100.0,
            height: 100.0,
            confidence: 0.5,
            ..Default::default()
        };
        let kept = nms(vec![low, high], 0.3);
        assert_eq!(kept.len(), 1);
        assert!((kept[0].confidence - 0.9).abs() < 1e-6);
    }
}
