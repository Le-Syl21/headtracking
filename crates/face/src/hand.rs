//! Hand detection scaffold.
//!
//! **Status — 2026-05-08, scaffold only.** The API surface and the
//! calibration glue are in place, but the actual ONNX model file is
//! not yet embedded. [`HandDetector::new`] succeeds and
//! [`HandDetector::detect`] returns `Ok(Vec::new())` until a model
//! lands at `crates/face/models/blazepalm.onnx` and `MODEL_BYTES` is
//! switched on. Plumbing this stub into `headtracking-demo` and the
//! plugin tracker is safe today — they'll just observe "no hands".
//!
//! ## Why we're doing this
//!
//! The player's hands sit on the flipper buttons during play, so they
//! are an *always-visible* fiducial that locates the cabinet lockbar
//! at known world width (~660 mm hand-to-hand on a standard widebody).
//! Combined with face detection, the pair gives us focal length,
//! horizon and metric scale without any explicit calibration step.
//! See `project_lockbar_calibration.md` in the agent memory and
//! `src/calibration/hand_fiducial.rs` for the math that consumes these
//! detections.
//!
//! ## Picked model
//!
//! **BlazePalm** (MediaPipe, Apache-2.0). It's the palm-detection
//! stage of the MediaPipe Hands pipeline — same family as BlazeFace
//! we already integrate, so we can share preprocessing patterns. The
//! 256×256 variant runs in ~5 ms on CPU. We don't need the downstream
//! 21-keypoint landmark model: a centroid is enough for the lockbar
//! fiducial use case.
//!
//! Source ONNX (TFLite → ONNX export):
//! <https://github.com/onnx/models> — to be vendored under
//! `crates/face/models/blazepalm.onnx` once we've validated tract can
//! analyse the graph (BlazeFace had issues — see the `lib.rs`
//! docstring — so this is a non-zero risk).

use std::sync::Arc;

use tract_onnx::prelude::*;

use crate::Error;

// ============================================================ Public types

/// One detected hand in the input image's pixel coordinates.
///
/// We carry the centroid (x, y) and a bbox so callers can draw an
/// overlay; the calibration math only consumes the centroid.
#[derive(Debug, Clone, Copy, Default)]
pub struct HandDetection {
    /// Centroid x in source-image pixels.
    pub center_x: f32,
    /// Centroid y in source-image pixels.
    pub center_y: f32,
    /// Bounding box for visualisation (source-image pixels).
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
    /// Detector-reported confidence in `[0, 1]`.
    pub confidence: f32,
}

impl HandDetection {
    /// Pixel midpoint of the bbox — handy when callers only have x/y/w/h.
    pub fn from_bbox(x: f32, y: f32, width: f32, height: f32, confidence: f32) -> Self {
        Self {
            center_x: x + width * 0.5,
            center_y: y + height * 0.5,
            x,
            y,
            width,
            height,
            confidence,
        }
    }
}

// ============================================================ Detector

/// Place-holder for the embedded BlazePalm ONNX bytes. Compiled to
/// `None` until we drop the model file in. Switch to:
///
/// ```ignore
/// const MODEL_BYTES: Option<&[u8]> = Some(include_bytes!(concat!(
///     env!("CARGO_MANIFEST_DIR"),
///     "/models/blazepalm.onnx"
/// )));
/// ```
///
/// once the model is vendored.
const MODEL_BYTES: Option<&[u8]> = None;

/// BlazePalm input is 256×256 RGB. Pre-processing follows the original
/// MediaPipe pipeline: scale to [-1, 1] via `(pixel / 127.5) - 1.0`.
#[allow(dead_code)]
const MODEL_SIZE: usize = 256;
#[allow(dead_code)]
const PIXEL_OFFSET: f32 = 127.5;
#[allow(dead_code)]
const DEFAULT_SCORE_THRESHOLD: f32 = 0.6;
#[allow(dead_code)]
const DEFAULT_NMS_THRESHOLD: f32 = 0.3;

#[allow(dead_code)]
type RunModel = TypedRunnableModel;

/// Hand detector instance. Cheap to construct (one ONNX graph load),
/// expected to be reused across frames.
pub struct HandDetector {
    /// `Some` when a model is embedded, `None` for the scaffold build.
    /// All public methods early-return when `inner` is `None`.
    inner: Option<Inner>,
    score_threshold: f32,
    nms_threshold: f32,
}

#[allow(dead_code)]
struct Inner {
    model: Arc<RunModel>,
}

impl HandDetector {
    /// Construct a detector. Returns `Ok(detector_with_no_model)` when
    /// the model isn't embedded yet — the public API stays consistent
    /// so callers can plumb the type through without `cfg` gates.
    pub fn new() -> Result<Self, Error> {
        let inner = match MODEL_BYTES {
            None => {
                tracing::warn!(
                    "BlazePalm model not embedded yet — hand detection disabled. \
                     Calibration via lockbar fiducial-from-hands will fall back \
                     to the depth-based detector."
                );
                None
            }
            Some(_bytes) => {
                // TODO: when the model is in place, mirror the
                // BlazeFace path:
                //   let model = tract_onnx::onnx()
                //       .model_for_read(&mut Cursor::new(bytes))?
                //       .into_optimized()?
                //       .into_runnable()?;
                //   Some(Inner { model: Arc::new(model) })
                None
            }
        };
        Ok(Self {
            inner,
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            nms_threshold: DEFAULT_NMS_THRESHOLD,
        })
    }

    /// Apply a custom confidence threshold. Builder-style.
    pub fn with_score_threshold(mut self, t: f32) -> Self {
        self.score_threshold = t;
        self
    }

    /// Apply a custom NMS overlap threshold. Builder-style.
    pub fn with_nms_threshold(mut self, t: f32) -> Self {
        self.nms_threshold = t;
        self
    }

    /// `true` when a model is loaded and detection actually runs.
    /// `false` in scaffold builds — `detect` will always return empty.
    pub fn is_ready(&self) -> bool {
        self.inner.is_some()
    }

    /// Run detection over an RGB888 frame (`width * height * 3` bytes,
    /// row-major, channel order R,G,B).
    ///
    /// In scaffold mode (no model embedded), returns `Ok(vec![])`.
    pub fn detect(
        &self,
        rgb888: &[u8],
        width: u32,
        height: u32,
    ) -> Result<Vec<HandDetection>, Error> {
        let Some(_inner) = &self.inner else {
            return Ok(Vec::new());
        };
        // TODO: implement once the model lands.
        //   1. Resize `rgb888` to MODEL_SIZE × MODEL_SIZE letterbox.
        //   2. Normalise to [-1, 1].
        //   3. Run `inner.model.run(...)`.
        //   4. Decode anchors (BlazePalm SSD-style, 2944 anchors at
        //      256×256, scales {0.07, 0.79, 0.84, ...}).
        //   5. Apply score threshold + NMS.
        //   6. Map normalised coords back to source-image pixels.
        let _ = (rgb888, width, height);
        Ok(Vec::new())
    }
}

impl Default for HandDetector {
    fn default() -> Self {
        Self::new().expect("HandDetector::new should always succeed in scaffold mode")
    }
}

// ============================================================ Helpers used by callers

/// Sort a pair of detected hands by image-x and return `(left, right)`
/// — i.e. the left-of-image hand (typically the player's *left* hand
/// when the camera is centred on the backbox) first.
///
/// If the slice has fewer than two entries, returns `None`. If more
/// than two, picks the highest-confidence pair.
pub fn sort_lr(mut hands: Vec<HandDetection>) -> Option<(HandDetection, HandDetection)> {
    if hands.len() < 2 {
        return None;
    }
    if hands.len() > 2 {
        hands.sort_by(|a, b| {
            b.confidence
                .partial_cmp(&a.confidence)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        hands.truncate(2);
    }
    let (mut a, mut b) = (hands[0], hands[1]);
    if a.center_x > b.center_x {
        std::mem::swap(&mut a, &mut b);
    }
    Some((a, b))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sort_lr_orders_by_x() {
        let right = HandDetection {
            center_x: 800.0,
            confidence: 0.9,
            ..Default::default()
        };
        let left = HandDetection {
            center_x: 200.0,
            confidence: 0.8,
            ..Default::default()
        };
        // Submit in arbitrary order; sort_lr should always return left first.
        let (a, b) = sort_lr(vec![right, left]).unwrap();
        assert_eq!(a.center_x, 200.0);
        assert_eq!(b.center_x, 800.0);
    }

    #[test]
    fn sort_lr_returns_none_below_two() {
        assert!(sort_lr(vec![]).is_none());
        assert!(sort_lr(vec![HandDetection::default()]).is_none());
    }

    #[test]
    fn sort_lr_picks_top_two_by_confidence() {
        let h = |x, c| HandDetection {
            center_x: x,
            confidence: c,
            ..Default::default()
        };
        let (a, b) = sort_lr(vec![h(100.0, 0.5), h(900.0, 0.9), h(500.0, 0.8)]).unwrap();
        // top two by confidence are 900@0.9 and 500@0.8; left in image = 500.
        assert_eq!(a.center_x, 500.0);
        assert_eq!(b.center_x, 900.0);
    }

    #[test]
    fn from_bbox_centers_correctly() {
        let h = HandDetection::from_bbox(100.0, 200.0, 60.0, 80.0, 0.95);
        assert_eq!(h.center_x, 130.0);
        assert_eq!(h.center_y, 240.0);
    }

    #[test]
    fn scaffold_detector_returns_empty() {
        let det = HandDetector::new().unwrap();
        assert!(!det.is_ready());
        let result = det.detect(&[0u8; 12], 2, 2).unwrap();
        assert!(result.is_empty());
    }
}
