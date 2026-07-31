//! Person silhouette segmentation — MediaPipe Selfie Segmentation ("general",
//! 256×256) run through pure-Rust `tract`. Produces a *coarse* binary person
//! mask (foreground = 255) to feed a silhouette-based skeleton tracker
//! (`skeleton-depth`) from a plain webcam, on **any** background and robust to
//! lighting — the depth-less counterpart to the Kinect near-slab silhouette.
//!
//! We only need a rough silhouette, not a clean matte: the downstream skeleton
//! method (largest region → thinning → topmost/outermost points) tolerates
//! ragged edges. So the smallest model wins — MediaPipe Selfie is 454 KB.
//!
//! Model: `onnx-community/mediapipe_selfie_segmentation` (Apache-2.0). Input
//! `pixel_values [1,3,256,256]` RGB in `[0,1]` (plain resize, ÷255, no mean/std);
//! output `alphas [1,1,256,256]`, already sigmoid-activated in `[0,1]`.

use std::io::Read;
use std::path::Path;
use std::sync::Arc;

use image::imageops::FilterType;
use image::{ImageBuffer, Rgb};
use tract_onnx::prelude::*;

/// Model input side (square). MediaPipe Selfie "general".
pub const MODEL_SIDE: usize = 256;

/// Default person-probability threshold: alpha above this is foreground.
pub const DEFAULT_THRESHOLD: f32 = 0.5;

type RunModel = TypedRunnableModel;

/// A binary person silhouette at [`MODEL_SIDE`]×[`MODEL_SIDE`], row-major
/// (`255` = person, `0` = background). The model uses a plain aspect-distorting
/// resize, so map joint pixels back to the source frame with `src_dim /
/// MODEL_SIDE` **per axis**.
pub struct Silhouette {
    pub side: usize,
    pub data: Vec<u8>,
}

/// Loaded segmenter. Build once (loads the embedded model), reuse per frame.
pub struct Segmenter {
    model: Arc<RunModel>,
}

/// The bundled MediaPipe Selfie Segmentation model, embedded so one shipped
/// `.so` pins exactly one model version (no separate file to sync at deploy).
const MODEL_BYTES: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/models/selfie_seg.onnx"
));

impl Segmenter {
    /// Load the embedded default model — the plugin's normal entry point.
    pub fn new() -> Result<Self, Error> {
        Self::from_reader(&mut std::io::Cursor::new(MODEL_BYTES))
    }

    /// Load a selfie-segmentation ONNX from a file path.
    pub fn from_path(path: impl AsRef<Path>) -> Result<Self, Error> {
        let mut f = std::fs::File::open(path)?;
        Self::from_reader(&mut f)
    }

    /// Load from any reader. The graph must take `[1, 3, 256, 256]` f32 and
    /// return a single-channel `[1, 1, 256, 256]` alpha in `[0, 1]`.
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
        Ok(Self { model: runnable })
    }

    /// Segment an RGB888 image (`width`×`height`, 3 bytes/pixel) into a binary
    /// person silhouette. `threshold` in `[0, 1]` (see [`DEFAULT_THRESHOLD`]).
    /// Returns an all-background mask on bad input or inference failure.
    #[must_use]
    pub fn silhouette(&self, rgb888: &[u8], width: u32, height: u32, threshold: f32) -> Silhouette {
        let n = MODEL_SIDE * MODEL_SIDE;
        let needed = (width as usize) * (height as usize) * 3;
        if width == 0 || height == 0 || rgb888.len() < needed {
            return Silhouette {
                side: MODEL_SIDE,
                data: vec![0u8; n],
            };
        }

        // Plain resize to 256×256 (aspect-distorting, matching the preprocessor).
        let buf: ImageBuffer<Rgb<u8>, Vec<u8>> =
            match ImageBuffer::from_raw(width, height, rgb888[..needed].to_vec()) {
                Some(b) => b,
                None => {
                    return Silhouette {
                        side: MODEL_SIDE,
                        data: vec![0u8; n],
                    };
                }
            };
        let resized = image::imageops::resize(
            &buf,
            MODEL_SIDE as u32,
            MODEL_SIDE as u32,
            FilterType::Triangle,
        );

        // NCHW float in [0,1]: three contiguous R/G/B planes.
        let mut input = vec![0f32; 3 * n];
        for y in 0..MODEL_SIDE {
            for x in 0..MODEL_SIDE {
                let p = resized.get_pixel(x as u32, y as u32);
                let o = y * MODEL_SIDE + x;
                input[o] = f32::from(p[0]) / 255.0;
                input[n + o] = f32::from(p[1]) / 255.0;
                input[2 * n + o] = f32::from(p[2]) / 255.0;
            }
        }
        let tensor: Tensor =
            match tract_ndarray::Array4::from_shape_vec((1, 3, MODEL_SIDE, MODEL_SIDE), input) {
                Ok(a) => a.into(),
                Err(_) => {
                    return Silhouette {
                        side: MODEL_SIDE,
                        data: vec![0u8; n],
                    };
                }
            };

        let outputs = match self.model.run(tvec!(tensor.into())) {
            Ok(o) => o,
            Err(e) => {
                tracing::warn!("personseg inference failed: {e}");
                return Silhouette {
                    side: MODEL_SIDE,
                    data: vec![0u8; n],
                };
            }
        };
        let alpha = match outputs[0].to_plain_array_view::<f32>() {
            Ok(a) => a,
            Err(_) => {
                return Silhouette {
                    side: MODEL_SIDE,
                    data: vec![0u8; n],
                };
            }
        };
        let data: Vec<u8> = alpha
            .iter()
            .map(|&a| if a >= threshold { 255 } else { 0 })
            .collect();
        Silhouette {
            side: MODEL_SIDE,
            data,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("failed to load selfie-segmentation model: {0}")]
    ModelLoad(String),
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_model_loads_and_runs() {
        let seg = Segmenter::new().expect("embedded selfie_seg.onnx loads through tract");
        // Exercise the full path on a blank frame; a blank frame yields no person.
        let s = seg.silhouette(&vec![0u8; 64 * 64 * 3], 64, 64, DEFAULT_THRESHOLD);
        assert_eq!(s.side, MODEL_SIDE);
        assert_eq!(s.data.len(), MODEL_SIDE * MODEL_SIDE);
        assert!(s.data.iter().all(|&v| v == 0 || v == 255));
    }

    #[test]
    fn bad_input_is_all_background() {
        let seg = Segmenter::new().unwrap();
        let s = seg.silhouette(&[], 0, 0, DEFAULT_THRESHOLD);
        assert!(s.data.iter().all(|&v| v == 0));
    }
}
