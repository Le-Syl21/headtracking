//! Calibration helpers — pincab geometry and per-sensor anchoring.
//!
//! Two complementary fiducial paths:
//! * [`lockbar`] — bar-silhouette detection from depth (Kinect-only).
//!   The RGB-from-any-camera variant moved to the `lockbar-onnx`
//!   workspace crate (YOLOv11n-OBB via tract) in v0.0.21; the legacy
//!   HSV/gradient detector that used to live here was retired.
//!   known-width fiducial. Always visible during play; supersedes
//!   `lockbar` for the webcam tracker once BlazePalm is wired in.

// is itself behind the `dep:face` feature pulled by any tracker. Gate
// the module on the same union so `--no-default-features` still
// compiles (only loses the lockbar-from-hands path).
#[cfg(any(feature = "kinect-v2", feature = "kinect-v1", feature = "webcam"))]
#[cfg(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam"))]
pub mod autocalib;
pub mod lockbar;

#[cfg(any(feature = "kinect-v2", feature = "kinect-v1", feature = "webcam"))]
pub use lockbar::{LOCKBAR_WIDTH_MM, LockbarQuadRgb};
