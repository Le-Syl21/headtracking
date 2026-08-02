//! Calibration helpers — pincab geometry and per-sensor anchoring.
//!
//! Two complementary fiducial paths:
//! * [`lockbar`] — bar-silhouette detection from depth (Kinect-only).
//!   The RGB-from-any-camera variant moved to the `lockbar-onnx`
//!   workspace crate (YOLOv11n-OBB via tract) in v0.0.21; the legacy
//!   HSV/gradient detector that used to live here was retired.
//! * [`hand_fiducial`] — 2D detection of the player's hands as a
//!   known-width fiducial. Always visible during play; supersedes
//!   `lockbar` for the webcam tracker once BlazePalm is wired in.

// `hand_fiducial` consumes types from the `face` workspace crate, which
// is itself behind the `dep:face` feature pulled by any tracker. Gate
// the module on the same union so `--no-default-features` still
// compiles (only loses the lockbar-from-hands path).
#[cfg(any(feature = "kinect-v2", feature = "kinect-v1", feature = "webcam"))]
pub mod autocalib;
pub mod hand_fiducial;
pub mod lockbar;

#[cfg(any(feature = "kinect-v2", feature = "kinect-v1", feature = "webcam"))]
pub use hand_fiducial::{HandFiducialFrame, HandPair, LockbarGeometry, observe};
pub use lockbar::{
    LOCKBAR_WIDTH_MM, LockbarObservation, LockbarParams, LockbarQuadRgb, detect_lockbar,
};
