//! Calibration helpers — pincab geometry and per-sensor anchoring.
//!
//! Two complementary fiducial paths:
//! * [`lockbar`] — depth-based detection of the bar's silhouette.
//!   Robust on Kinect, fragile on webcam (occluded by hands, lit
//!   poorly by ambient light).
//! * [`hand_fiducial`] — 2D detection of the player's hands as a
//!   known-width fiducial. Always visible during play; supersedes
//!   `lockbar` for the webcam tracker once BlazePalm is wired in.

// `hand_fiducial` consumes types from the `face` workspace crate, which
// is itself behind the `dep:face` feature pulled by any tracker. Gate
// the module on the same union so `--no-default-features` still
// compiles (only loses the lockbar-from-hands path).
#[cfg(any(feature = "kinect-v2", feature = "kinect-v1", feature = "webcam"))]
pub mod hand_fiducial;
pub mod lockbar;

#[cfg(any(feature = "kinect-v2", feature = "kinect-v1", feature = "webcam"))]
pub use hand_fiducial::{HandFiducialFrame, HandPair, LockbarGeometry, observe};
pub use lockbar::{LockbarObservation, LockbarParams, detect_lockbar};
