//! Calibration helpers — pincab geometry and per-sensor anchoring.

pub mod lockbar;

pub use lockbar::{LockbarObservation, LockbarParams, detect_lockbar};
