//! Head tracker abstraction shared by Kinect v1/v2 and webcam backends.

#[cfg(feature = "kinect-v2")]
pub mod kinect_v2;

/// 3D head pose in the device coordinate frame, with monotonic timestamp.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// XYZ position, millimeters, in the device's own coordinate frame.
    pub position_mm: [f32; 3],
    /// Monotonic timestamp at capture (microseconds).
    pub timestamp_us: u64,
    /// Tracker confidence in `[0.0, 1.0]`. `0.0` means lost.
    pub confidence: f32,
}

impl Pose {
    pub const ZERO: Self = Self {
        position_mm: [0.0; 3],
        timestamp_us: 0,
        confidence: 0.0,
    };
}

impl Default for Pose {
    fn default() -> Self {
        Self::ZERO
    }
}

/// Common interface every backend (Kinect v1/v2, webcam, mock) must satisfy.
///
/// Backends are owned by a dedicated tracker thread spawned at plugin load.
/// The thread writes the latest `Pose` into an `arc_swap::ArcSwap<Pose>` that
/// the VPX render-thread callback reads each frame.
pub trait HeadTracker: Send {
    /// Pull the next available pose from the backend, if any.
    /// Returns `None` if no new sample is ready since the last call.
    fn poll(&mut self) -> Option<Pose>;

    /// Human-readable backend name (used in logs and config).
    fn name(&self) -> &'static str;

    /// Release device handles. Must be safe to call multiple times.
    fn shutdown(&mut self);
}
