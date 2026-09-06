//! Head tracker abstraction shared by Kinect v1/v2 and webcam backends.

#[cfg(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam"))]
pub mod pipeline;

#[cfg(feature = "kinect-v1")]
pub mod kinect_v1;
#[cfg(feature = "kinect-v2")]
pub mod kinect_v2;
#[cfg(feature = "webcam")]
pub mod webcam;

pub mod session;

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
/// Why a backend could not enter its tracking stream.
///
/// A cause, never a sentence. The wording belongs to whatever is showing it —
/// today `plugin::ffi`, beside the other player-facing notes — because a
/// message assembled down here could not be translated, and the driver's own
/// error text is diagnostic material for the log rather than something to put
/// in front of a player mid-game.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingFault {
    /// The video endpoint could not be handed over to infrared. The Kinect v1
    /// has exactly one, shared between colour and IR, so this means something
    /// else on the machine is holding the camera.
    IrStreamBusy,
    /// No camera opened at all — every backend in the chain declined. Head
    /// tracking cannot start, and the player should be told rather than left
    /// to wonder why the view never moves.
    NoCamera,
}

pub trait HeadTracker: Send {
    /// Pull the next available pose from the backend, if any.
    /// Returns `None` if no new sample is ready since the last call.
    fn poll(&mut self) -> Option<Pose>;

    /// Human-readable backend name (used in logs and config).
    fn name(&self) -> &'static str;

    /// Device label for user-facing notifications (e.g. the webcam's UVC
    /// product name, or the Kinect model + active stream). Defaults to
    /// [`Self::name`].
    fn device_label(&self) -> String {
        self.name().to_string()
    }

    /// Poll one rgb888 frame for the anchor-calibration phase at session
    /// start, from whichever stream the backend calibrates on -- colour for
    /// webcams, infrared on a Kinect v2 tracking in IR. Backends that can
    /// serve neither return `None`.
    fn poll_calibration_frame(&mut self) -> Option<(u32, u32, Vec<u8>)> {
        None
    }

    /// Leave the calibration phase and start the tracking stream.
    ///
    /// Called exactly once, before the first `poll`. A backend that cannot
    /// reach its tracking stream has failed, and saying so beats degrading to
    /// something that half works and gets reported as a tracking bug months
    /// later.
    fn begin_tracking(&mut self) -> Result<(), TrackingFault> {
        Ok(())
    }

    /// Intrinsics `[fx, fy, cx, cy]` of the stream [`poll_calibration_frame`]
    /// serves, when the device knows them (Kinect factory calibration).
    /// `None` = use nominals. Must match that stream, not merely the colour
    /// one: on a Kinect v2 the IR camera has its own focal length.
    fn calibration_intrinsics(&self) -> Option<[f32; 4]> {
        None
    }

    /// Which stream `poll_calibration_frame` hands over.
    ///
    /// The anchor detector carries one model per stream. A backend that gets
    /// this wrong feeds a model the distribution it was not trained on, which
    /// is the failure the split exists to prevent.
    fn calibration_stream(&self) -> anchor::Stream {
        anchor::Stream::Colour
    }

    /// Release device handles. Must be safe to call multiple times.
    fn shutdown(&mut self);
}
