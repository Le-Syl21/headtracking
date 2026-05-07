//! Cxx bridge to libfreenect2 (Kinect v2 driver).
//!
//! This crate is `-sys`-style: it exposes a thin opaque-pointer API plus the
//! depth-frame and intrinsics types we care about. The safe, channel-based
//! wrapper lives in the sibling `freenect2` crate.
//!
//! Threading: the C++ packet pipeline calls back on its own internal thread.
//! Our shim copies each depth frame under a mutex into an internal slot;
//! `Device::poll_depth` reads it. The Rust side is therefore free to call
//! `poll_depth` from any thread, but `start`/`stop`/`open_default` should be
//! treated as single-threaded ownership of the device.

#[cxx::bridge(namespace = "freenect2_shim")]
mod ffi {
    /// Depth frame copied out of libfreenect2's internal buffer.
    /// `data` holds `width * height` floats, each in millimeters.
    /// `0.0` denotes "no data" (out of range or low confidence).
    #[derive(Clone)]
    pub struct DepthFrame {
        pub width: u32,
        pub height: u32,
        /// `timestamp` field from libfreenect2 (units of 0.125 ms per tick).
        pub timestamp_raw: u32,
        pub data: Vec<f32>,
    }

    /// Color frame from the Kinect v2 (1920×1080, BGRX 4 bytes per pixel).
    /// libfreenect2 decodes the on-wire MJPEG via TurboJPEG transparently;
    /// we just hand the decoded buffer up.
    #[derive(Clone)]
    pub struct RgbFrame {
        pub width: u32,
        pub height: u32,
        pub timestamp_raw: u32,
        /// Row-major BGRX bytes — `width * height * 4` entries.
        pub data: Vec<u8>,
    }

    /// IR camera intrinsics (depth camera). Matches `Freenect2Device::IrCameraParams`.
    #[derive(Clone, Copy, Default)]
    pub struct IrCameraParams {
        pub fx: f32,
        pub fy: f32,
        pub cx: f32,
        pub cy: f32,
        pub k1: f32,
        pub k2: f32,
        pub k3: f32,
        pub p1: f32,
        pub p2: f32,
    }

    unsafe extern "C++" {
        include!("shim.h");

        /// Wraps `libfreenect2::Freenect2`.
        type Freenect2Ctx;

        /// Wraps a `libfreenect2::Freenect2Device*` plus our depth FrameListener.
        type Freenect2Dev;

        /// Construct a libfreenect2 context. Cheap; does not yet enumerate.
        fn new_context() -> UniquePtr<Freenect2Ctx>;

        /// Scan USB for Kinect v2 devices. Returns the count.
        fn enumerate(ctx: Pin<&mut Freenect2Ctx>) -> i32;

        /// Open the first Kinect v2 with the CPU packet pipeline. Returns a
        /// null `UniquePtr` if no device is available or opening fails.
        fn open_default(ctx: Pin<&mut Freenect2Ctx>) -> UniquePtr<Freenect2Dev>;

        /// Start the depth stream (RGB stays off — the head-tracker path).
        fn start_depth(dev: Pin<&mut Freenect2Dev>) -> bool;

        /// Start the requested streams. Useful for diagnostic tools that
        /// need both RGB and depth (e.g. `headtracking-demo`).
        fn start_streams(dev: Pin<&mut Freenect2Dev>, rgb: bool, depth: bool) -> bool;

        /// Stop the device. Idempotent.
        fn stop_device(dev: Pin<&mut Freenect2Dev>) -> bool;

        /// Read the most recent depth frame, if any. Returns `false` if no new
        /// frame has arrived since the last call.
        fn poll_depth(dev: Pin<&mut Freenect2Dev>, out: &mut DepthFrame) -> bool;

        /// Read the most recent color frame, if any.
        fn poll_rgb(dev: Pin<&mut Freenect2Dev>, out: &mut RgbFrame) -> bool;

        /// IR / depth camera intrinsics, available after the device starts.
        fn ir_params(dev: &Freenect2Dev) -> IrCameraParams;
    }
}

pub use ffi::{DepthFrame, Freenect2Ctx, Freenect2Dev, IrCameraParams, RgbFrame};
pub use ffi::{
    enumerate, ir_params, new_context, open_default, poll_depth, poll_rgb, start_depth,
    start_streams, stop_device,
};

// SAFETY: libfreenect2 spawns its own internal worker threads; the
// Rust-visible handles are only ever touched from a single Rust thread
// (the tracker thread, in our typical use). Moving them between threads
// is safe as long as we don't call into the shim concurrently, which is
// enforced by the safe wrapper's `Mutex<UniquePtr<...>>`.
unsafe impl Send for ffi::Freenect2Ctx {}
unsafe impl Send for ffi::Freenect2Dev {}
