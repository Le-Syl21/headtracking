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

use std::cell::RefCell;

thread_local! {
    /// Most recent `Error`-level message emitted by libfreenect2 on this
    /// thread. Populated by the logger bridge; drained by
    /// [`take_last_log_error`] so callers can surface the precise C++
    /// reason (e.g. "failed to open Kinect v2: ... LIBUSB_ERROR_ACCESS")
    /// in their own `Result::Err` instead of inventing a generic string.
    ///
    /// Thread-local so a v1 open on the tracker thread can't poison a
    /// v2 open on the main thread, and so packet-pipeline worker threads
    /// that emit error logs during streaming don't overwrite the slot the
    /// caller is about to read.
    static LAST_LOG_ERROR: RefCell<Option<String>> = const { RefCell::new(None) };
}

/// Forward a libfreenect2 internal log message into Rust tracing.
/// Called from C++ on whichever thread libfreenect2 was on (USB IO,
/// packet pipeline worker, …) — `tracing` macros are thread-safe.
///
/// `level` matches `libfreenect2::Logger::Level`: 1=Error, 2=Warning,
/// 3=Info, 4=Debug. Anything else is dropped. Error messages are also
/// stashed in [`LAST_LOG_ERROR`] (per-thread) so callers can read the
/// last C++ reason verbatim — see [`take_last_log_error`].
fn freenect2_log_forward(level: u32, message: &str) {
    if level == 1 {
        LAST_LOG_ERROR.with(|cell| *cell.borrow_mut() = Some(message.to_string()));
    }
    match level {
        1 => tracing::error!(target: "libfreenect2", "{}", message),
        2 => tracing::warn!(target: "libfreenect2", "{}", message),
        3 => tracing::info!(target: "libfreenect2", "{}", message),
        4 => tracing::debug!(target: "libfreenect2", "{}", message),
        _ => {}
    }
}

/// Take and clear the most recent `Error`-level libfreenect2 message
/// observed on the calling thread. Returns `None` if no error has been
/// logged since the last `take_*`. Use this right after a libfreenect2
/// call that failed silently (e.g. `enumerate()` returning 0, or
/// `open_default()` returning a null pointer) to recover the precise
/// C++-side reason.
pub fn take_last_log_error() -> Option<String> {
    LAST_LOG_ERROR.with(|cell| cell.borrow_mut().take())
}

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

    /// IR frame from the Kinect v2 (512×424, same geometry as depth).
    /// `data` holds `width * height` floats, each an IR intensity in roughly
    /// `[0, 65535]`. Delivered on the same listener as depth.
    #[derive(Clone)]
    pub struct IrFrame {
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

    /// A depth pixel mapped onto the color image via [`map_depth_to_color`].
    /// `x`/`y` are color-frame pixel coordinates (0..1920, 0..1080); `valid`
    /// is `false` when the depth point has no color mapping (out of the color
    /// frustum, or the registration was built before camera params loaded).
    #[derive(Clone, Copy, Default)]
    pub struct ColorPixel {
        pub x: f32,
        pub y: f32,
        pub valid: bool,
    }

    extern "Rust" {
        /// Bridge symbol called from the C++ `RustLogger`. Must match the
        /// free function defined above; cxx generates the trampoline.
        fn freenect2_log_forward(level: u32, message: &str);
    }

    unsafe extern "C++" {
        include!("shim.h");

        /// Wraps `libfreenect2::Freenect2`.
        type Freenect2Ctx;

        /// Wraps a `libfreenect2::Freenect2Device*` plus our depth FrameListener.
        type Freenect2Dev;

        /// Owns a `libfreenect2::Registration` (depth↔color mapping model).
        /// Built from a started device via [`new_registration`]; used by
        /// [`map_depth_to_color`] to correct IR-vs-RGB sensor parallax.
        type Registration;

        /// Install a `RustLogger` as libfreenect2's global logger, capped
        /// at the given verbosity. `level` matches
        /// `libfreenect2::Logger::Level`: 1=Error, 2=Warning, 3=Info,
        /// 4=Debug. Replaces (and frees) any previously-installed logger.
        /// Idempotent — call once at startup.
        fn install_logger(level: u32);

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

        /// Read the most recent IR frame, if any. Returns `false` if no new
        /// frame has arrived since the last call. IR is produced by the depth
        /// pipeline, so it flows whenever the depth stream is running.
        fn poll_ir(dev: Pin<&mut Freenect2Dev>, out: &mut IrFrame) -> bool;

        /// Read the most recent color frame, if any.
        fn poll_rgb(dev: Pin<&mut Freenect2Dev>, out: &mut RgbFrame) -> bool;

        /// IR / depth camera intrinsics, available after the device starts.
        fn ir_params(dev: &Freenect2Dev) -> IrCameraParams;

        /// Build a depth↔color [`Registration`] from the device's factory
        /// intrinsics. Call after `start_streams` — before the camera params
        /// have loaded the returned registration maps nothing (all `valid=false`).
        fn new_registration(dev: &Freenect2Dev) -> UniquePtr<Registration>;

        /// Map a depth pixel `(dx, dy)` at depth `dz` (mm) onto the color image.
        /// See [`ColorPixel`]. Pure/const — safe to call from any thread.
        fn map_depth_to_color(reg: &Registration, dx: i32, dy: i32, dz: f32) -> ColorPixel;
    }
}

pub use ffi::{
    ColorPixel, DepthFrame, Freenect2Ctx, Freenect2Dev, IrCameraParams, IrFrame, Registration,
    RgbFrame,
};
pub use ffi::{
    enumerate, install_logger, ir_params, map_depth_to_color, new_context, new_registration,
    open_default, poll_depth, poll_ir, poll_rgb, start_depth, start_streams, stop_device,
};

// SAFETY: libfreenect2 spawns its own internal worker threads; the
// Rust-visible handles are only ever touched from a single Rust thread
// (the tracker thread, in our typical use). Moving them between threads
// is safe as long as we don't call into the shim concurrently, which is
// enforced by the safe wrapper's `Mutex<UniquePtr<...>>`.
unsafe impl Send for ffi::Freenect2Ctx {}
unsafe impl Send for ffi::Freenect2Dev {}
// `Registration::apply` is a const, pure-math method with no shared mutable
// state — safe to move between threads and to share by `&`.
unsafe impl Send for ffi::Registration {}
unsafe impl Sync for ffi::Registration {}
