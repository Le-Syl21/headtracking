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

/// Depth frame copied out of libfreenect2's internal buffer.
/// `data` holds `width * height` floats, each in millimeters.
/// `0.0` denotes "no data" (out of range or low confidence).
///
/// Plain Rust, not a cxx shared struct: the shim copies straight into
/// `data`'s storage through a slice, so C++ never names the type.
#[derive(Clone, Default)]
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
#[derive(Clone, Default)]
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
#[derive(Clone, Default)]
pub struct RgbFrame {
    pub width: u32,
    pub height: u32,
    pub timestamp_raw: u32,
    /// Row-major BGRX bytes — `width * height * 4` entries.
    pub data: Vec<u8>,
}

#[cxx::bridge(namespace = "freenect2_shim")]
mod ffi {
    /// Geometry and timestamp of the frame a `poll_*_into` just copied out.
    /// The pixels went straight into the caller's buffer, so this is all that
    /// travels through the bridge.
    #[derive(Clone, Copy, Default)]
    pub struct FrameMeta {
        pub width: u32,
        pub height: u32,
        /// `timestamp` field from libfreenect2 (units of 0.125 ms per tick).
        pub timestamp_raw: u32,
    }

    /// Per-stream frame accounting, straight from the listener slots.
    ///
    /// `*_received` counts what libfreenect2 delivered — the sensor's own
    /// rate. `*_dropped` counts deliveries that landed on a slot the reader
    /// had not drained yet: frames the *application* threw away. The pair is
    /// the difference between "the Kinect is slow" and "we are slow", which
    /// is otherwise unknowable from a log.
    #[derive(Clone, Copy, Default)]
    pub struct StreamStats {
        pub rgb_received: u64,
        pub rgb_dropped: u64,
        pub depth_received: u64,
        pub depth_dropped: u64,
        pub ir_received: u64,
        pub ir_dropped: u64,
    }

    /// What the colour camera's own auto-exposure decided, plus its frame
    /// clock — read straight off the last colour frame libfreenect2 decoded.
    ///
    /// This is the missing half of the frame-rate story. The Kinect v2's
    /// colour camera auto-exposes, and halves to 15 Hz when the room is dark
    /// enough to need it, while the IR/depth streams hold ~30 Hz off their own
    /// illuminator. Reading `cam 14.9 | ir+depth 29.8` and having to *guess*
    /// whether the room was dim is what made those logs ambiguous.
    ///
    /// `exposure` runs from 0.5 (very bright) to about 60.0 (lens covered),
    /// `gain` from 1.0 to 1.5, `gamma` from 1.0 to 6.4 — libfreenect2 gives no
    /// unit beyond that, so treat them as a brightness index, not photometry.
    #[derive(Clone, Copy, Default)]
    pub struct ColorExposure {
        pub exposure: f32,
        pub gain: f32,
        pub gamma: f32,
        /// Step between the last two colour frames on the sensor's own clock,
        /// in units of 0.125 ms: 266 at 30 Hz, 533 at 15 Hz. `0` until two
        /// frames have arrived. Unlike everything else we print, this number
        /// is the camera's own account of its cadence.
        pub frame_step: u32,
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

    /// Color camera intrinsics — the full `Freenect2Device::ColorCameraParams`,
    /// pinhole terms plus the depth-to-color polynomial libfreenect2 fits per
    /// unit. Valid once the device has started streaming.
    ///
    /// Callers that only deproject want `fx/fy/cx/cy`; the rest is carried so
    /// a [`Registration`] can be rebuilt from a recorded calibration with no
    /// device attached — which is what makes the colour-space projection
    /// testable at all.
    #[derive(Clone, Copy, Default)]
    pub struct ColorCameraParams {
        pub fx: f32,
        pub fy: f32,
        pub cx: f32,
        pub cy: f32,
        pub shift_d: f32,
        pub shift_m: f32,
        pub mx_x3y0: f32,
        pub mx_x0y3: f32,
        pub mx_x2y1: f32,
        pub mx_x1y2: f32,
        pub mx_x2y0: f32,
        pub mx_x0y2: f32,
        pub mx_x1y1: f32,
        pub mx_x1y0: f32,
        pub mx_x0y1: f32,
        pub mx_x0y0: f32,
        pub my_x3y0: f32,
        pub my_x0y3: f32,
        pub my_x2y1: f32,
        pub my_x1y2: f32,
        pub my_x2y0: f32,
        pub my_x0y2: f32,
        pub my_x1y1: f32,
        pub my_x1y0: f32,
        pub my_x0y1: f32,
        pub my_x0y0: f32,
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
        fn open_default(ctx: Pin<&mut Freenect2Ctx>, allow_gpu: bool) -> UniquePtr<Freenect2Dev>;

        /// Start the depth stream (RGB stays off — the head-tracker path).
        /// `"OpenCL"` or `"CPU"` — which depth pipeline the device opened
        /// with. The CPU one drops USB packets on a Kinect v2, so this is the
        /// first thing to read in a slow-stream report.
        fn depth_pipeline(dev: &Freenect2Dev) -> *const c_char;

        fn start_depth(dev: Pin<&mut Freenect2Dev>) -> bool;

        /// Start the requested streams. Useful for diagnostic tools that
        /// need both RGB and depth (e.g. `headtracking-demo`).
        fn start_streams(dev: Pin<&mut Freenect2Dev>, rgb: bool, depth: bool) -> bool;

        /// Stop the device. Idempotent.
        fn stop_device(dev: Pin<&mut Freenect2Dev>) -> bool;

        /// Copy the most recent depth frame into `out`, which the caller owns
        /// and reuses. Returns `false` if no new frame has arrived since the
        /// last call, or if `out` is too small for it.
        fn poll_depth_into(
            dev: Pin<&mut Freenect2Dev>,
            out: &mut [f32],
            meta: &mut FrameMeta,
        ) -> bool;

        /// Same for the most recent IR frame. IR is produced by the depth
        /// pipeline, so it flows whenever the depth stream is running.
        fn poll_ir_into(dev: Pin<&mut Freenect2Dev>, out: &mut [f32], meta: &mut FrameMeta)
        -> bool;

        /// Same for the most recent colour frame (BGRX, `1920*1080*4` bytes).
        fn poll_rgb_into(dev: Pin<&mut Freenect2Dev>, out: &mut [u8], meta: &mut FrameMeta)
        -> bool;

        /// Frames delivered and frames dropped, per stream. See [`StreamStats`].
        fn stream_stats(dev: &Freenect2Dev) -> StreamStats;
        fn color_exposure(dev: &Freenect2Dev) -> ColorExposure;
        fn set_color_auto_exposure(dev: Pin<&mut Freenect2Dev>, compensation: f32);
        fn set_color_semi_auto_exposure(dev: Pin<&mut Freenect2Dev>, pseudo_ms: f32);
        fn set_color_manual_exposure(
            dev: Pin<&mut Freenect2Dev>,
            integration_ms: f32,
            analog_gain: f32,
        );

        /// IR / depth camera intrinsics, available after the device starts.
        fn ir_params(dev: &Freenect2Dev) -> IrCameraParams;

        /// Color camera intrinsics, available after the device starts. Needed
        /// to deproject a point sampled from the *color-space* depth map
        /// produced by [`register_bigdepth`].
        fn color_params(dev: &Freenect2Dev) -> ColorCameraParams;

        /// Build a depth↔color [`Registration`] from the device's factory
        /// intrinsics. Call after `start_streams` — before the camera params
        /// have loaded the returned registration maps nothing (all `valid=false`).
        fn new_registration(dev: &Freenect2Dev) -> UniquePtr<Registration>;

        /// Same model, built straight from a recorded pair of intrinsics
        /// instead of a live device. Lets the depth↔colour projection be
        /// exercised without a Kinect on the bus.
        fn new_registration_from_params(
            ir: IrCameraParams,
            color: ColorCameraParams,
        ) -> UniquePtr<Registration>;

        /// Map a depth pixel `(dx, dy)` at depth `dz` (mm) onto the color image.
        /// See [`ColorPixel`]. Pure/const — safe to call from any thread.
        fn map_depth_to_color(reg: &Registration, dx: i32, dy: i32, dz: f32) -> ColorPixel;

        /// Project the whole depth frame into **color space**: fills `bigdepth`
        /// with `1920 × 1082` millimetre floats, so color pixel `(x, y)` reads
        /// at `bigdepth[(y + 1) * 1920 + x]` (one blank border row top and
        /// bottom). Pixels with no depth come back `+inf`, *not* zero.
        ///
        /// `rgb` is BGRX `1920*1080*4` bytes, `depth` is `512*424` millimetre
        /// floats. Returns `false` — leaving `bigdepth` untouched — if the
        /// registration was built before the camera params loaded, or any
        /// buffer length is wrong.
        fn register_bigdepth(
            reg: Pin<&mut Registration>,
            rgb: &[u8],
            depth: &[f32],
            bigdepth: &mut [f32],
        ) -> bool;

        /// Colour-space depth over a `(2*half+1)²` window centred on colour
        /// pixel `(center_x, center_y)`, written row-major into `out`.
        ///
        /// Same values [`register_bigdepth`] would have left in that window —
        /// libfreenect2's nearest-z-per-colour-pixel filter map — without
        /// building the other two million pixels. Pixels no depth sample
        /// reaches come back `+inf`, as in `bigdepth`. Returns `false` on a
        /// length mismatch or a registration built before the camera params
        /// loaded.
        fn depth_window_min(
            reg: Pin<&mut Registration>,
            depth: &[f32],
            center_x: i32,
            center_y: i32,
            half: i32,
            out: &mut [f32],
        ) -> bool;
    }
}

pub use ffi::{
    ColorCameraParams, ColorExposure, ColorPixel, FrameMeta, Freenect2Ctx, Freenect2Dev,
    IrCameraParams, Registration, StreamStats,
};
pub use ffi::{
    color_exposure, color_params, depth_pipeline, depth_window_min, enumerate, install_logger,
    ir_params, map_depth_to_color, new_context, new_registration, new_registration_from_params,
    open_default, poll_depth_into, poll_ir_into, poll_rgb_into, register_bigdepth,
    set_color_auto_exposure, set_color_manual_exposure, set_color_semi_auto_exposure, start_depth,
    start_streams, stop_device, stream_stats,
};

/// Length of the color-space depth buffer [`register_bigdepth`] fills:
/// `1920 × 1082` floats (the color plane plus a one-row border top and bottom).
pub const BIGDEPTH_LEN: usize = 1920 * 1082;

// SAFETY: libfreenect2 spawns its own internal worker threads; the
// Rust-visible handles are only ever touched from a single Rust thread
// (the tracker thread, in our typical use). Moving them between threads
// is safe as long as we don't call into the shim concurrently, which is
// enforced by the safe wrapper's `Mutex<UniquePtr<...>>`.
unsafe impl Send for ffi::Freenect2Ctx {}
unsafe impl Send for ffi::Freenect2Dev {}
// The shim `Registration` now carries persistent scratch buffers, but they
// are only ever written through `register_bigdepth(Pin<&mut _>)` — shared
// `&` access (`map_depth_to_color`) stays const pure math. So moving it
// between threads is fine (in practice a single capture thread owns it),
// and sharing by `&` cannot race the scratch state, which is only reachable
// through an exclusive borrow.
unsafe impl Send for ffi::Registration {}
unsafe impl Sync for ffi::Registration {}

#[cfg(test)]
mod gpu_pipeline_tests {
    /// The Kinect v2 depth decode must be compiled with a GPU pipeline.
    ///
    /// On the CPU pipeline libfreenect2 does not merely run slower: it drops
    /// USB depth packets it cannot consume in time and delivers roughly 5 fps
    /// instead of 30, which downstream reads as a head position updating five
    /// times a second. That shipped on Windows and macOS for months because
    /// nothing checked — `ENABLE_OPENCL` is a *request*, and a build image
    /// without the SDK silently produces a CPU-only library.
    ///
    /// Set `HT_ALLOW_CPU_ONLY=1` to build without it. Install the SDK to fix
    /// it properly: `opencl-headers ocl-icd-opencl-dev` on Debian/Ubuntu,
    /// `vcpkg install opencl:x64-windows` on Windows; macOS has the framework
    /// already.
    #[test]
    fn gpu_depth_pipeline_is_compiled_in() {
        if cfg!(freenect2_opencl) {
            return;
        }
        if std::env::var_os("HT_ALLOW_CPU_ONLY").is_some() {
            eprintln!(
                "warning: libfreenect2 built without OpenCL — the Kinect v2 \
                 depth pipeline will run on the CPU and drop USB packets"
            );
            return;
        }
        panic!(
            "libfreenect2 was built WITHOUT the OpenCL depth pipeline: the \
             Kinect v2 would run its depth decode on the CPU, drop USB packets \
             and deliver ~5 fps instead of 30. Install the OpenCL SDK for this \
             platform (Debian/Ubuntu: opencl-headers ocl-icd-opencl-dev), or \
             set HT_ALLOW_CPU_ONLY=1 to accept a CPU-only build."
        );
    }
}
