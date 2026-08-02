//! Safe wrapper around `freenect2-sys` (libfreenect2 / Kinect v2).
//!
//! Three-piece API:
//!
//!   * [`Context`]: enumerate devices, open the first one.
//!   * [`Device`]: start/stop the depth stream, query intrinsics, poll frames.
//!   * [`DepthFrame`]: borrowed view of the latest depth buffer (millimeters).
//!
//! Threading model: libfreenect2's packet pipeline calls back on its own
//! internal thread, but our shim copies each frame under a mutex into an
//! atomic-flagged slot. [`Device::poll_depth`] is therefore safe to call
//! from any thread; the typical layout is:
//!
//! ```text
//!   tracker thread               render thread
//!   ┌──────────────┐             ┌────────────────────┐
//!   │ open_default │             │ ArcSwap<Pose>      │
//!   │   loop {     │   pose      │  load() each frame │
//!   │    poll_depth│ ───────────►│  apply to ViewSetup│
//!   │    blob algo │             │                    │
//!   │   }          │             │                    │
//!   └──────────────┘             └────────────────────┘
//! ```

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use cxx::UniquePtr;
use parking_lot::Mutex;

use freenect2_sys as sys;
pub use freenect2_sys::{DepthFrame, IrCameraParams, IrFrame, RgbFrame, take_last_log_error};

/// Depth↔color registration model (IR-vs-RGB parallax correction).
///
/// Wraps libfreenect2's `Registration`, built from the device's factory IR +
/// color intrinsics. Use [`Registration::map_depth_to_color`] to place a depth
/// pixel onto the 1920×1080 color image accurately — the Kinect v2's IR and
/// color sensors sit ~5 cm apart with different fields of view, so a naive
/// resolution-ratio scale is visibly off (worse as the subject nears the
/// camera). `apply` is pure math, so this is `Send + Sync`.
pub struct Registration {
    inner: UniquePtr<sys::Registration>,
}

impl Registration {
    /// Map a depth pixel `(dx, dy)` at depth `dz` millimeters onto the color
    /// image. Returns `Some((cx, cy))` in color-frame pixels, or `None` when
    /// the point has no valid color mapping (out of the color frustum, or the
    /// registration was built before the device streamed its camera params).
    pub fn map_depth_to_color(&self, dx: u32, dy: u32, dz: f32) -> Option<(f32, f32)> {
        let reg = self.inner.as_ref()?;
        let p = sys::map_depth_to_color(reg, dx as i32, dy as i32, dz);
        p.valid.then_some((p.x, p.y))
    }
}

/// Top-level libfreenect2 context. Construct one per process; opening multiple
/// devices off the same context is supported by libfreenect2 itself.
pub struct Context {
    inner: Mutex<UniquePtr<sys::Freenect2Ctx>>,
}

/// Install our Rust-forwarding logger as libfreenect2's global logger
/// the first time any `Context` is built. Default cap is `Info` to match
/// libfreenect2's own default; `LIBFREENECT2_LOGGER_LEVEL=Debug` (the same
/// env var the upstream ConsoleLogger honours) bumps it to Debug for
/// debugging USB enumeration / packet pipeline issues. We deliberately
/// open the floodgates at the C++ side and let Rust's `tracing` env
/// filter (`HEADTRACKING_LOG=libfreenect2=debug`) do the runtime
/// filtering — single config point.
fn ensure_logger_installed() {
    use std::sync::Once;
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        let level: u32 = std::env::var("LIBFREENECT2_LOGGER_LEVEL")
            .ok()
            .and_then(|s| match s.to_ascii_lowercase().trim() {
                "none" => Some(0),
                "error" => Some(1),
                "warning" | "warn" => Some(2),
                "info" => Some(3),
                "debug" => Some(4),
                _ => None,
            })
            .unwrap_or(3);
        sys::install_logger(level);
    });
}

impl Context {
    /// Build a fresh libfreenect2 context. Cheap; does not touch USB yet.
    pub fn new() -> Result<Self, Error> {
        ensure_logger_installed();
        let ctx = sys::new_context();
        if ctx.is_null() {
            return Err(Error::ContextInit);
        }
        Ok(Self {
            inner: Mutex::new(ctx),
        })
    }

    /// Scan for plugged-in Kinect v2 devices. Returns the count.
    pub fn enumerate(&self) -> i32 {
        let mut guard = self.inner.lock();
        sys::enumerate(guard.pin_mut())
    }

    /// Open the first detected Kinect v2 with the CPU packet pipeline.
    /// Returns `Err(Error::NoDevice)` if none are present, or
    /// `Err(Error::OpenFailed)` if libfreenect2 declines to open it.
    pub fn open_default(&self) -> Result<Device, Error> {
        let mut guard = self.inner.lock();
        let dev = sys::open_default(guard.pin_mut());
        if dev.is_null() {
            return Err(Error::OpenFailed);
        }
        Ok(Device {
            inner: Mutex::new(dev),
            running: Arc::new(AtomicBool::new(false)),
        })
    }
}

/// A single Kinect v2 device. Drop stops the stream automatically.
pub struct Device {
    inner: Mutex<UniquePtr<sys::Freenect2Dev>>,
    running: Arc<AtomicBool>,
}

impl Device {
    /// Begin streaming depth. Idempotent: returns `Ok` if already started.
    pub fn start(&self) -> Result<(), Error> {
        self.start_streams(false, true)
    }

    /// Begin the requested streams. Used by diagnostic tools that need both
    /// the color and the depth feed; the head-tracker path uses [`Device::start`]
    /// (depth only) instead.
    pub fn start_streams(&self, rgb: bool, depth: bool) -> Result<(), Error> {
        if self.running.swap(true, Ordering::AcqRel) {
            return Ok(());
        }
        let mut guard = self.inner.lock();
        if !sys::start_streams(guard.pin_mut(), rgb, depth) {
            self.running.store(false, Ordering::Release);
            return Err(Error::StartFailed);
        }
        Ok(())
    }

    /// Stop the device. Idempotent.
    pub fn stop(&self) -> Result<(), Error> {
        if !self.running.swap(false, Ordering::AcqRel) {
            return Ok(());
        }
        let mut guard = self.inner.lock();
        if !sys::stop_device(guard.pin_mut()) {
            return Err(Error::StopFailed);
        }
        Ok(())
    }

    /// Read the latest depth frame, if any. Returns `None` if no new sample
    /// has arrived since the last call.
    pub fn poll_depth(&self) -> Option<DepthFrame> {
        let mut guard = self.inner.lock();
        let mut out = DepthFrame {
            width: 0,
            height: 0,
            timestamp_raw: 0,
            data: Vec::new(),
        };
        if sys::poll_depth(guard.pin_mut(), &mut out) {
            Some(out)
        } else {
            None
        }
    }

    /// Read the latest IR frame (512×424, f32 intensity ~0..65535), if any.
    /// Returns `None` if no new sample has arrived since the last call. IR is
    /// produced by the same depth pipeline, so it streams whenever depth does
    /// (see [`Device::start_streams`] with `depth = true`).
    pub fn poll_ir(&self) -> Option<IrFrame> {
        let mut guard = self.inner.lock();
        let mut out = IrFrame {
            width: 0,
            height: 0,
            timestamp_raw: 0,
            data: Vec::new(),
        };
        if sys::poll_ir(guard.pin_mut(), &mut out) {
            Some(out)
        } else {
            None
        }
    }

    /// Read the latest color frame (BGRX, 1920×1080), if any.
    pub fn poll_rgb(&self) -> Option<RgbFrame> {
        let mut guard = self.inner.lock();
        let mut out = RgbFrame {
            width: 0,
            height: 0,
            timestamp_raw: 0,
            data: Vec::new(),
        };
        if sys::poll_rgb(guard.pin_mut(), &mut out) {
            Some(out)
        } else {
            None
        }
    }

    /// IR camera intrinsics. Valid after [`Device::start`].
    pub fn ir_params(&self) -> IrCameraParams {
        let guard = self.inner.lock();
        sys::ir_params(&guard)
    }

    /// Build a depth↔color [`Registration`] from this device's factory
    /// intrinsics. Call after the color stream has started (see
    /// [`Device::start_streams`]) — otherwise the camera params are all-zero
    /// and every mapping comes back `None`. Cheap to build; build it once and
    /// reuse it (the intrinsics don't change while streaming).
    pub fn registration(&self) -> Registration {
        let guard = self.inner.lock();
        Registration {
            inner: sys::new_registration(&guard),
        }
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        if self.running.swap(false, Ordering::AcqRel) {
            let mut guard = self.inner.lock();
            // Best effort — log failures, but don't panic in Drop.
            if !sys::stop_device(guard.pin_mut()) {
                tracing::warn!("freenect2: stop returned false on drop");
            }
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("failed to allocate libfreenect2 context")]
    ContextInit,
    #[error("no Kinect v2 device available")]
    NoDevice,
    // libfreenect2's C++ API only signals success/failure via a null
    // pointer; we can't recover a libusb rc. The hint covers the by-far
    // most common cause we hit in the wild — Windows users with the
    // Microsoft SDK driver but no UsbDk filter / no libusbK replacement.
    #[error(
        "libfreenect2 declined to open the device — typical causes: \
         Linux missing udev rules, Windows missing UsbDk filter or libusbK \
         driver replacement (see README)"
    )]
    OpenFailed,
    #[error("libfreenect2 startStreams returned false")]
    StartFailed,
    #[error("libfreenect2 stop returned false")]
    StopFailed,
}
