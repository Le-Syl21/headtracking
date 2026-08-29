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
pub use freenect2_sys::{
    BIGDEPTH_LEN, ColorCameraParams, DepthFrame, IrCameraParams, IrFrame, RgbFrame, StreamStats,
    take_last_log_error,
};

/// Samples in one Kinect v2 depth or IR frame (512 × 424).
pub const DEPTH_LEN: usize = 512 * 424;
/// Bytes in one Kinect v2 colour frame (1920 × 1080, BGRX).
pub const COLOR_BYTES: usize = 1920 * 1080 * 4;

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

    /// Project a whole depth frame into **color space**, filling `out` with
    /// `1920 × 1082` millimetre floats — color pixel `(x, y)` reads at
    /// `out[(y + 1) * 1920 + x]`, since libfreenect2 pads one border row top
    /// and bottom. Pixels the depth camera couldn't see come back **`+inf`**,
    /// not zero, so callers must gate on `is_finite()`.
    ///
    /// This is the accurate alternative to scaling a color coordinate into the
    /// depth grid by resolution ratio: the two sensors sit ~5 cm apart with
    /// different fields of view, so the naive mapping samples the wrong pixel
    /// (increasingly so as the subject nears the camera).
    ///
    /// `rgb` is BGRX (`1920*1080*4` bytes) and `depth` is `512*424` millimetre
    /// floats, exactly as [`Device::poll_rgb`] and [`Device::poll_depth`]
    /// deliver them. `out` must be [`BIGDEPTH_LEN`] long — allocate it once and
    /// reuse it, it's ~8 MB. Returns `false` (leaving `out` untouched) if any
    /// length is wrong or the registration was built before the device had
    /// streamed its camera params.
    /// `&mut` because the shim reuses per-registration scratch planes across
    /// calls instead of reallocating ~1.7 MB each frame.
    pub fn bigdepth(&mut self, rgb: &[u8], depth: &[f32], out: &mut [f32]) -> bool {
        if self.inner.is_null() {
            return false;
        }
        sys::register_bigdepth(self.inner.pin_mut(), rgb, depth, out)
    }

    /// Colour-space depth over a small window instead of the whole frame:
    /// fills `out` (row-major, `(2*half+1)²` floats) with the same values
    /// [`Registration::bigdepth`] would have written around colour pixel
    /// `center`, `+inf` where no depth sample reaches.
    ///
    /// Use this whenever the caller only samples a neighbourhood — a head, a
    /// marker. `bigdepth` costs an 8.3 MB infinity fill, 3.3 M scattered
    /// min-writes and a 512×424 colour image nobody reads, all to answer a
    /// question about a few hundred pixels.
    ///
    /// `depth` is the raw `512*424` millimetre frame from
    /// [`Device::poll_depth_into`]; no colour frame is needed at all.
    pub fn depth_window(
        &mut self,
        depth: &[f32],
        center: (i32, i32),
        half: i32,
        out: &mut [f32],
    ) -> bool {
        if self.inner.is_null() {
            return false;
        }
        sys::depth_window_min(self.inner.pin_mut(), depth, center.0, center.1, half, out)
    }
}

impl Registration {
    /// Build the model from a recorded pair of intrinsics rather than a live
    /// device — the same maths, no Kinect on the bus. Exists so the
    /// depth↔colour projection can be exercised offline.
    #[must_use]
    pub fn from_params(ir: IrCameraParams, color: ColorCameraParams) -> Self {
        Self {
            inner: sys::new_registration_from_params(ir, color),
        }
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
    /// Which depth pipeline this device actually opened with: `"OpenCL"` or
    /// `"CPU"`.
    ///
    /// Worth logging on every open. Compiling the GPU path in does not mean
    /// it runs — no registered ICD, or no usable device, and libfreenect2
    /// falls back — and the difference is the whole story behind a Kinect v2
    /// delivering 5 fps of depth instead of 30. Reading it from a user's log
    /// beats inferring it from frame rates.
    #[must_use]
    pub fn depth_pipeline(&self) -> &'static str {
        let guard = self.inner.lock();
        let Some(dev) = guard.as_ref() else {
            return "unknown";
        };
        // SAFETY: the shim returns a pointer to a string literal with static
        // storage duration, never null.
        let raw = sys::depth_pipeline(dev);
        if raw.is_null() {
            return "unknown";
        }
        unsafe { std::ffi::CStr::from_ptr(raw) }
            .to_str()
            .unwrap_or("unknown")
    }

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

    /// Copy the latest depth frame into `out`, returning `false` if no new
    /// sample has arrived since the last call.
    ///
    /// The caller owns `out` and reuses it across frames: the shim memcpy's
    /// its slot straight into `out.data`, so a poll is one copy and — after
    /// the first frame — no allocation.
    pub fn poll_depth_into(&self, out: &mut DepthFrame) -> bool {
        let mut guard = self.inner.lock();
        if out.data.len() != DEPTH_LEN {
            out.data.resize(DEPTH_LEN, 0.0);
        }
        let mut meta = sys::FrameMeta::default();
        if !sys::poll_depth_into(guard.pin_mut(), &mut out.data, &mut meta) {
            return false;
        }
        out.width = meta.width;
        out.height = meta.height;
        out.timestamp_raw = meta.timestamp_raw;
        true
    }

    /// Same for the latest IR frame (512×424, f32 intensity ~0..65535). IR is
    /// produced by the same depth pipeline, so it streams whenever depth does
    /// (see [`Device::start_streams`] with `depth = true`).
    pub fn poll_ir_into(&self, out: &mut IrFrame) -> bool {
        let mut guard = self.inner.lock();
        if out.data.len() != DEPTH_LEN {
            out.data.resize(DEPTH_LEN, 0.0);
        }
        let mut meta = sys::FrameMeta::default();
        if !sys::poll_ir_into(guard.pin_mut(), &mut out.data, &mut meta) {
            return false;
        }
        out.width = meta.width;
        out.height = meta.height;
        out.timestamp_raw = meta.timestamp_raw;
        true
    }

    /// Same for the latest colour frame (BGRX, 1920×1080 — 8.3 MB).
    pub fn poll_rgb_into(&self, out: &mut RgbFrame) -> bool {
        let mut guard = self.inner.lock();
        if out.data.len() != COLOR_BYTES {
            out.data.resize(COLOR_BYTES, 0);
        }
        let mut meta = sys::FrameMeta::default();
        if !sys::poll_rgb_into(guard.pin_mut(), &mut out.data, &mut meta) {
            return false;
        }
        out.width = meta.width;
        out.height = meta.height;
        out.timestamp_raw = meta.timestamp_raw;
        true
    }

    /// Frames libfreenect2 delivered, and frames it delivered onto a slot we
    /// had not read yet. See [`StreamStats`] — the second number is how many
    /// frames the *application* dropped, which is the only way to tell a slow
    /// reader from a slow sensor after the fact.
    #[must_use]
    pub fn stream_stats(&self) -> StreamStats {
        let guard = self.inner.lock();
        sys::stream_stats(&guard)
    }

    /// IR camera intrinsics. Valid after [`Device::start`].
    pub fn ir_params(&self) -> IrCameraParams {
        let guard = self.inner.lock();
        sys::ir_params(&guard)
    }

    /// Color camera intrinsics. Valid after [`Device::start`]. Use these — not
    /// [`Device::ir_params`] — to deproject a point sampled from
    /// [`Registration::bigdepth`], which lives in color space.
    pub fn color_params(&self) -> ColorCameraParams {
        let guard = self.inner.lock();
        sys::color_params(&guard)
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

#[cfg(test)]
mod projection_tests {
    use super::*;

    /// Factory intrinsics off a real Kinect v2 — the values the field log
    /// printed (`fx=366.25` IR, `fx=1081.37` colour) plus a representative
    /// depth↔colour polynomial. Nothing here needs a device.
    fn params() -> (IrCameraParams, ColorCameraParams) {
        let ir = IrCameraParams {
            fx: 366.249,
            fy: 366.249,
            cx: 256.0,
            cy: 208.0,
            k1: 0.0937,
            k2: -0.2731,
            k3: 0.0919,
            p1: 0.0,
            p2: 0.0,
        };
        let color = ColorCameraParams {
            fx: 1081.37,
            fy: 1081.37,
            cx: 959.5,
            cy: 539.5,
            shift_d: 863.0,
            shift_m: 52.0,
            mx_x3y0: 0.000_433,
            mx_x0y3: 3.1e-5,
            mx_x2y1: 4.5e-5,
            mx_x1y2: 0.000_284,
            mx_x2y0: -0.000_371,
            mx_x0y2: -3.6e-5,
            mx_x1y1: 0.000_121,
            mx_x1y0: 0.6392,
            mx_x0y1: 0.0017,
            mx_x0y0: 0.0553,
            my_x3y0: 3.4e-5,
            my_x0y3: 0.000_431,
            my_x2y1: 0.000_282,
            my_x1y2: 3.9e-5,
            my_x2y0: 0.0014,
            my_x0y2: -0.0022,
            my_x1y1: -0.000_376,
            my_x1y0: -0.0021,
            my_x0y1: 0.6386,
            my_x0y0: -0.0333,
        };
        (ir, color)
    }

    /// A depth frame with structure and dropouts, so the min-per-colour-pixel
    /// filter actually has something to choose between.
    fn depth_frame() -> Vec<f32> {
        (0..DEPTH_LEN)
            .map(|i| {
                if i % 17 == 0 {
                    0.0 // no reading
                } else {
                    900.0 + ((i * 7) % 400) as f32
                }
            })
            .collect()
    }

    /// [`Registration::depth_window`] must return exactly what
    /// [`Registration::bigdepth`] holds in the same window — it is the same
    /// splat loop restricted to a neighbourhood, not an approximation of it.
    ///
    /// This is the guard on a 6x cut in per-frame cost: the head sampler asks
    /// for 289 colour pixels, and used to pay for two million.
    #[test]
    fn depth_window_matches_bigdepth() {
        let (ir, color) = params();
        let mut reg = Registration::from_params(ir, color);
        let depth = depth_frame();

        let mut big = vec![0.0f32; BIGDEPTH_LEN];
        let rgb = vec![0u8; COLOR_BYTES];
        assert!(reg.bigdepth(&rgb, &depth, &mut big), "bigdepth failed");

        let half = 8i32;
        let side = (2 * half + 1) as usize;
        let mut window = vec![0.0f32; side * side];

        // Several centres, including one far to the side where the depth
        // frustum does not reach: there both projections must agree that
        // there is nothing, which is its own thing worth checking.
        let mut finite = 0;
        for center in [(960, 540), (640, 400), (1400, 700), (200, 540)] {
            assert!(
                reg.depth_window(&depth, center, half, &mut window),
                "depth_window failed at {center:?}"
            );
            for dv in -half..=half {
                for du in -half..=half {
                    let v = center.1 + dv;
                    let u = center.0 + du;
                    // bigdepth pads one border row: colour row v is row v + 1.
                    let full = big[(v + 1) as usize * 1920 + u as usize];
                    let win = window[((dv + half) * (2 * half + 1) + (du + half)) as usize];
                    if full.is_finite() {
                        finite += 1;
                    }
                    assert_eq!(
                        full.to_bits(),
                        win.to_bits(),
                        "mismatch at {center:?} + ({du}, {dv})"
                    );
                }
            }
        }
        assert!(
            finite > 0,
            "every window came back empty — the comparison would pass on nothing"
        );
    }
}
