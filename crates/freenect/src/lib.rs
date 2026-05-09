//! Safe wrapper around `freenect-sys` (libfreenect / Kinect v1).
//!
//! Unlike libfreenect2 (which spawns its own packet pipeline thread),
//! libfreenect is single-threaded: the caller has to drive the event loop
//! by calling [`freenect_process_events`][freenect-sys] continuously. This
//! crate spawns that loop in a dedicated thread when the device starts,
//! and tears it down on `stop()` / `Drop`.
//!
//! ```text
//!   user thread       libfreenect event thread          callbacks
//!   ┌──────────┐     ┌────────────────────────┐   ┌─────────────────────┐
//!   │ open     │     │ process_events_timeout │ → │ depth_cb → DepthSlot │
//!   │ start    │     │   loop until stop      │ → │ video_cb → VideoSlot │
//!   │ poll(...)│ ←── reads from the slots ────────────────────────────────┘
//!   │ stop     │
//!   └──────────┘
//! ```
//!
//! Depth mode: `FREENECT_DEPTH_MM` at 640×480 / 30 Hz, `u16` millimeter
//! depth values (0 = no data).
//! Video mode: `FREENECT_VIDEO_RGB` at 640×480 / 30 Hz, raw RGB888 (no
//! decompression — the v1 sensor doesn't push MJPEG).

use std::ffi::c_void;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use parking_lot::Mutex;
use tracing::{error, warn};

use freenect_sys as sys;

pub const DEPTH_WIDTH: u32 = 640;
pub const DEPTH_HEIGHT: u32 = 480;
pub const VIDEO_WIDTH: u32 = 640;
pub const VIDEO_HEIGHT: u32 = 480;

/// One depth frame copied out of libfreenect's internal buffer.
#[derive(Debug, Clone)]
pub struct DepthFrame {
    pub width: u32,
    pub height: u32,
    /// `timestamp` field reported by libfreenect (units depend on firmware).
    pub timestamp_raw: u32,
    /// Row-major `u16` millimeter depths. `0` = no data.
    pub data: Vec<u16>,
}

/// One color frame copied out of libfreenect's internal buffer.
/// Layout is row-major RGB888 — `width * height * 3` bytes, channel order
/// `[R, G, B]` per pixel.
#[derive(Debug, Clone)]
pub struct RgbFrame {
    pub width: u32,
    pub height: u32,
    pub timestamp_raw: u32,
    pub data: Vec<u8>,
}

/// Pinhole intrinsics for the Kinect v1 IR / depth camera. libfreenect does
/// not expose factory parameters, so the values are nominal Microsoft specs
/// (good to ~1% on a calibrated device).
pub const FX: f32 = 580.0;
pub const FY: f32 = 580.0;
pub const CX: f32 = (DEPTH_WIDTH as f32) / 2.0;
pub const CY: f32 = (DEPTH_HEIGHT as f32) / 2.0;

/// Mechanical tilt range advertised by the Kinect v1 motor (in degrees).
pub const TILT_MIN_DEG: f32 = -31.0;
pub const TILT_MAX_DEG: f32 = 31.0;

/// LED colours / blink patterns the Kinect v1 base supports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LedState {
    Off,
    Green,
    Red,
    Yellow,
    BlinkGreen,
    BlinkRedYellow,
}

impl LedState {
    fn to_raw(self) -> u32 {
        // Values from `freenect.h`. There is no value 5 — historical hole.
        match self {
            LedState::Off => 0,
            LedState::Green => 1,
            LedState::Red => 2,
            LedState::Yellow => 3,
            LedState::BlinkGreen => 4,
            LedState::BlinkRedYellow => 6,
        }
    }
}

/// Snapshot of the motor / accelerometer state.
#[derive(Debug, Clone, Copy)]
pub struct TiltState {
    /// Current tilt in degrees (zero = horizontal, positive = looking up).
    pub angle_deg: f32,
    pub status: TiltStatus,
    /// Gravity vector reported by the on-board accelerometer, in m/s².
    /// Useful to know how the cab is oriented or whether it's been moved.
    pub accel_mks: [f32; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TiltStatus {
    Stopped,
    Limit,
    Moving,
    Unknown,
}

impl TiltStatus {
    fn from_raw(raw: i32) -> Self {
        // libfreenect tilt status codes: 0 = stopped, 1 = limit, 4 = moving.
        match raw {
            0 => TiltStatus::Stopped,
            1 => TiltStatus::Limit,
            4 => TiltStatus::Moving,
            _ => TiltStatus::Unknown,
        }
    }
}

/// libfreenect context. Holds the USB context and the list of open devices.
pub struct Context {
    handle: Arc<CtxHandle>,
}

struct CtxHandle {
    raw: *mut sys::freenect_context,
    api_lock: Mutex<()>,
}

// SAFETY: libfreenect's API is single-threaded but the raw context pointer
// is opaque; we serialize access through the safe wrapper's `api_lock`.
unsafe impl Send for CtxHandle {}
unsafe impl Sync for CtxHandle {}

impl Context {
    /// Build a fresh context. Equivalent to `freenect_init(&mut ctx, NULL)`.
    pub fn new() -> Result<Self, Error> {
        let mut raw: *mut sys::freenect_context = std::ptr::null_mut();
        // SAFETY: libfreenect requires we pass a valid out-pointer; usb_ctx
        // is allowed to be null (libfreenect creates one for us).
        let rc = unsafe { sys::freenect_init(&mut raw, std::ptr::null_mut()) };
        if rc < 0 || raw.is_null() {
            return Err(Error::ContextInit(rc));
        }
        // Claim camera + motor (so we can drive the tilt and LED). Audio
        // stays off.
        // SAFETY: `raw` is non-null per the check above.
        unsafe {
            sys::freenect_select_subdevices(
                raw,
                sys::freenect_device_flags_FREENECT_DEVICE_CAMERA
                    | sys::freenect_device_flags_FREENECT_DEVICE_MOTOR,
            );
        }
        Ok(Self {
            handle: Arc::new(CtxHandle {
                raw,
                api_lock: Mutex::new(()),
            }),
        })
    }

    /// Number of Kinect v1 devices visible on USB.
    pub fn enumerate(&self) -> i32 {
        let _g = self.handle.api_lock.lock();
        // SAFETY: `raw` is valid for the lifetime of CtxHandle.
        unsafe { sys::freenect_num_devices(self.handle.raw) }
    }

    /// Open the device at `index`. Mirrors `freenect_open_device`.
    pub fn open(&self, index: i32) -> Result<Device, Error> {
        let g = self.handle.api_lock.lock();
        let mut raw: *mut sys::freenect_device = std::ptr::null_mut();
        // SAFETY: ctx is alive, `raw` is a valid out-pointer.
        let rc = unsafe { sys::freenect_open_device(self.handle.raw, &mut raw, index) };
        drop(g);
        if rc < 0 || raw.is_null() {
            return Err(Error::OpenFailed(OpenFailureCode(rc)));
        }
        Device::wrap(self.handle.clone(), raw)
    }
}

impl Drop for CtxHandle {
    fn drop(&mut self) {
        if !self.raw.is_null() {
            // SAFETY: we own raw, no other thread holds it (Arc reached zero).
            unsafe { sys::freenect_shutdown(self.raw) };
        }
    }
}

/// One Kinect v1 device. Streams depth (and optionally RGB video) via a
/// background thread that pumps libfreenect's event loop.
pub struct Device {
    // Drop order: stop the event thread first (clears callbacks), close the
    // device pointer, then release the Slots. Rust drops fields in
    // declaration order — so list event_thread first.
    event_thread: Option<JoinHandle<()>>,
    stop: Arc<AtomicBool>,
    raw: *mut sys::freenect_device,
    slots: Box<Slots>,
    started: AtomicBool,
    depth_running: bool,
    video_running: bool,
    ctx: Arc<CtxHandle>,
}

// SAFETY: same rationale as CtxHandle — the safe wrapper serializes access
// through `ctx.api_lock`, and the raw pointer is opaque.
unsafe impl Send for Device {}

impl Device {
    fn wrap(ctx: Arc<CtxHandle>, raw: *mut sys::freenect_device) -> Result<Self, Error> {
        // The Slots struct must have a stable heap address: we hand its
        // pointer to libfreenect via freenect_set_user, and both depth and
        // video callbacks retrieve it on every frame.
        let slots = Box::new(Slots::default());
        let user_ptr = (&raw const *slots).cast::<c_void>().cast_mut();

        // Configure depth (640x480 mm) and video (640x480 RGB888) modes,
        // then install both callbacks. Streams stay off until start_streams.
        // SAFETY: `raw` is a freshly opened device; the API lock keeps the
        // ctx single-threaded, the modes come from libfreenect's descriptor.
        let api = ctx.api_lock.lock();
        let setup_rc = unsafe {
            let depth_mode = sys::freenect_find_depth_mode(
                sys::freenect_resolution_FREENECT_RESOLUTION_MEDIUM,
                sys::freenect_depth_format_FREENECT_DEPTH_MM,
            );
            let video_mode = sys::freenect_find_video_mode(
                sys::freenect_resolution_FREENECT_RESOLUTION_MEDIUM,
                sys::freenect_video_format_FREENECT_VIDEO_RGB,
            );
            if depth_mode.is_valid == 0 || video_mode.is_valid == 0 {
                drop(api);
                sys::freenect_close_device(raw);
                return Err(Error::ModeUnavailable);
            }
            let rc_d = sys::freenect_set_depth_mode(raw, depth_mode);
            let rc_v = sys::freenect_set_video_mode(raw, video_mode);
            if rc_d < 0 || rc_v < 0 {
                drop(api);
                sys::freenect_close_device(raw);
                return Err(Error::ModeUnavailable);
            }
            sys::freenect_set_user(raw, user_ptr);
            sys::freenect_set_depth_callback(raw, Some(depth_callback));
            sys::freenect_set_video_callback(raw, Some(video_callback));
            0
        };
        drop(api);
        if setup_rc < 0 {
            // unreachable — the early returns above handle the failure
            // cases — kept for completeness if a future mode call is added.
            return Err(Error::ModeUnavailable);
        }

        Ok(Self {
            event_thread: None,
            stop: Arc::new(AtomicBool::new(false)),
            raw,
            slots,
            started: AtomicBool::new(false),
            depth_running: false,
            video_running: false,
            ctx,
        })
    }

    /// Start the depth stream only and spawn the libfreenect event loop.
    /// Equivalent to `start_streams(false, true)`.
    pub fn start(&mut self) -> Result<(), Error> {
        self.start_streams(false, true)
    }

    /// Start the requested streams. Either or both can be enabled. Once a
    /// stream is started, only `stop()` (or Drop) turns it off — repeated
    /// calls are no-ops.
    pub fn start_streams(&mut self, rgb: bool, depth: bool) -> Result<(), Error> {
        if !rgb && !depth {
            return Ok(());
        }
        let g = self.ctx.api_lock.lock();
        if depth && !self.depth_running {
            // SAFETY: device is open, callback is installed.
            let rc = unsafe { sys::freenect_start_depth(self.raw) };
            if rc < 0 {
                drop(g);
                return Err(Error::StartFailed(rc));
            }
            self.depth_running = true;
        }
        if rgb && !self.video_running {
            // SAFETY: device is open, video callback is installed.
            let rc = unsafe { sys::freenect_start_video(self.raw) };
            if rc < 0 {
                if depth && self.depth_running {
                    // SAFETY: we just started the depth stream above.
                    let _ = unsafe { sys::freenect_stop_depth(self.raw) };
                    self.depth_running = false;
                }
                drop(g);
                return Err(Error::StartFailed(rc));
            }
            self.video_running = true;
        }
        drop(g);

        // Spawn the event loop on the first start; subsequent calls reuse it.
        if !self.started.swap(true, Ordering::AcqRel) {
            let stop = self.stop.clone();
            let ctx = self.ctx.clone();
            let handle = thread::Builder::new()
                .name("freenect-events".to_string())
                .spawn(move || event_loop(ctx, stop))
                .map_err(Error::Spawn)?;
            self.event_thread = Some(handle);
        }
        Ok(())
    }

    /// Stop all streams and join the event loop thread.
    pub fn stop(&mut self) -> Result<(), Error> {
        if !self.started.swap(false, Ordering::AcqRel) {
            return Ok(());
        }
        self.stop.store(true, Ordering::Release);
        if let Some(handle) = self.event_thread.take()
            && handle.join().is_err()
        {
            error!("freenect event thread panicked");
        }
        let g = self.ctx.api_lock.lock();
        let mut last_err: Option<i32> = None;
        if self.depth_running {
            // SAFETY: depth stream was started.
            let rc = unsafe { sys::freenect_stop_depth(self.raw) };
            if rc < 0 {
                last_err = Some(rc);
            }
            self.depth_running = false;
        }
        if self.video_running {
            // SAFETY: video stream was started.
            let rc = unsafe { sys::freenect_stop_video(self.raw) };
            if rc < 0 {
                last_err = Some(rc);
            }
            self.video_running = false;
        }
        drop(g);
        if let Some(rc) = last_err {
            return Err(Error::StopFailed(rc));
        }
        Ok(())
    }

    /// Read the latest depth frame, if any. Returns `None` when no new
    /// frame has arrived since the last call.
    pub fn poll_depth(&self) -> Option<DepthFrame> {
        self.slots.depth.poll()
    }

    /// Read the latest color frame (640×480 RGB888), if any.
    pub fn poll_rgb(&self) -> Option<RgbFrame> {
        self.slots.video.poll()
    }

    /// Drive the motorised base to `angle_deg` (clamped to the
    /// `[TILT_MIN_DEG, TILT_MAX_DEG]` range). The call returns once the
    /// command has been queued; mechanical motion takes a beat.
    pub fn set_tilt_degrees(&self, angle_deg: f32) -> Result<(), Error> {
        let clamped = angle_deg.clamp(TILT_MIN_DEG, TILT_MAX_DEG);
        let _g = self.ctx.api_lock.lock();
        // SAFETY: device is open and the motor subdevice was claimed at
        // open time.
        let rc = unsafe { sys::freenect_set_tilt_degs(self.raw, f64::from(clamped)) };
        if rc < 0 {
            return Err(Error::TiltFailed(rc));
        }
        Ok(())
    }

    /// Set the LED on the front of the base.
    pub fn set_led(&self, state: LedState) -> Result<(), Error> {
        let _g = self.ctx.api_lock.lock();
        // SAFETY: device is open and the motor subdevice was claimed.
        // `as _` lets Rust pick the bindgen-generated enum type at the
        // call site: clang renders C anonymous enums as `u32` on
        // Linux/macOS, `i32` on Windows MSVC. `LedState::to_raw()`
        // stays `u32`-typed for our Rust users.
        let rc = unsafe { sys::freenect_set_led(self.raw, state.to_raw() as _) };
        if rc < 0 {
            return Err(Error::LedFailed(rc));
        }
        Ok(())
    }

    /// Force a USB roundtrip to refresh the tilt + accelerometer state and
    /// return a snapshot. Cheap (~1 ms) but not free; throttle to a few
    /// times per second if you're polling continuously.
    pub fn tilt_state(&self) -> Result<TiltState, Error> {
        let _g = self.ctx.api_lock.lock();
        // SAFETY: device is open with motor claimed.
        let rc = unsafe { sys::freenect_update_tilt_state(self.raw) };
        if rc < 0 {
            return Err(Error::TiltStateFailed(rc));
        }
        // SAFETY: device pointer is valid; libfreenect returns a pointer to
        // its internal raw state struct, valid until the next update_tilt
        // call on this device.
        let raw_state = unsafe { sys::freenect_get_tilt_state(self.raw) };
        if raw_state.is_null() {
            return Err(Error::TiltStateFailed(-1));
        }
        // SAFETY: pointer non-null per check above.
        let angle = unsafe { sys::freenect_get_tilt_degs(raw_state) } as f32;
        // SAFETY: same.
        let status_raw = unsafe { sys::freenect_get_tilt_status(raw_state) };
        let mut ax = 0.0_f64;
        let mut ay = 0.0_f64;
        let mut az = 0.0_f64;
        // SAFETY: out-pointers are valid stack locations.
        unsafe { sys::freenect_get_mks_accel(raw_state, &mut ax, &mut ay, &mut az) };
        Ok(TiltState {
            angle_deg: angle,
            status: TiltStatus::from_raw(status_raw as i32),
            accel_mks: [ax as f32, ay as f32, az as f32],
        })
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        if let Err(e) = self.stop() {
            warn!(?e, "freenect: stop failed during Drop");
        }
        let _g = self.ctx.api_lock.lock();
        if !self.raw.is_null() {
            // SAFETY: we own raw, the event thread has been joined, the
            // callbacks can no longer fire.
            unsafe { sys::freenect_close_device(self.raw) };
        }
    }
}

/// Aggregates the per-stream slots into a single struct so libfreenect's
/// single user-data pointer can reach both callbacks.
#[derive(Default)]
struct Slots {
    depth: DepthSlot,
    video: VideoSlot,
}

#[derive(Default)]
struct DepthSlot {
    inner: Mutex<DepthInner>,
    has_new: AtomicBool,
}

#[derive(Default)]
struct DepthInner {
    timestamp: u32,
    data: Vec<u16>,
}

impl DepthSlot {
    fn write(&self, data: &[u16], timestamp: u32) {
        let mut g = self.inner.lock();
        g.timestamp = timestamp;
        if g.data.len() != data.len() {
            g.data.resize(data.len(), 0);
        }
        g.data.copy_from_slice(data);
        drop(g);
        self.has_new.store(true, Ordering::Release);
    }

    fn poll(&self) -> Option<DepthFrame> {
        if !self.has_new.load(Ordering::Acquire) {
            return None;
        }
        let g = self.inner.lock();
        if !self.has_new.load(Ordering::Relaxed) {
            return None;
        }
        let frame = DepthFrame {
            width: DEPTH_WIDTH,
            height: DEPTH_HEIGHT,
            timestamp_raw: g.timestamp,
            data: g.data.clone(),
        };
        drop(g);
        self.has_new.store(false, Ordering::Release);
        Some(frame)
    }
}

#[derive(Default)]
struct VideoSlot {
    inner: Mutex<VideoInner>,
    has_new: AtomicBool,
}

#[derive(Default)]
struct VideoInner {
    timestamp: u32,
    data: Vec<u8>,
}

impl VideoSlot {
    fn write(&self, data: &[u8], timestamp: u32) {
        let mut g = self.inner.lock();
        g.timestamp = timestamp;
        if g.data.len() != data.len() {
            g.data.resize(data.len(), 0);
        }
        g.data.copy_from_slice(data);
        drop(g);
        self.has_new.store(true, Ordering::Release);
    }

    fn poll(&self) -> Option<RgbFrame> {
        if !self.has_new.load(Ordering::Acquire) {
            return None;
        }
        let g = self.inner.lock();
        if !self.has_new.load(Ordering::Relaxed) {
            return None;
        }
        let frame = RgbFrame {
            width: VIDEO_WIDTH,
            height: VIDEO_HEIGHT,
            timestamp_raw: g.timestamp,
            data: g.data.clone(),
        };
        drop(g);
        self.has_new.store(false, Ordering::Release);
        Some(frame)
    }
}

extern "C" fn depth_callback(dev: *mut sys::freenect_device, depth: *mut c_void, timestamp: u32) {
    if dev.is_null() || depth.is_null() {
        return;
    }
    // SAFETY: set_user was called with a stable Box<Slots> address;
    // libfreenect just hands the pointer back unchanged.
    let user = unsafe { sys::freenect_get_user(dev) };
    if user.is_null() {
        return;
    }
    // SAFETY: the only writer of `set_user` was Device::wrap, with a
    // *const Slots. The Box outlives any callback because we join the
    // event thread before dropping the box.
    let slots = unsafe { &*(user as *const Slots) };
    // SAFETY: depth points to width*height u16 in mm (FREENECT_DEPTH_MM mode).
    let pixels = (DEPTH_WIDTH * DEPTH_HEIGHT) as usize;
    let data = unsafe { std::slice::from_raw_parts(depth.cast::<u16>(), pixels) };
    slots.depth.write(data, timestamp);
}

extern "C" fn video_callback(dev: *mut sys::freenect_device, video: *mut c_void, timestamp: u32) {
    if dev.is_null() || video.is_null() {
        return;
    }
    // SAFETY: same pointer round-trip as `depth_callback`.
    let user = unsafe { sys::freenect_get_user(dev) };
    if user.is_null() {
        return;
    }
    // SAFETY: see `depth_callback`.
    let slots = unsafe { &*(user as *const Slots) };
    // SAFETY: video points to width*height*3 bytes (FREENECT_VIDEO_RGB).
    let bytes = (VIDEO_WIDTH * VIDEO_HEIGHT * 3) as usize;
    let data = unsafe { std::slice::from_raw_parts(video.cast::<u8>(), bytes) };
    slots.video.write(data, timestamp);
}

fn event_loop(ctx: Arc<CtxHandle>, stop: Arc<AtomicBool>) {
    while !stop.load(Ordering::Relaxed) {
        let g = ctx.api_lock.lock();
        // 100 ms timeout so we re-check `stop` ten times a second.
        let mut tv = Timeval {
            tv_sec: 0,
            tv_usec: 100_000,
        };
        // SAFETY: ctx is alive, tv points to a valid timeval. The callback
        // dispatch happens synchronously inside this call.
        let rc = unsafe {
            sys::freenect_process_events_timeout(ctx.raw, (&raw mut tv).cast::<sys::timeval>())
        };
        drop(g);
        if rc < 0 {
            warn!(rc, "freenect_process_events_timeout returned an error");
            // Don't busy loop — back off briefly.
            thread::sleep(Duration::from_millis(50));
        }
    }
}

/// Mirror of `struct timeval` so we don't pull in libc just for one type.
#[repr(C)]
struct Timeval {
    tv_sec: i64,
    tv_usec: i64,
}

/// Wraps a libfreenect / libusb open-failure rc with a richer Display
/// that appends an actionable hint when the code is recognised.
/// libfreenect propagates raw libusb codes through `freenect_open_device`,
/// so e.g. `-12 = LIBUSB_ERROR_NOT_SUPPORTED` is exactly what Windows
/// libusb returns when another driver has claimed the device.
#[derive(Debug, Clone, Copy)]
pub struct OpenFailureCode(pub i32);

impl std::fmt::Display for OpenFailureCode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let hint = match self.0 {
            -3 => " — LIBUSB_ERROR_ACCESS: install the udev rules (see README)",
            -4 => " — LIBUSB_ERROR_NO_DEVICE: device disconnected mid-open",
            -5 => " — LIBUSB_ERROR_NOT_FOUND: kernel driver missing or device gone",
            -6 => " — LIBUSB_ERROR_BUSY: device already opened by another process",
            -12 => {
                " — LIBUSB_ERROR_NOT_SUPPORTED: on Windows, install UsbDk \
                   (filter driver) or replace the Kinect driver with libusbK \
                   via Zadig — see README"
            }
            _ => "",
        };
        write!(f, "rc={}{}", self.0, hint)
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("freenect_init failed (rc={0})")]
    ContextInit(i32),
    #[error("no Kinect v1 device available")]
    NoDevice,
    #[error("freenect_open_device failed: {0}")]
    OpenFailed(OpenFailureCode),
    #[error("requested stream mode is not advertised by the device")]
    ModeUnavailable,
    #[error("freenect_start_depth or _video failed (rc={0})")]
    StartFailed(i32),
    #[error("freenect_stop_depth or _video failed (rc={0})")]
    StopFailed(i32),
    #[error("failed to spawn event thread: {0}")]
    Spawn(std::io::Error),
    #[error("freenect_set_tilt_degs failed (rc={0})")]
    TiltFailed(i32),
    #[error("freenect_set_led failed (rc={0})")]
    LedFailed(i32),
    #[error("freenect_update_tilt_state / get failed (rc={0})")]
    TiltStateFailed(i32),
}
