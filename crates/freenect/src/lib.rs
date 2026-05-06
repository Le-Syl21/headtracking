//! Safe wrapper around `freenect-sys` (libfreenect / Kinect v1).
//!
//! Unlike libfreenect2 (which spawns its own packet pipeline thread),
//! libfreenect is single-threaded: the caller has to drive the event loop
//! by calling [`freenect_process_events`][freenect-sys] continuously. This
//! crate spawns that loop in a dedicated thread when the device starts,
//! and tears it down on `stop()` / `Drop`.
//!
//! ```text
//!   user thread       libfreenect event thread     callback (same thread)
//!   ┌──────────┐     ┌────────────────────────┐   ┌────────────────────┐
//!   │ open     │     │ process_events_timeout │ → │ depth_cb writes to │
//!   │ start    │     │   loop until stop      │   │ FrameSlot (mutex)  │
//!   │ poll(...)│ ←── reads FrameSlot ──────────────────────────────────┘
//!   │ stop     │
//!   └──────────┘
//! ```
//!
//! Depth mode: `FREENECT_DEPTH_MM` at 640×480 / 30 Hz, i.e. `u16` millimeter
//! depth values (0 = no data).

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

/// Pinhole intrinsics for the Kinect v1 IR / depth camera. libfreenect does
/// not expose factory parameters, so the values are nominal Microsoft specs
/// (good to ~1% on a calibrated device).
pub const FX: f32 = 580.0;
pub const FY: f32 = 580.0;
pub const CX: f32 = (DEPTH_WIDTH as f32) / 2.0;
pub const CY: f32 = (DEPTH_HEIGHT as f32) / 2.0;

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
        // We only care about the camera subdevice (not the motor/audio).
        // SAFETY: `raw` is non-null per the check above.
        unsafe {
            sys::freenect_select_subdevices(raw, sys::freenect_device_flags_FREENECT_DEVICE_CAMERA);
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
            return Err(Error::OpenFailed(rc));
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

/// One Kinect v1 device. Streams depth in millimeters via a background
/// thread that pumps libfreenect's event loop.
pub struct Device {
    // Drop order: stop the event thread first (clears callbacks), close the
    // device pointer, then release the FrameSlot. Rust drops fields in
    // declaration order — so list event_thread first.
    event_thread: Option<JoinHandle<()>>,
    stop: Arc<AtomicBool>,
    raw: *mut sys::freenect_device,
    frame_slot: Box<FrameSlot>,
    started: AtomicBool,
    ctx: Arc<CtxHandle>,
}

// SAFETY: same rationale as CtxHandle — the safe wrapper serializes access
// through `ctx.api_lock`, and the raw pointer is opaque.
unsafe impl Send for Device {}

impl Device {
    fn wrap(ctx: Arc<CtxHandle>, raw: *mut sys::freenect_device) -> Result<Self, Error> {
        // FrameSlot must have a stable heap address: we hand its pointer to
        // libfreenect via freenect_set_user, and the depth callback retrieves
        // it on every frame.
        let frame_slot = Box::new(FrameSlot::default());
        let user_ptr = (&raw const *frame_slot).cast::<c_void>().cast_mut();

        // Configure depth mode (640x480 mm) and install the callback.
        // SAFETY: `raw` is a freshly opened device; the API lock keeps the
        // ctx single-threaded, the frame mode comes from libfreenect's own
        // descriptor.
        let api = ctx.api_lock.lock();
        let rc = unsafe {
            let mode = sys::freenect_find_depth_mode(
                sys::freenect_resolution_FREENECT_RESOLUTION_MEDIUM,
                sys::freenect_depth_format_FREENECT_DEPTH_MM,
            );
            if mode.is_valid == 0 {
                drop(api);
                sys::freenect_close_device(raw);
                return Err(Error::DepthModeUnavailable);
            }
            sys::freenect_set_depth_mode(raw, mode)
        };
        if rc < 0 {
            drop(api);
            // SAFETY: nothing has been started yet.
            unsafe { sys::freenect_close_device(raw) };
            return Err(Error::DepthModeUnavailable);
        }
        // SAFETY: same API-lock + freshly opened device.
        unsafe {
            sys::freenect_set_user(raw, user_ptr);
            sys::freenect_set_depth_callback(raw, Some(depth_callback));
        }
        drop(api);

        Ok(Self {
            event_thread: None,
            stop: Arc::new(AtomicBool::new(false)),
            raw,
            frame_slot,
            started: AtomicBool::new(false),
            ctx,
        })
    }

    /// Start the depth stream and spawn the libfreenect event loop thread.
    pub fn start(&mut self) -> Result<(), Error> {
        if self.started.swap(true, Ordering::AcqRel) {
            return Ok(());
        }
        let g = self.ctx.api_lock.lock();
        // SAFETY: device is open, callback is installed.
        let rc = unsafe { sys::freenect_start_depth(self.raw) };
        drop(g);
        if rc < 0 {
            self.started.store(false, Ordering::Release);
            return Err(Error::StartFailed(rc));
        }

        let stop = self.stop.clone();
        let ctx = self.ctx.clone();
        let handle = thread::Builder::new()
            .name("freenect-events".to_string())
            .spawn(move || event_loop(ctx, stop))
            .map_err(Error::Spawn)?;
        self.event_thread = Some(handle);
        Ok(())
    }

    /// Stop the depth stream and join the event loop thread.
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
        // SAFETY: stream was started; the event thread is no longer pumping.
        let rc = unsafe { sys::freenect_stop_depth(self.raw) };
        drop(g);
        if rc < 0 {
            return Err(Error::StopFailed(rc));
        }
        Ok(())
    }

    /// Read the latest depth frame, if any. Returns `None` when no new
    /// frame has arrived since the last call.
    pub fn poll_depth(&self) -> Option<DepthFrame> {
        self.frame_slot.poll()
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
            // callback can no longer fire.
            unsafe { sys::freenect_close_device(self.raw) };
        }
    }
}

/// Thread-safe slot the C callback writes into and Rust polls from.
#[derive(Default)]
struct FrameSlot {
    inner: Mutex<Inner>,
    has_new: AtomicBool,
}

#[derive(Default)]
struct Inner {
    timestamp: u32,
    data: Vec<u16>,
}

impl FrameSlot {
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

extern "C" fn depth_callback(dev: *mut sys::freenect_device, depth: *mut c_void, timestamp: u32) {
    if dev.is_null() || depth.is_null() {
        return;
    }
    // SAFETY: set_user was called with a stable Box<FrameSlot> address;
    // libfreenect just hands the pointer back unchanged.
    let user = unsafe { sys::freenect_get_user(dev) };
    if user.is_null() {
        return;
    }
    // SAFETY: the only writer of `set_user` was Device::wrap, with a
    // *const FrameSlot. The Box outlives any callback because we join the
    // event thread before dropping the box.
    let slot = unsafe { &*(user as *const FrameSlot) };
    // SAFETY: depth points to width*height u16 in mm (FREENECT_DEPTH_MM mode).
    let pixels = (DEPTH_WIDTH * DEPTH_HEIGHT) as usize;
    let data = unsafe { std::slice::from_raw_parts(depth.cast::<u16>(), pixels) };
    slot.write(data, timestamp);
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

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("freenect_init failed (rc={0})")]
    ContextInit(i32),
    #[error("no Kinect v1 device available")]
    NoDevice,
    #[error("freenect_open_device failed (rc={0})")]
    OpenFailed(i32),
    #[error("FREENECT_DEPTH_MM mode is not advertised by the device")]
    DepthModeUnavailable,
    #[error("freenect_start_depth failed (rc={0})")]
    StartFailed(i32),
    #[error("freenect_stop_depth failed (rc={0})")]
    StopFailed(i32),
    #[error("failed to spawn event thread: {0}")]
    Spawn(std::io::Error),
}
