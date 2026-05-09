//! Cross-platform webcam capture, mirroring the API shape of the
//! `freenect` / `freenect2` crates so consumers can swap backends with
//! minimal glue.
//!
//! Built on **SDL3**'s `SDL_Camera` API: `sdl3-sys` is pulled in with the
//! `build-from-source-static` + `sdl-camera` features so the final binary
//! ships SDL3 statically — no runtime application dep on the user's box,
//! consistent with the libfreenect[2] / libjpeg-turbo strategy.
//!
//! Threading: SDL_AcquireCameraFrame / SDL_ReleaseCameraFrame are not
//! thread-safe per SDL3 docs, so the [`Camera`] handle is `!Sync`. Polling
//! happens on whichever thread owns the [`Camera`].

use std::ffi::{CStr, c_int, c_void};
use std::ptr::NonNull;
use std::sync::Once;

use parking_lot::Mutex;
use tracing::{info, warn};

use sdl3_sys::camera as sdl_cam;
use sdl3_sys::camera::SDL_CameraID;
use sdl3_sys::error::SDL_GetError;
use sdl3_sys::events::SDL_PumpEvents;
use sdl3_sys::init::{SDL_INIT_CAMERA, SDL_Init, SDL_InitSubSystem, SDL_QuitSubSystem};
use sdl3_sys::pixels::{SDL_Colorspace, SDL_PIXELFORMAT_RGB24};
use sdl3_sys::surface::{SDL_ConvertSurface, SDL_DestroySurface, SDL_Surface};

// ============================================================ Subsystem init

static INIT: Once = Once::new();
static INIT_RESULT: Mutex<Result<(), String>> = Mutex::new(Ok(()));

fn ensure_subsystem() -> Result<(), Error> {
    INIT.call_once(|| {
        // SAFETY: SDL_Init is callable from any thread; SDL3 ref-counts
        // subsystem inits internally.
        let ok = unsafe { SDL_Init(SDL_INIT_CAMERA) };
        if !ok {
            *INIT_RESULT.lock() = Err(read_sdl_error());
        } else {
            info!("SDL3 camera subsystem initialised");
        }
    });
    INIT_RESULT.lock().clone().map_err(Error::Init)
}

fn read_sdl_error() -> String {
    // sdl3-sys exposes SDL_GetError as a safe function (the returned pointer
    // is thread-local static storage with a stable lifetime).
    let ptr = SDL_GetError();
    if ptr.is_null() {
        return String::new();
    }
    // SAFETY: pointer is non-null and points to a NUL-terminated C string.
    unsafe { CStr::from_ptr(ptr) }
        .to_string_lossy()
        .into_owned()
}

/// Force the SDL3 camera subsystem to drop its cached device list and
/// re-enumerate from scratch. Necessary on Windows because the
/// MediaFoundation backend ships *no hot-plug detection at all*
/// (`SDL_camera_mediafoundation.c` has a literal `"no hotplug for you!"`
/// FIXME) — its only enumeration point is `MEDIAFOUNDATION_DetectDevices`,
/// called once from `SDL_CameraInit`. Tearing the subsystem down to ref
/// count 0 and re-initialising is the only way to trigger another scan.
///
/// **The caller MUST drop every live [`Camera`] handle before invoking
/// this.** The subsystem teardown closes every open device; any kept
/// `Camera` would be a use-after-free waiting to happen.
pub fn force_refresh() -> Result<(), Error> {
    ensure_subsystem()?;
    // SAFETY: ref count is ≥ 1 after `ensure_subsystem`. SDL3 docs (init.h)
    // state both calls are thread-safe and ref-counted; quit→init at the
    // same call site cycles the subsystem (re-running DetectDevices) without
    // racing other in-flight inits.
    unsafe {
        SDL_QuitSubSystem(SDL_INIT_CAMERA);
        if !SDL_InitSubSystem(SDL_INIT_CAMERA) {
            return Err(Error::Init(read_sdl_error()));
        }
    }
    info!("SDL3 camera subsystem cycled (forced rescan — Windows hot-plug workaround)");
    Ok(())
}

// ============================================================ Public types

/// Description of a camera advertised by the OS.
#[derive(Debug, Clone)]
pub struct CameraInfo {
    /// Stable identifier handed back to [`Camera::open`].
    pub id: u32,
    /// Human-readable name as reported by the OS.
    pub name: String,
}

/// One color frame copied out of SDL's internal buffer.
/// Layout is row-major RGB888 — `width * height * 3` bytes, channel order
/// `[R, G, B]` per pixel. SDL handles the conversion from whatever native
/// format the camera ships (YUYV, NV12, MJPG, …).
#[derive(Debug, Clone)]
pub struct RgbFrame {
    pub width: u32,
    pub height: u32,
    /// SDL frame timestamp in nanoseconds. Zero if the driver doesn't
    /// expose a stable clock.
    pub timestamp_ns: u64,
    pub data: Vec<u8>,
}

// ============================================================ list()

/// Enumerate all webcams visible to the OS. Cheap (no streams opened).
///
/// **Windows caveat**: SDL3's MediaFoundation backend has *no* hot-plug
/// support (cf. `SDL_camera_mediafoundation.c`: *"no hotplug for you!"*).
/// A camera plugged in after the first call stays invisible until the
/// subsystem is torn down and re-initialised. Use [`force_refresh`] before
/// `list()` when implementing a "rescan" UX.
pub fn list() -> Result<Vec<CameraInfo>, Error> {
    ensure_subsystem()?;
    // SDL_GetCameras returns a snapshot of an internal device list which
    // is only refreshed when SDL processes its camera ADDED / REMOVED
    // events. Without an event pump, hot-plugged webcams stay invisible.
    // SAFETY: SDL_PumpEvents is callable from any thread once SDL_Init has
    // succeeded.
    unsafe {
        SDL_PumpEvents();
    }
    let mut count: c_int = 0;
    // SAFETY: SDL_GetCameras takes a writable c_int pointer and returns a
    // SDL-allocated array of SDL_CameraID. We must free the array via
    // SDL_free (sdl3-sys exposes this as `stdinc::SDL_free`). For
    // simplicity we rely on the array being valid for the duration of the
    // copy below; we don't try to free it (the leak is negligible: a
    // handful of u32s).
    let raw = unsafe { sdl_cam::SDL_GetCameras(&mut count) };
    if raw.is_null() {
        return Err(Error::Enumerate(read_sdl_error()));
    }
    let mut out = Vec::with_capacity(count as usize);
    for i in 0..count {
        // SAFETY: 0 <= i < count, raw is non-null.
        let id: SDL_CameraID = unsafe { *raw.offset(i as isize) };
        // SAFETY: id is a valid SDL_CameraID returned just above.
        let name_ptr = unsafe { sdl_cam::SDL_GetCameraName(id) };
        let name = if name_ptr.is_null() {
            String::new()
        } else {
            // SAFETY: pointer is non-null and SDL strings are NUL-terminated.
            unsafe { CStr::from_ptr(name_ptr) }
                .to_string_lossy()
                .into_owned()
        };
        out.push(CameraInfo {
            id: id.into(),
            name,
        });
    }
    // SAFETY: SDL_GetCameras returns memory that should be freed via
    // SDL_free. We use sdl3_sys::stdinc::SDL_free, available via the
    // same crate.
    unsafe { sdl3_sys::stdinc::SDL_free(raw.cast::<c_void>()) };
    Ok(out)
}

// ============================================================ Camera

/// Single open webcam streaming RGB frames.
pub struct Camera {
    raw: NonNull<sdl_cam::SDL_Camera>,
    width: u32,
    height: u32,
}

// SAFETY: SDL_Camera handles can move between threads; we just don't share
// them concurrently — `!Sync`.
unsafe impl Send for Camera {}

impl Camera {
    /// Open the camera with the given `SDL_CameraID`. We pass a NULL spec so
    /// SDL keeps the camera's native format/resolution — many UVC devices
    /// reject `VIDIOC_S_FMT` for RGB24 directly. The `poll_rgb` path
    /// converts from whatever native format SDL picked (YUYV / NV12 / MJPG).
    pub fn open(id: u32) -> Result<Self, Error> {
        ensure_subsystem()?;
        // SAFETY: id is treated as opaque by SDL; passing a NULL spec is
        // explicitly supported and tells SDL to keep the device defaults.
        let raw = unsafe { sdl_cam::SDL_OpenCamera(SDL_CameraID(id), std::ptr::null()) };
        let raw = NonNull::new(raw).ok_or_else(|| Error::Open(read_sdl_error()))?;

        // Read back the actual format SDL settled on.
        let mut got = sdl_cam::SDL_CameraSpec {
            format: SDL_PIXELFORMAT_RGB24,
            colorspace: SDL_Colorspace(0),
            width: 0,
            height: 0,
            framerate_numerator: 0,
            framerate_denominator: 0,
        };
        // SAFETY: raw is non-null; got is a valid out-pointer.
        let _ = unsafe { sdl_cam::SDL_GetCameraFormat(raw.as_ptr(), &mut got) };
        let width = got.width.max(0) as u32;
        let height = got.height.max(0) as u32;
        info!(id, width, height, "webcam opened");

        Ok(Self { raw, width, height })
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    /// Read the latest RGB frame, if any. Returns `None` when no new frame
    /// has arrived since the last call.
    pub fn poll_rgb(&self) -> Option<RgbFrame> {
        let mut ts_ns: u64 = 0;
        // SAFETY: raw is alive for the duration of self; ts_ns is a valid
        // out-pointer. AcquireCameraFrame returns a Surface we must
        // Release once we're done.
        let surf_ptr = unsafe { sdl_cam::SDL_AcquireCameraFrame(self.raw.as_ptr(), &mut ts_ns) };
        if surf_ptr.is_null() {
            return None;
        }
        let frame = self.surface_to_rgb(surf_ptr, ts_ns);
        // SAFETY: surf_ptr was just returned by Acquire; we own the
        // matching Release call.
        unsafe { sdl_cam::SDL_ReleaseCameraFrame(self.raw.as_ptr(), surf_ptr) };
        frame
    }

    fn surface_to_rgb(&self, surf_ptr: *mut SDL_Surface, ts_ns: u64) -> Option<RgbFrame> {
        // SAFETY: surf_ptr is non-null per the caller; SDL keeps the
        // surface live until SDL_ReleaseCameraFrame.
        let surf = unsafe { &*surf_ptr };
        if surf.format == SDL_PIXELFORMAT_RGB24 {
            return copy_rgb24_surface(surf, ts_ns);
        }
        // Different format → convert through SDL.
        // SAFETY: surf_ptr is non-null and lives for the duration of this
        // call; SDL_ConvertSurface returns a fresh surface we own.
        let converted = unsafe { SDL_ConvertSurface(surf_ptr, SDL_PIXELFORMAT_RGB24) };
        if converted.is_null() {
            warn!(err = %read_sdl_error(), "webcam: SDL_ConvertSurface failed");
            return None;
        }
        // SAFETY: converted is non-null per the check above.
        let converted_ref = unsafe { &*converted };
        let frame = copy_rgb24_surface(converted_ref, ts_ns);
        // SAFETY: converted was allocated by SDL_ConvertSurface; we own it.
        unsafe { SDL_DestroySurface(converted) };
        frame
    }
}

fn copy_rgb24_surface(surf: &SDL_Surface, ts_ns: u64) -> Option<RgbFrame> {
    let width = surf.w.max(0) as u32;
    let height = surf.h.max(0) as u32;
    if width == 0 || height == 0 || surf.pixels.is_null() {
        return None;
    }
    let pitch = surf.pitch.max(0) as usize;
    let row_bytes = (width as usize) * 3;
    let mut data = Vec::with_capacity(row_bytes * (height as usize));
    // SAFETY: pixels is non-null; we read `pitch` bytes per row for `h` rows
    // and copy the leading `row_bytes` of each into our packed RGB buffer.
    unsafe {
        let base = surf.pixels.cast::<u8>();
        for y in 0..(height as usize) {
            let row = base.add(y * pitch);
            data.extend_from_slice(std::slice::from_raw_parts(row, row_bytes));
        }
    }
    Some(RgbFrame {
        width,
        height,
        timestamp_ns: ts_ns,
        data,
    })
}

impl Drop for Camera {
    fn drop(&mut self) {
        // SAFETY: raw is non-null and we own it.
        unsafe { sdl_cam::SDL_CloseCamera(self.raw.as_ptr()) };
    }
}

// ============================================================ Errors

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("SDL_Init(SDL_INIT_CAMERA) failed: {0}")]
    Init(String),
    #[error("camera enumeration failed: {0}")]
    Enumerate(String),
    #[error("failed to open camera: {0}")]
    Open(String),
}

// ============================================================ Re-exports

/// Re-exported so callers can pattern match against SDL's pixel formats if
/// they want to skip the conversion path; not needed for the default
/// poll_rgb flow.
pub use sdl3_sys::pixels::SDL_PixelFormat as SdlPixelFormat;
