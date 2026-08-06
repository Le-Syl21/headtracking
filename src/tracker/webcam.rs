//! Webcam head tracker backend (SDL3 capture via dlsym + BlazePose).
//!
//! Unlike `tools/headtracking-demo` which links SDL3 statically through the
//! `webcam` workspace crate, the plugin resolves SDL3 symbols at runtime
//! against `libSDL3.so` already loaded by VPX. Two SDL3 state machines in
//! the same process would race on the V4L2 device claim, the event bus,
//! and the camera subsystem refcount — see `project_sdl3_strategy.md`.
//!
//! The dlopen of `libSDL3.so.0` returns the existing handle when VPX has
//! already loaded the library (which it always has by `OnGameStart`),
//! and only loads a fresh copy in the unlikely case the plugin is the
//! first SDL3 user. SDL3 internally refcounts subsystem inits, so calling
//! `SDL_Init(SDL_INIT_CAMERA)` ourselves is safe regardless.
//!
//! Algorithm (demo-validated): BlazePose on each frame; distance from the
//! shoulder width (a stable ~0.40 m span), glabella deprojected through the
//! configured focal (`WebcamFocalPx` setting) or a nominal one derived from
//! the frame width.

use std::ffi::{CStr, c_char, c_int, c_void};
use std::ptr::NonNull;
use std::sync::OnceLock;
use std::time::Instant;

use libloading::{Library, Symbol};
use tracing::{info, warn};

use super::{HeadTracker, Pose};

// ============================================================ SDL3 ABI

// Numeric values cribbed from SDL_init.h / SDL_pixels.h. Stable across
// SDL3 patch versions because they're public API.
const SDL_INIT_CAMERA: u32 = 0x0001_0000;
const SDL_PIXELFORMAT_RGB24: c_int = 0x1710_1803;

#[allow(non_camel_case_types)]
type SDL_PixelFormat = c_int;
#[allow(non_camel_case_types)]
type SDL_Colorspace = u32;
#[allow(non_camel_case_types)]
type SDL_SurfaceFlags = u32;
#[allow(non_camel_case_types)]
type SDL_CameraID = u32;

#[repr(C)]
#[derive(Default)]
#[allow(non_snake_case)]
struct SDL_CameraSpec {
    format: SDL_PixelFormat,
    colorspace: SDL_Colorspace,
    width: c_int,
    height: c_int,
    framerate_numerator: c_int,
    framerate_denominator: c_int,
}

// SDL_Surface — only the fields we read are typed; the rest is opaque
// padding. SDL3 keeps the leading layout stable, so reading flags / format
// / w / h / pitch / pixels is safe.
#[repr(C)]
#[allow(non_snake_case)]
struct SDL_Surface {
    flags: SDL_SurfaceFlags,
    format: SDL_PixelFormat,
    w: c_int,
    h: c_int,
    pitch: c_int,
    pixels: *mut c_void,
    // Trailing private fields — refcount, reserved, props, internal —
    // are accessed only by SDL itself. We never touch them.
}

/// Function pointers resolved against `libSDL3.so`.
struct Sdl3Api {
    // Lifecycle
    sdl_get_error: unsafe extern "C" fn() -> *const c_char,
    sdl_pump_events: unsafe extern "C" fn(),
    sdl_free: unsafe extern "C" fn(*mut c_void),

    // Camera enumeration / open / poll
    sdl_get_cameras: unsafe extern "C" fn(count: *mut c_int) -> *mut SDL_CameraID,
    sdl_get_camera_name: unsafe extern "C" fn(SDL_CameraID) -> *const c_char,
    sdl_open_camera: unsafe extern "C" fn(SDL_CameraID, *const SDL_CameraSpec) -> *mut c_void,
    sdl_close_camera: unsafe extern "C" fn(*mut c_void),
    sdl_get_camera_format: unsafe extern "C" fn(*mut c_void, *mut SDL_CameraSpec) -> bool,
    sdl_acquire_camera_frame: unsafe extern "C" fn(*mut c_void, *mut u64) -> *mut SDL_Surface,
    sdl_release_camera_frame: unsafe extern "C" fn(*mut c_void, *mut SDL_Surface),

    // Surface conversion (handles non-RGB24 native camera formats)
    sdl_convert_surface:
        unsafe extern "C" fn(*mut SDL_Surface, SDL_PixelFormat) -> *mut SDL_Surface,
    sdl_destroy_surface: unsafe extern "C" fn(*mut SDL_Surface),

    /// Held to keep the dlopen handle alive — symbols above point into
    /// this library's text segment.
    _lib: Library,
}

unsafe fn load_sym<'a, T>(lib: &'a Library, name: &[u8]) -> Result<Symbol<'a, T>, Error> {
    // SAFETY: caller picks a valid C-string symbol name; libloading
    // performs the actual dlsym and reports an error if the symbol is
    // missing.
    unsafe { lib.get::<T>(name) }
        .map_err(|e| Error::Symbol(format!("{}: {e}", String::from_utf8_lossy(name))))
}

impl Sdl3Api {
    fn load() -> Result<Self, Error> {
        // Try the versioned soname first (what every modern distro and the
        // VPX bundle ships), then the unversioned alias as a fallback. Both
        // paths use the standard linker search order: if VPX already
        // dlopen'd SDL3, the dynamic loader returns the existing handle and
        // bumps its refcount.
        // SAFETY: Library::new dlopens the library; the returned Library
        // is dropped with dlclose, which decrements VPX's refcount but
        // doesn't unload the library while VPX still holds it.
        let lib = unsafe { Library::new("libSDL3.so.0") }
            .or_else(|_| unsafe { Library::new("libSDL3.so") })
            .map_err(|e| Error::Open(format!("dlopen libSDL3.so[.0]: {e}")))?;

        // Dlsym every function we need up front so any version mismatch
        // surfaces at PluginLoad rather than mid-frame.
        // SAFETY: each symbol has the documented SDL3 ABI signature; the
        // Library handle outlives the function pointers (kept in `_lib`).
        unsafe {
            let sdl_init: Symbol<unsafe extern "C" fn(u32) -> bool> = load_sym(&lib, b"SDL_Init")?;
            let sdl_get_error: Symbol<unsafe extern "C" fn() -> *const c_char> =
                load_sym(&lib, b"SDL_GetError")?;
            let sdl_pump_events: Symbol<unsafe extern "C" fn()> =
                load_sym(&lib, b"SDL_PumpEvents")?;
            let sdl_free: Symbol<unsafe extern "C" fn(*mut c_void)> = load_sym(&lib, b"SDL_free")?;
            let sdl_get_cameras: Symbol<unsafe extern "C" fn(*mut c_int) -> *mut SDL_CameraID> =
                load_sym(&lib, b"SDL_GetCameras")?;
            let sdl_get_camera_name: Symbol<unsafe extern "C" fn(SDL_CameraID) -> *const c_char> =
                load_sym(&lib, b"SDL_GetCameraName")?;
            let sdl_open_camera: Symbol<
                unsafe extern "C" fn(SDL_CameraID, *const SDL_CameraSpec) -> *mut c_void,
            > = load_sym(&lib, b"SDL_OpenCamera")?;
            let sdl_close_camera: Symbol<unsafe extern "C" fn(*mut c_void)> =
                load_sym(&lib, b"SDL_CloseCamera")?;
            let sdl_get_camera_format: Symbol<
                unsafe extern "C" fn(*mut c_void, *mut SDL_CameraSpec) -> bool,
            > = load_sym(&lib, b"SDL_GetCameraFormat")?;
            let sdl_acquire_camera_frame: Symbol<
                unsafe extern "C" fn(*mut c_void, *mut u64) -> *mut SDL_Surface,
            > = load_sym(&lib, b"SDL_AcquireCameraFrame")?;
            let sdl_release_camera_frame: Symbol<
                unsafe extern "C" fn(*mut c_void, *mut SDL_Surface),
            > = load_sym(&lib, b"SDL_ReleaseCameraFrame")?;
            let sdl_convert_surface: Symbol<
                unsafe extern "C" fn(*mut SDL_Surface, SDL_PixelFormat) -> *mut SDL_Surface,
            > = load_sym(&lib, b"SDL_ConvertSurface")?;
            let sdl_destroy_surface: Symbol<unsafe extern "C" fn(*mut SDL_Surface)> =
                load_sym(&lib, b"SDL_DestroySurface")?;

            // Ref-counted SDL_Init: if VPX already initialised the camera
            // subsystem (or any other), this is a no-op increment. If we're
            // first, we bring it up.
            let ok = sdl_init(SDL_INIT_CAMERA);
            if !ok {
                let msg = read_err(*sdl_get_error);
                return Err(Error::Init(msg));
            }

            Ok(Self {
                sdl_get_error: *sdl_get_error,
                sdl_pump_events: *sdl_pump_events,
                sdl_free: *sdl_free,
                sdl_get_cameras: *sdl_get_cameras,
                sdl_get_camera_name: *sdl_get_camera_name,
                sdl_open_camera: *sdl_open_camera,
                sdl_close_camera: *sdl_close_camera,
                sdl_get_camera_format: *sdl_get_camera_format,
                sdl_acquire_camera_frame: *sdl_acquire_camera_frame,
                sdl_release_camera_frame: *sdl_release_camera_frame,
                sdl_convert_surface: *sdl_convert_surface,
                sdl_destroy_surface: *sdl_destroy_surface,
                _lib: lib,
            })
        }
    }

    fn err_str(&self) -> String {
        // SAFETY: SDL_GetError returns a thread-local C string with stable
        // lifetime until the next SDL call on this thread.
        unsafe { read_err(self.sdl_get_error) }
    }
}

unsafe fn read_err(get: unsafe extern "C" fn() -> *const c_char) -> String {
    // SAFETY: caller passes a valid SDL_GetError pointer; SDL3 returns a
    // NUL-terminated thread-local string (or NULL when there's no error).
    unsafe {
        let p = get();
        if p.is_null() {
            String::new()
        } else {
            CStr::from_ptr(p).to_string_lossy().into_owned()
        }
    }
}

static SDL3: OnceLock<Result<Sdl3Api, Error>> = OnceLock::new();

fn sdl3() -> Result<&'static Sdl3Api, Error> {
    SDL3.get_or_init(Sdl3Api::load)
        .as_ref()
        .map_err(Clone::clone)
}

/// Enumerate the cameras SDL currently sees — product names in SDL order,
/// which is exactly the `DeviceIndex` space used by [`OpenedCamera::open`].
/// Called at plugin load to label the setting's dropdown with real names;
/// returns an empty list when SDL (or any camera) is unavailable.
pub fn list_cameras() -> Vec<String> {
    let Ok(api) = sdl3() else {
        return Vec::new();
    };
    let mut count: c_int = 0;
    // SAFETY: same contract as `OpenedCamera::open` — pump then snapshot
    // the camera list; the returned array is ours to free with SDL_free.
    let raw = unsafe {
        (api.sdl_pump_events)();
        (api.sdl_get_cameras)(&mut count)
    };
    if raw.is_null() || count <= 0 {
        if !raw.is_null() {
            // SAFETY: allocated by SDL_GetCameras.
            unsafe { (api.sdl_free)(raw.cast::<c_void>()) };
        }
        return Vec::new();
    }
    let names = (0..count as isize)
        .map(|i| {
            // SAFETY: 0 <= i < count; name pointer per SDL contract.
            let id: SDL_CameraID = unsafe { *raw.offset(i) };
            let ptr = unsafe { (api.sdl_get_camera_name)(id) };
            if ptr.is_null() {
                format!("Camera #{id}")
            } else {
                // SAFETY: non-null NUL-terminated string per SDL contract.
                unsafe { CStr::from_ptr(ptr) }
                    .to_string_lossy()
                    .into_owned()
            }
        })
        .collect();
    // SAFETY: allocated by SDL_GetCameras.
    unsafe { (api.sdl_free)(raw.cast::<c_void>()) };
    names
}

// ============================================================ Camera + backend

// Bootstrap focal-length heuristic when no calibration is available
// yet: a typical webcam has a horizontal FOV ~60°, which puts
// `fx ≈ width / (2 * tan(30°))` ≈ width × 0.866. We round to 0.85
// for a small safety margin against the wide end of cheap cameras.
// The hand-fiducial path (`hand_fiducial::observe`) refines this on
// the fly once both player hands are detected.

struct OpenedCamera {
    handle: NonNull<c_void>,
    width: u32,
    height: u32,
    name: String,
}

// SAFETY: SDL_Camera handles can move between threads as long as we
// don't share them concurrently. The session thread is the only owner.
unsafe impl Send for OpenedCamera {}

impl OpenedCamera {
    fn open(api: &Sdl3Api, index: usize) -> Result<Self, Error> {
        // SDL_GetCameras returns a snapshot of the camera list maintained
        // by SDL3's event loop. Without a pump, hot-plugged devices stay
        // invisible.
        // SAFETY: pump and getter are valid SDL3 entry points; both
        // tolerate being called on any thread once SDL_Init has succeeded.
        let mut count: c_int = 0;
        let raw = unsafe {
            (api.sdl_pump_events)();
            (api.sdl_get_cameras)(&mut count)
        };
        if raw.is_null() {
            return Err(Error::Enumerate(api.err_str()));
        }
        if count <= 0 {
            // SAFETY: SDL3 still hands back an allocated array we own; free it.
            unsafe { (api.sdl_free)(raw.cast::<c_void>()) };
            return Err(Error::NoDevice);
        }

        // List every camera SDL sees — the user picks one with the
        // DeviceIndex setting, so give them the full menu in the logs.
        let device_name = |cam_id: SDL_CameraID| -> String {
            // SAFETY: valid camera id just read from SDL_GetCameras; SDL3
            // returns a NUL-terminated string with stable lifetime until
            // the camera is closed.
            let name_ptr = unsafe { (api.sdl_get_camera_name)(cam_id) };
            if name_ptr.is_null() {
                format!("Camera #{cam_id}")
            } else {
                // SAFETY: non-null NUL-terminated string per above.
                unsafe { CStr::from_ptr(name_ptr) }
                    .to_string_lossy()
                    .into_owned()
            }
        };
        for i in 0..count as isize {
            // SAFETY: 0 <= i < count, raw is non-null.
            let id: SDL_CameraID = unsafe { *raw.offset(i) };
            info!(device_index = i, id, name = %device_name(id), "webcam: available camera");
        }

        // Pick the n-th device, falling back to the first when the index
        // overshoots (so DeviceIndex=0 always works).
        let pick_idx = (index as isize).min(count as isize - 1).max(0);
        // SAFETY: 0 <= pick_idx < count, raw is non-null.
        let cam_id: SDL_CameraID = unsafe { *raw.offset(pick_idx) };
        let name = device_name(cam_id);
        // SAFETY: array allocated by SDL_GetCameras; SDL_free is the
        // matching deallocator.
        unsafe { (api.sdl_free)(raw.cast::<c_void>()) };

        // Pass NULL spec so SDL keeps the camera's native format (UVC
        // devices often reject VIDIOC_S_FMT for RGB24 directly; we
        // convert per-frame instead).
        // SAFETY: SDL_OpenCamera tolerates a NULL spec per its docs.
        let handle_ptr = unsafe { (api.sdl_open_camera)(cam_id, std::ptr::null()) };
        let handle = NonNull::new(handle_ptr).ok_or_else(|| Error::Open(api.err_str()))?;

        // Read back the actual format/resolution SDL settled on.
        let mut got = SDL_CameraSpec::default();
        // SAFETY: handle is non-null; got is a valid out-pointer.
        unsafe { (api.sdl_get_camera_format)(handle.as_ptr(), &mut got) };
        let width = got.width.max(0) as u32;
        let height = got.height.max(0) as u32;

        info!(id = cam_id, name, width, height, "webcam (dlsym): opened");
        Ok(Self {
            handle,
            width,
            height,
            name,
        })
    }

    fn poll_rgb(&self, api: &Sdl3Api) -> Option<RgbFrame> {
        let mut ts_ns: u64 = 0;
        // SAFETY: handle is alive; ts_ns is a valid out-pointer; the
        // returned surface must be released via SDL_ReleaseCameraFrame.
        let surf_ptr = unsafe { (api.sdl_acquire_camera_frame)(self.handle.as_ptr(), &mut ts_ns) };
        if surf_ptr.is_null() {
            return None;
        }
        let frame = unsafe { surface_to_rgb(api, surf_ptr, ts_ns) };
        // SAFETY: surf_ptr was returned by Acquire; we own the matching Release.
        unsafe { (api.sdl_release_camera_frame)(self.handle.as_ptr(), surf_ptr) };
        frame
    }
}

impl Drop for OpenedCamera {
    fn drop(&mut self) {
        if let Ok(api) = sdl3() {
            // SAFETY: handle was returned by SDL_OpenCamera and we own it.
            unsafe { (api.sdl_close_camera)(self.handle.as_ptr()) };
        }
    }
}

struct RgbFrame {
    width: u32,
    height: u32,
    data: Vec<u8>,
}

unsafe fn surface_to_rgb(
    api: &Sdl3Api,
    surf_ptr: *mut SDL_Surface,
    _ts_ns: u64,
) -> Option<RgbFrame> {
    // SAFETY: surf_ptr is non-null per the caller; SDL keeps the surface
    // live until SDL_ReleaseCameraFrame. Reading fields off the leading
    // layout is documented as stable.
    let surf = unsafe { &*surf_ptr };
    if surf.format == SDL_PIXELFORMAT_RGB24 {
        return unsafe { copy_rgb24(surf) };
    }
    // Different format → convert through SDL.
    // SAFETY: surf_ptr is non-null and lives for the duration of this
    // call; SDL_ConvertSurface returns a fresh surface we own.
    let converted = unsafe { (api.sdl_convert_surface)(surf_ptr, SDL_PIXELFORMAT_RGB24) };
    if converted.is_null() {
        warn!(err = %api.err_str(), "webcam (dlsym): SDL_ConvertSurface failed");
        return None;
    }
    // SAFETY: converted is non-null per the check above.
    let frame = unsafe { copy_rgb24(&*converted) };
    // SAFETY: converted was allocated by SDL_ConvertSurface; we own it.
    unsafe { (api.sdl_destroy_surface)(converted) };
    frame
}

unsafe fn copy_rgb24(surf: &SDL_Surface) -> Option<RgbFrame> {
    let width = surf.w.max(0) as u32;
    let height = surf.h.max(0) as u32;
    if width == 0 || height == 0 || surf.pixels.is_null() {
        return None;
    }
    let pitch = surf.pitch.max(0) as usize;
    let row_bytes = (width as usize) * 3;
    let mut data = Vec::with_capacity(row_bytes * (height as usize));
    // SAFETY: pixels is non-null; we read `pitch` bytes per row for `h`
    // rows and copy the leading `row_bytes` of each into our packed
    // RGB buffer.
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
        data,
    })
}

// ============================================================ HeadTracker impl

pub struct WebcamBackend {
    camera: OpenedCamera,
    blaze: blazepose::BlazePose,
    started_at: Instant,
    /// Declared last: released only after the camera handle above closes.
    _hwlock: crate::hwlock::HwLock,
}

impl WebcamBackend {
    pub fn open(index: usize) -> Result<Self, Error> {
        // One "webcam" slug for every index: SDL ids aren't stable across
        // processes, and a cab has one webcam. Same policy as the demo.
        let hwlock = crate::hwlock::HwLock::acquire("webcam").map_err(Error::Busy)?;
        let api = sdl3()?;
        let camera = OpenedCamera::open(api, index)?;
        let blaze = blazepose::BlazePose::new().map_err(|e| Error::Model(e.to_string()))?;
        info!(
            name = camera.name.as_str(),
            width = camera.width,
            height = camera.height,
            "webcam backend ready (BlazePose + shoulder-width ranging)"
        );
        Ok(Self {
            camera,
            blaze,
            started_at: Instant::now(),
            _hwlock: hwlock,
        })
    }

    /// BlazePose on the frame, then the demo-validated webcam ranging:
    /// distance from the shoulder width, glabella deprojected through the
    /// configured (or nominal) focal.
    fn frame_to_pose(&mut self, frame: &RgbFrame) -> Option<Pose> {
        let pose = match self.blaze.poll(&frame.data, frame.width, frame.height) {
            Ok(p) => p?,
            Err(e) => {
                warn!("webcam: blazepose failed: {e}");
                return None;
            }
        };
        let focal = crate::config::current().webcam_focal_px;
        let head = crate::tracker::pipeline::head_pixel_from_pose_webcam(
            &pose,
            frame.width,
            frame.height,
            focal,
        )?;
        Some(Pose {
            position_mm: [head.x_mm, head.y_mm, head.depth_mm],
            timestamp_us: self.started_at.elapsed().as_micros() as u64,
            confidence: pose.presence.clamp(0.0, 1.0),
        })
    }
}

impl HeadTracker for WebcamBackend {
    fn poll(&mut self) -> Option<Pose> {
        let api = sdl3().ok()?;
        let frame = self.camera.poll_rgb(api)?;
        self.frame_to_pose(&frame)
    }

    fn name(&self) -> &'static str {
        "webcam"
    }

    fn device_label(&self) -> String {
        self.camera.name.clone()
    }

    fn poll_calibration_rgb(&mut self) -> Option<(u32, u32, Vec<u8>)> {
        let api = sdl3().ok()?;
        let frame = self.camera.poll_rgb(api)?;
        Some((frame.width, frame.height, frame.data))
    }

    fn shutdown(&mut self) {
        // SDL_CloseCamera runs in OpenedCamera's Drop.
    }
}

#[derive(Debug, Clone, thiserror::Error)]
pub enum Error {
    #[error("dlopen libSDL3 failed: {0}")]
    Open(String),
    #[error("SDL3 symbol not found: {0}")]
    Symbol(String),
    #[error("SDL_Init(SDL_INIT_CAMERA) failed: {0}")]
    Init(String),
    #[error("SDL_GetCameras failed: {0}")]
    Enumerate(String),
    #[error("no webcam available")]
    NoDevice,
    #[error("pose model init: {0}")]
    Model(String),
    #[error("{0}")]
    Busy(String),
}
