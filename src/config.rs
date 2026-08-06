//! Plugin configuration backed by VPX's `[Plugin.HeadTracking]` settings.
//!
//! Each setting is registered with the host via `MsgPluginAPI::RegisterSetting`
//! at `PluginLoad`. VPX then calls our `Set` callback with the value parsed
//! from `VPinballX.ini` (falling back to the declared default), and the
//! in-game plugin settings page edits values LIVE through the same
//! callbacks. The value is mirrored into a global `RwLock<Config>` that the
//! rest of the plugin reads at session-spawn time and per-frame.
//!
//! The surface is deliberately tiny (the plug-and-play doctrine): backend,
//! gain, a three-preset smoothing knob, the tracking stream, per-axis
//! inversion for exotic camera mountings, and a manual webcam focal as a
//! fallback. Everything VPX already knows (table incline, screen geometry,
//! cabinet lockbar width/height) is READ from the host, never asked here.
//!
//! Settings live for the lifetime of the process: the `MsgSettingDef` we
//! pass to `RegisterSetting` is `Box::leak`-ed because the host keeps the
//! pointer for as long as it cares about the setting (e.g. when re-saving).

use std::ffi::{CStr, c_char, c_int};
use std::sync::RwLock;

use crate::plugin::vpx_sys::{
    MSGPI_SETTING_TYPE_BOOL, MSGPI_SETTING_TYPE_FLOAT, MSGPI_SETTING_TYPE_INT, MsgPluginAPI,
    MsgSettingDef, MsgSettingDef__bindgen_ty_1,
    MsgSettingDef__bindgen_ty_1__bindgen_ty_1 as FloatDef,
    MsgSettingDef__bindgen_ty_1__bindgen_ty_2 as IntDef,
    MsgSettingDef__bindgen_ty_1__bindgen_ty_3 as BoolDef,
};

/// Numeric IDs for the `Backend` enum setting. Keep in sync with the
/// `BACKEND_VALUES` label array — VPX uses the array to render the
/// dropdown in the in-game settings UI.
pub const BACKEND_AUTO: i32 = 0;
pub const BACKEND_KINECT_V2: i32 = 1;
pub const BACKEND_KINECT_V1: i32 = 2;
pub const BACKEND_WEBCAM: i32 = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackendKind {
    Auto,
    KinectV2,
    KinectV1,
    Webcam,
}

impl BackendKind {
    fn from_i32(v: i32) -> Self {
        match v {
            BACKEND_KINECT_V2 => Self::KinectV2,
            BACKEND_KINECT_V1 => Self::KinectV1,
            BACKEND_WEBCAM => Self::Webcam,
            _ => Self::Auto,
        }
    }
    fn to_i32(self) -> i32 {
        match self {
            Self::Auto => BACKEND_AUTO,
            Self::KinectV2 => BACKEND_KINECT_V2,
            Self::KinectV1 => BACKEND_KINECT_V1,
            Self::Webcam => BACKEND_WEBCAM,
        }
    }
}

/// The one smoothing knob, as presets rather than raw One-Euro parameters
/// (the BAM lesson: one comprehensible control). Each preset maps to
/// per-axis parameters via [`Config::one_euro_params`]; Z always gets a
/// tighter cutoff because depth readings are inherently noisier than X/Y.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SmoothingPreset {
    /// Kills every tremor; a touch of lag on fast moves. For players who
    /// prefer "un poil moins fluide plutôt que des mouvements parasites".
    Stable,
    /// The field-validated default.
    Normal,
    /// Follows fast; may breathe a little at rest.
    Reactive,
}

impl SmoothingPreset {
    fn from_i32(v: i32) -> Self {
        match v {
            0 => Self::Stable,
            2 => Self::Reactive,
            _ => Self::Normal,
        }
    }
    fn to_i32(self) -> i32 {
        match self {
            Self::Stable => 0,
            Self::Normal => 1,
            Self::Reactive => 2,
        }
    }
}

/// Which sensor stream feeds the pose model on Kinects. `Auto` = IR: the
/// active illumination holds 30 fps in a dark game room where auto-exposed
/// colour drops to 15. Webcams have no IR and ignore this.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamPref {
    Auto,
    Rgb,
}

impl StreamPref {
    fn from_i32(v: i32) -> Self {
        if v == 1 { Self::Rgb } else { Self::Auto }
    }
    fn to_i32(self) -> i32 {
        match self {
            Self::Auto => 0,
            Self::Rgb => 1,
        }
    }
}

/// All plugin settings, mirrored from VPX into the process. Reads happen
/// from the tracker thread (per-loop snapshot — live retuning works) and
/// the OnPrepareFrame callback (gain + trims, cheap RwLock read).
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub backend: BackendKind,
    pub device_index: i32,
    pub gain: f32,
    pub smoothing: SmoothingPreset,
    pub tracking_stream: StreamPref,
    pub invert_x: bool,
    pub invert_y: bool,
    pub invert_z: bool,
    /// Manual webcam focal length in pixels; `0` = automatic (nominal
    /// focal now, playfield-rectangle homography when it lands).
    pub webcam_focal_px: f32,
    pub baseline_offset_x_mm: f32,
    pub baseline_offset_y_mm: f32,
    pub baseline_offset_z_mm: f32,
}

/// One-Euro parameters for one axis.
#[derive(Debug, Clone, Copy)]
pub struct AxisParams {
    pub min_cutoff_hz: f32,
    pub beta: f32,
}

impl Config {
    /// Per-axis One-Euro parameters `[x, y, z]` for the active preset.
    #[must_use]
    pub fn one_euro_params(&self) -> [AxisParams; 3] {
        let (xy, z) = match self.smoothing {
            SmoothingPreset::Stable => (
                AxisParams {
                    min_cutoff_hz: 0.6,
                    beta: 0.005,
                },
                AxisParams {
                    min_cutoff_hz: 0.25,
                    beta: 0.02,
                },
            ),
            SmoothingPreset::Normal => (
                AxisParams {
                    min_cutoff_hz: 1.0,
                    beta: 0.01,
                },
                AxisParams {
                    min_cutoff_hz: 0.4,
                    beta: 0.05,
                },
            ),
            SmoothingPreset::Reactive => (
                AxisParams {
                    min_cutoff_hz: 2.0,
                    beta: 0.05,
                },
                AxisParams {
                    min_cutoff_hz: 0.8,
                    beta: 0.1,
                },
            ),
        };
        [xy, xy, z]
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            backend: BackendKind::Auto,
            device_index: 0,
            gain: 1.0,
            smoothing: SmoothingPreset::Stable,
            tracking_stream: StreamPref::Auto,
            invert_x: false,
            invert_y: false,
            invert_z: false,
            webcam_focal_px: 0.0,
            baseline_offset_x_mm: 0.0,
            baseline_offset_y_mm: 0.0,
            baseline_offset_z_mm: 0.0,
        }
    }
}

static CONFIG: RwLock<Config> = RwLock::new(Config {
    backend: BackendKind::Auto,
    device_index: 0,
    gain: 1.0,
    smoothing: SmoothingPreset::Stable,
    tracking_stream: StreamPref::Auto,
    invert_x: false,
    invert_y: false,
    invert_z: false,
    webcam_focal_px: 0.0,
    baseline_offset_x_mm: 0.0,
    baseline_offset_y_mm: 0.0,
    baseline_offset_z_mm: 0.0,
});

/// Snapshot of the current configuration. Cheap (RwLock read).
pub fn current() -> Config {
    *CONFIG.read().expect("config rwlock poisoned")
}

// ============================================================ Get/Set callbacks
//
// VPX expects bare `extern "C" fn` pointers — no closures, no captured state.
// Each setting has its own pair of trampolines that read/write a single field
// of the global `CONFIG`.

macro_rules! rw {
    () => {
        CONFIG.write().expect("config rwlock poisoned")
    };
}

unsafe extern "C" fn get_backend() -> c_int {
    current().backend.to_i32()
}
unsafe extern "C" fn set_backend(v: c_int) {
    rw!().backend = BackendKind::from_i32(v);
}

unsafe extern "C" fn get_device_index() -> c_int {
    current().device_index
}
unsafe extern "C" fn set_device_index(v: c_int) {
    rw!().device_index = v.max(0);
}

unsafe extern "C" fn get_gain() -> f32 {
    current().gain
}
unsafe extern "C" fn set_gain(v: f32) {
    rw!().gain = v;
}

unsafe extern "C" fn get_smoothing() -> c_int {
    current().smoothing.to_i32()
}
unsafe extern "C" fn set_smoothing(v: c_int) {
    rw!().smoothing = SmoothingPreset::from_i32(v);
}

unsafe extern "C" fn get_stream() -> c_int {
    current().tracking_stream.to_i32()
}
unsafe extern "C" fn set_stream(v: c_int) {
    rw!().tracking_stream = StreamPref::from_i32(v);
}

unsafe extern "C" fn get_invert_x() -> c_int {
    c_int::from(current().invert_x)
}
unsafe extern "C" fn set_invert_x(v: c_int) {
    rw!().invert_x = v != 0;
}

unsafe extern "C" fn get_invert_y() -> c_int {
    c_int::from(current().invert_y)
}
unsafe extern "C" fn set_invert_y(v: c_int) {
    rw!().invert_y = v != 0;
}

unsafe extern "C" fn get_invert_z() -> c_int {
    c_int::from(current().invert_z)
}
unsafe extern "C" fn set_invert_z(v: c_int) {
    rw!().invert_z = v != 0;
}

unsafe extern "C" fn get_webcam_focal() -> f32 {
    current().webcam_focal_px
}
unsafe extern "C" fn set_webcam_focal(v: f32) {
    rw!().webcam_focal_px = v.max(0.0);
}

unsafe extern "C" fn get_offset_x() -> f32 {
    current().baseline_offset_x_mm
}
unsafe extern "C" fn set_offset_x(v: f32) {
    rw!().baseline_offset_x_mm = v;
}

unsafe extern "C" fn get_offset_y() -> f32 {
    current().baseline_offset_y_mm
}
unsafe extern "C" fn set_offset_y(v: f32) {
    rw!().baseline_offset_y_mm = v;
}

unsafe extern "C" fn get_offset_z() -> f32 {
    current().baseline_offset_z_mm
}
unsafe extern "C" fn set_offset_z(v: f32) {
    rw!().baseline_offset_z_mm = v;
}

// ============================================================ register_settings

/// Wrapper that lets us put a `*const c_char` array in a `static` —
/// raw pointers aren't `Sync` by default, but every pointer in the array
/// points at a static C-string literal which is read-only and lives
/// forever, and the host only reads through them.
#[repr(transparent)]
struct EnumValues<const N: usize>([*const c_char; N]);
// SAFETY: see the type doc — all pointers target read-only static memory.
unsafe impl<const N: usize> Sync for EnumValues<N> {}

static BACKEND_VALUES: EnumValues<5> = EnumValues([
    c"Auto".as_ptr(),
    c"Kinect v2".as_ptr(),
    c"Kinect v1".as_ptr(),
    c"Webcam".as_ptr(),
    std::ptr::null(),
]);

static SMOOTHING_VALUES: EnumValues<4> = EnumValues([
    c"Stable".as_ptr(),
    c"Normal".as_ptr(),
    c"Reactive".as_ptr(),
    std::ptr::null(),
]);

static STREAM_VALUES: EnumValues<3> = EnumValues([
    c"Auto (IR on Kinect)".as_ptr(),
    c"Color".as_ptr(),
    std::ptr::null(),
]);

#[allow(clippy::too_many_arguments)]
fn make_int_setting(
    prop_id: &'static CStr,
    name: &'static CStr,
    description: &'static CStr,
    min_val: c_int,
    max_val: c_int,
    def_val: c_int,
    values: *mut *const c_char,
    getter: unsafe extern "C" fn() -> c_int,
    setter: unsafe extern "C" fn(c_int),
) -> *mut MsgSettingDef {
    let def = MsgSettingDef {
        propId: prop_id.as_ptr(),
        name: name.as_ptr(),
        description: description.as_ptr(),
        isUserEditable: 1,
        type_: MSGPI_SETTING_TYPE_INT as c_int,
        __bindgen_anon_1: MsgSettingDef__bindgen_ty_1 {
            intDef: IntDef {
                minVal: min_val,
                maxVal: max_val,
                defVal: def_val,
                values,
                Get: Some(getter),
                Set: Some(setter),
            },
        },
    };
    Box::into_raw(Box::new(def))
}

#[allow(clippy::too_many_arguments)]
fn make_float_setting(
    prop_id: &'static CStr,
    name: &'static CStr,
    description: &'static CStr,
    min_val: f32,
    max_val: f32,
    step: f32,
    def_val: f32,
    getter: unsafe extern "C" fn() -> f32,
    setter: unsafe extern "C" fn(f32),
) -> *mut MsgSettingDef {
    let def = MsgSettingDef {
        propId: prop_id.as_ptr(),
        name: name.as_ptr(),
        description: description.as_ptr(),
        isUserEditable: 1,
        type_: MSGPI_SETTING_TYPE_FLOAT as c_int,
        __bindgen_anon_1: MsgSettingDef__bindgen_ty_1 {
            floatDef: FloatDef {
                minVal: min_val,
                maxVal: max_val,
                step,
                defVal: def_val,
                Get: Some(getter),
                Set: Some(setter),
            },
        },
    };
    Box::into_raw(Box::new(def))
}

fn make_bool_setting(
    prop_id: &'static CStr,
    name: &'static CStr,
    description: &'static CStr,
    def_val: bool,
    getter: unsafe extern "C" fn() -> c_int,
    setter: unsafe extern "C" fn(c_int),
) -> *mut MsgSettingDef {
    let def = MsgSettingDef {
        propId: prop_id.as_ptr(),
        name: name.as_ptr(),
        description: description.as_ptr(),
        isUserEditable: 1,
        type_: MSGPI_SETTING_TYPE_BOOL as c_int,
        __bindgen_anon_1: MsgSettingDef__bindgen_ty_1 {
            boolDef: BoolDef {
                defVal: c_int::from(def_val),
                Get: Some(getter),
                Set: Some(setter),
            },
        },
    };
    Box::into_raw(Box::new(def))
}

/// Register every plugin setting with the host. Called once at PluginLoad.
/// The `MsgSettingDef` instances are leaked on purpose — the host keeps
/// the pointers for the lifetime of the plugin session.
///
/// # Safety
/// `api` must be a valid pointer obtained from VPX's plugin loader, and
/// `endpoint_id` must be the plugin endpoint VPX assigned us at load time.
pub unsafe fn register_settings(api: &MsgPluginAPI, endpoint_id: u32) {
    let Some(register) = api.RegisterSetting else {
        return;
    };

    let defs = [
        make_int_setting(
            c"Backend",
            c"Backend",
            c"Tracker backend (Auto picks the first available: Kinect v2 -> v1 -> Webcam)",
            0,
            3,
            BACKEND_AUTO,
            BACKEND_VALUES.0.as_ptr().cast_mut(),
            get_backend,
            set_backend,
        ),
        make_int_setting(
            c"DeviceIndex",
            c"Device Index",
            c"0-based index when the host has several webcams (Kinects always use the first device)",
            0,
            7,
            0,
            std::ptr::null_mut(),
            get_device_index,
            set_device_index,
        ),
        make_float_setting(
            c"Gain",
            c"Gain",
            c"Multiplier on the head-motion delta before it's applied to the camera",
            0.0,
            5.0,
            0.05,
            1.0,
            get_gain,
            set_gain,
        ),
        make_int_setting(
            c"Smoothing",
            c"Smoothing",
            c"Stable is the field-tested default (kills every tremor for a touch of lag), Normal follows quicker, Reactive follows fast moves closest",
            0,
            2,
            SmoothingPreset::Stable.to_i32(),
            SMOOTHING_VALUES.0.as_ptr().cast_mut(),
            get_smoothing,
            set_smoothing,
        ),
        make_int_setting(
            c"TrackingStream",
            c"Tracking Stream",
            c"Auto tracks on the Kinect's infrared stream (works in a dark room at full rate); Color uses the RGB stream. Webcams always use color.",
            0,
            1,
            StreamPref::Auto.to_i32(),
            STREAM_VALUES.0.as_ptr().cast_mut(),
            get_stream,
            set_stream,
        ),
        make_bool_setting(
            c"InvertX",
            c"Invert X",
            c"Flip the left/right response (for mirrored or unusual camera mountings)",
            false,
            get_invert_x,
            set_invert_x,
        ),
        make_bool_setting(
            c"InvertY",
            c"Invert Y",
            c"Flip the up/down response",
            false,
            get_invert_y,
            set_invert_y,
        ),
        make_bool_setting(
            c"InvertZ",
            c"Invert Z",
            c"Flip the closer/farther response",
            false,
            get_invert_z,
            set_invert_z,
        ),
        make_float_setting(
            c"WebcamFocalPx",
            c"Webcam Focal (px)",
            c"Webcam focal length in pixels; 0 = automatic. Only needed if the webcam depth feels off.",
            0.0,
            5000.0,
            1.0,
            0.0,
            get_webcam_focal,
            set_webcam_focal,
        ),
        make_float_setting(
            c"BaselineOffsetX",
            c"Baseline Offset X (mm)",
            c"Trim added to the captured neutral head position, left/right",
            -500.0,
            500.0,
            1.0,
            0.0,
            get_offset_x,
            set_offset_x,
        ),
        make_float_setting(
            c"BaselineOffsetY",
            c"Baseline Offset Y (mm)",
            c"Trim added to the captured neutral head position, up/down",
            -500.0,
            500.0,
            1.0,
            0.0,
            get_offset_y,
            set_offset_y,
        ),
        make_float_setting(
            c"BaselineOffsetZ",
            c"Baseline Offset Z (mm)",
            c"Trim added to the captured neutral head position, closer/farther",
            -500.0,
            500.0,
            1.0,
            0.0,
            get_offset_z,
            set_offset_z,
        ),
    ];
    for def in defs {
        // SAFETY: caller guarantees `api`/`endpoint_id`; `def` is leaked and
        // outlives the host's use of it.
        unsafe { register(endpoint_id, def) };
    }
}
