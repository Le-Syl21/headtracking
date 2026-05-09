//! Plugin configuration backed by VPX's `[Plugin.HeadTracking]` settings.
//!
//! Each setting is registered with the host via `MsgPluginAPI::RegisterSetting`
//! at `PluginLoad`. VPX then calls our `Set` callback with the value parsed
//! from `~/.vpinball/VPinballX.ini`, falling back to the declared default if
//! the key is missing. The value is mirrored into a global `RwLock<Config>`
//! that the rest of the plugin reads at session-spawn time and per-frame.
//!
//! Settings live for the lifetime of the process: the `MsgSettingDef` we
//! pass to `RegisterSetting` is `Box::leak`-ed because the host keeps the
//! pointer for as long as it cares about the setting (e.g. when re-saving).

use std::ffi::{CStr, c_char, c_int};
use std::sync::RwLock;

use crate::plugin::vpx_sys::{
    MSGPI_SETTING_TYPE_FLOAT, MSGPI_SETTING_TYPE_INT, MsgPluginAPI, MsgSettingDef,
    MsgSettingDef__bindgen_ty_1, MsgSettingDef__bindgen_ty_1__bindgen_ty_1 as FloatDef,
    MsgSettingDef__bindgen_ty_1__bindgen_ty_2 as IntDef,
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

/// All plugin settings, mirrored from VPX into the process. Reads happen
/// from the tracker thread (config snapshot at spawn) and the OnPrepareFrame
/// callback (gain + baseline offset, cheap RwLock read).
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub backend: BackendKind,
    pub device_index: i32,
    pub gain: f32,
    pub min_cutoff_hz: f32,
    pub beta: f32,
    pub baseline_offset_x_mm: f32,
    pub baseline_offset_y_mm: f32,
    pub baseline_offset_z_mm: f32,
    /// World distance between the player's hand centroids when both
    /// rest on the flipper buttons. Drives the hand-as-fiducial
    /// calibration (see `src/calibration/hand_fiducial.rs`). Default
    /// 660 mm is slightly less than the 700 mm lockbar width on
    /// Sylvain's widebody — hands wrap inboard of the button posts.
    pub lockbar_hand_span_mm: f32,
    /// Distance from the floor to the top of the lockbar. Used by
    /// downstream code to convert hand-Y to world Y. 850 mm = standard
    /// widebody pincab.
    pub lockbar_floor_height_mm: f32,
    /// Anatomical interocular distance, used by face-bbox-based
    /// distance estimation. Adult mean ≈ 63 mm; user can lower to
    /// ~55 mm for kids or raise to ~68 mm for very tall players.
    pub ipd_mm: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            backend: BackendKind::Auto,
            device_index: 0,
            gain: 1.0,
            // Same defaults as headtracking-demo — Z gets a tighter cutoff because
            // depth-camera Z is inherently noisier than X/Y.
            min_cutoff_hz: 0.4,
            beta: 0.05,
            baseline_offset_x_mm: 0.0,
            baseline_offset_y_mm: 0.0,
            baseline_offset_z_mm: 0.0,
            lockbar_hand_span_mm: 660.0,
            lockbar_floor_height_mm: 850.0,
            ipd_mm: 63.0,
        }
    }
}

static CONFIG: RwLock<Config> = RwLock::new(Config {
    backend: BackendKind::Auto,
    device_index: 0,
    gain: 1.0,
    min_cutoff_hz: 0.4,
    beta: 0.05,
    baseline_offset_x_mm: 0.0,
    baseline_offset_y_mm: 0.0,
    baseline_offset_z_mm: 0.0,
    lockbar_hand_span_mm: 660.0,
    lockbar_floor_height_mm: 850.0,
    ipd_mm: 63.0,
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

unsafe extern "C" fn get_backend() -> c_int {
    current().backend.to_i32()
}
unsafe extern "C" fn set_backend(v: c_int) {
    let mut c = CONFIG.write().expect("config rwlock poisoned");
    c.backend = BackendKind::from_i32(v);
}

unsafe extern "C" fn get_device_index() -> c_int {
    current().device_index
}
unsafe extern "C" fn set_device_index(v: c_int) {
    CONFIG.write().expect("config rwlock poisoned").device_index = v.max(0);
}

unsafe extern "C" fn get_gain() -> f32 {
    current().gain
}
unsafe extern "C" fn set_gain(v: f32) {
    CONFIG.write().expect("config rwlock poisoned").gain = v;
}

unsafe extern "C" fn get_min_cutoff() -> f32 {
    current().min_cutoff_hz
}
unsafe extern "C" fn set_min_cutoff(v: f32) {
    CONFIG
        .write()
        .expect("config rwlock poisoned")
        .min_cutoff_hz = v.max(0.01);
}

unsafe extern "C" fn get_beta() -> f32 {
    current().beta
}
unsafe extern "C" fn set_beta(v: f32) {
    CONFIG.write().expect("config rwlock poisoned").beta = v.max(0.0);
}

unsafe extern "C" fn get_offset_x() -> f32 {
    current().baseline_offset_x_mm
}
unsafe extern "C" fn set_offset_x(v: f32) {
    CONFIG
        .write()
        .expect("config rwlock poisoned")
        .baseline_offset_x_mm = v;
}

unsafe extern "C" fn get_offset_y() -> f32 {
    current().baseline_offset_y_mm
}
unsafe extern "C" fn set_offset_y(v: f32) {
    CONFIG
        .write()
        .expect("config rwlock poisoned")
        .baseline_offset_y_mm = v;
}

unsafe extern "C" fn get_offset_z() -> f32 {
    current().baseline_offset_z_mm
}
unsafe extern "C" fn set_offset_z(v: f32) {
    CONFIG
        .write()
        .expect("config rwlock poisoned")
        .baseline_offset_z_mm = v;
}

unsafe extern "C" fn get_lockbar_hand_span() -> f32 {
    current().lockbar_hand_span_mm
}
unsafe extern "C" fn set_lockbar_hand_span(v: f32) {
    CONFIG
        .write()
        .expect("config rwlock poisoned")
        .lockbar_hand_span_mm = v.max(100.0);
}

unsafe extern "C" fn get_lockbar_floor_height() -> f32 {
    current().lockbar_floor_height_mm
}
unsafe extern "C" fn set_lockbar_floor_height(v: f32) {
    CONFIG
        .write()
        .expect("config rwlock poisoned")
        .lockbar_floor_height_mm = v.max(0.0);
}

unsafe extern "C" fn get_ipd() -> f32 {
    current().ipd_mm
}
unsafe extern "C" fn set_ipd(v: f32) {
    // 40-80 mm bracket covers everyone from young kids to outliers.
    CONFIG.write().expect("config rwlock poisoned").ipd_mm = v.clamp(40.0, 80.0);
}

// ============================================================ register_settings
//
// Static label table for the Backend enum — referenced as a `*const *const
// c_char`. The trailing NULL is required by VPX to know where the array ends.
// Strings live for the lifetime of the process (they're plain `c"..."`
// literals).

/// Wrapper that lets us put a `*const c_char` array in a `static` —
/// raw pointers aren't `Sync` by default, but every pointer in the array
/// points at a static C-string literal which is read-only and lives
/// forever, and the host only reads through them.
#[repr(transparent)]
struct BackendValues([*const c_char; 5]);
// SAFETY: see the type doc — all pointers target read-only static memory.
unsafe impl Sync for BackendValues {}

static BACKEND_VALUES: BackendValues = BackendValues([
    c"Auto".as_ptr(),
    c"Kinect v2".as_ptr(),
    c"Kinect v1".as_ptr(),
    c"Webcam".as_ptr(),
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

    let backend = make_int_setting(
        c"Backend",
        c"Backend",
        c"Tracker backend (Auto picks the first available: Kinect v2 → v1 → Webcam)",
        0,
        3,
        BACKEND_AUTO,
        BACKEND_VALUES.0.as_ptr().cast_mut(),
        get_backend,
        set_backend,
    );
    let device = make_int_setting(
        c"DeviceIndex",
        c"Device Index",
        c"0-based index when the host has several Kinect v1 sensors or webcams (ignored for Kinect v2: always uses the first device)",
        0,
        7,
        0,
        std::ptr::null_mut(),
        get_device_index,
        set_device_index,
    );
    let gain = make_float_setting(
        c"Gain",
        c"Gain",
        c"Multiplier on the head-motion delta before it's applied to the camera",
        0.0,
        5.0,
        0.05,
        1.0,
        get_gain,
        set_gain,
    );
    let cutoff = make_float_setting(
        c"MinCutoffHz",
        c"Min Cutoff (Hz)",
        c"1€ filter baseline cutoff for Z (lower = more smoothing when still)",
        0.05,
        5.0,
        0.05,
        0.4,
        get_min_cutoff,
        set_min_cutoff,
    );
    let beta = make_float_setting(
        c"Beta",
        c"Beta",
        c"1€ filter responsiveness to fast motion (higher = less lag, more jitter)",
        0.0,
        1.0,
        0.005,
        0.05,
        get_beta,
        set_beta,
    );
    let off_x = make_float_setting(
        c"BaselineOffsetX",
        c"Baseline Offset X (mm)",
        c"Lateral correction added to the captured neutral pose",
        -500.0,
        500.0,
        1.0,
        0.0,
        get_offset_x,
        set_offset_x,
    );
    let off_y = make_float_setting(
        c"BaselineOffsetY",
        c"Baseline Offset Y (mm)",
        c"Vertical correction added to the captured neutral pose",
        -500.0,
        500.0,
        1.0,
        0.0,
        get_offset_y,
        set_offset_y,
    );
    let off_z = make_float_setting(
        c"BaselineOffsetZ",
        c"Baseline Offset Z (mm)",
        c"Depth correction added to the captured neutral pose",
        -500.0,
        500.0,
        1.0,
        0.0,
        get_offset_z,
        set_offset_z,
    );
    let hand_span = make_float_setting(
        c"LockbarHandSpan",
        c"Lockbar Hand Span (mm)",
        c"World distance between the player's hand centroids on the flipper buttons. \
          Drives the hand-as-fiducial calibration. ~660 mm on a standard widebody (slightly \
          inboard of the actual lockbar).",
        300.0,
        1000.0,
        1.0,
        660.0,
        get_lockbar_hand_span,
        set_lockbar_hand_span,
    );
    let floor_h = make_float_setting(
        c"LockbarFloorHeight",
        c"Lockbar Floor Height (mm)",
        c"Distance from the floor to the top of the lockbar — used to convert hand-Y to \
          world Y. 850 mm on a standard widebody.",
        500.0,
        1500.0,
        1.0,
        850.0,
        get_lockbar_floor_height,
        set_lockbar_floor_height,
    );
    let ipd = make_float_setting(
        c"IPDmm",
        c"Interpupillary Distance (mm)",
        c"Distance between the eye pupils — used by face-bbox-based depth estimation. \
          Adult mean ≈ 63 mm; lower for kids (~55), raise for outliers (~68).",
        40.0,
        80.0,
        0.5,
        63.0,
        get_ipd,
        set_ipd,
    );

    // SAFETY: `register` is a valid C function pointer per the API contract;
    // each pointer is a freshly leaked Box<MsgSettingDef> that lives forever.
    unsafe {
        register(endpoint_id, backend);
        register(endpoint_id, device);
        register(endpoint_id, gain);
        register(endpoint_id, cutoff);
        register(endpoint_id, beta);
        register(endpoint_id, off_x);
        register(endpoint_id, off_y);
        register(endpoint_id, off_z);
        register(endpoint_id, hand_span);
        register(endpoint_id, floor_h);
        register(endpoint_id, ipd);
    }
}
