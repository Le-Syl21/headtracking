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
    /// Drive the two One-Euro knobs yourself (`SmoothingResponsiveness` and
    /// `SmoothingCatchUp`). The per-axis profile the presets carry is kept —
    /// see [`Config::one_euro_params`].
    Custom,
}

impl SmoothingPreset {
    fn from_i32(v: i32) -> Self {
        match v {
            0 => Self::Stable,
            2 => Self::Reactive,
            3 => Self::Custom,
            _ => Self::Normal,
        }
    }
    fn to_i32(self) -> i32 {
        match self {
            Self::Stable => 0,
            Self::Normal => 1,
            Self::Reactive => 2,
            Self::Custom => 3,
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
    /// Per-axis trims multiplying [`Self::gain`], `1.0` = no change. The
    /// master stays a single number so existing configs keep behaving
    /// exactly as before; these only let one direction be dialled back when
    /// a cabinet needs it (a shallow playfield often wants less near/far).
    pub gain_x: f32,
    pub gain_y: f32,
    pub gain_z: f32,
    pub smoothing: SmoothingPreset,
    /// One-Euro `min_cutoff` for the left/right axis under
    /// [`SmoothingPreset::Custom`]; ignored by the three presets.
    pub smoothing_responsiveness: f32,
    /// One-Euro `beta` for the left/right axis under
    /// [`SmoothingPreset::Custom`]; ignored by the three presets.
    pub smoothing_catch_up: f32,
    /// Median spike-gate window in frames (odd, 1 = off); see
    /// `filter::MedianGate` for the latency trade-off.
    pub median_window: i32,
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

/// Range of the `SmoothingResponsiveness` setting (One-Euro `min_cutoff`, Hz).
const MIN_CUTOFF_RANGE: (f32, f32) = (0.05, 5.0);
/// Range of the `SmoothingCatchUp` setting (One-Euro `beta`).
const BETA_RANGE: (f32, f32) = (0.0, 1.5);
/// Range of the per-axis gain trims.
const GAIN_TRIM_RANGE: (f32, f32) = (0.0, 3.0);

/// One-Euro parameters for one axis.
#[derive(Debug, Clone, Copy)]
pub struct AxisParams {
    pub min_cutoff_hz: f32,
    pub beta: f32,
}

impl Config {
    /// Median gate window as a usize for `MedianGate`.
    #[must_use]
    pub fn median_window_frames(&self) -> usize {
        self.median_window.max(1) as usize
    }

    /// Per-axis One-Euro parameters `[x, y, z]` for the active preset.
    ///
    /// The three axes are tuned independently: X (lateral) is the visible
    /// parallax and gets the most responsiveness; Y (height) matters less
    /// and can sit calmer; Z (depth) is the noisiest reading AND the one
    /// that re-skews the whole Window-mode projection, so it gets the
    /// tightest cutoff of all.
    #[must_use]
    pub fn one_euro_params(&self) -> [AxisParams; 3] {
        let p = |min_cutoff_hz: f32, beta: f32| AxisParams {
            min_cutoff_hz,
            beta,
        };
        match self.smoothing {
            SmoothingPreset::Stable => [p(0.25, 0.002), p(0.2, 0.002), p(0.1, 0.006)],
            SmoothingPreset::Normal => [p(1.0, 0.01), p(0.8, 0.008), p(0.4, 0.05)],
            SmoothingPreset::Reactive => [p(2.0, 0.05), p(1.6, 0.04), p(0.8, 0.1)],
            // Custom drives the left/right axis from the two settings and
            // keeps the shape the three presets share, rather than flattening
            // all three axes onto one pair of numbers — which would silently
            // throw away the near/far tuning that makes the projection sit
            // still. The cutoff ratios (0.8 up/down, 0.4 near/far) are
            // identical across all three presets. The near/far beta ratio is
            // the one that varies (x3 Stable, x5 Normal, x2 Reactive), so
            // Custom takes the middle and clamps to the setting's own range.
            SmoothingPreset::Custom => {
                let (c, b) = (
                    self.smoothing_responsiveness
                        .clamp(MIN_CUTOFF_RANGE.0, MIN_CUTOFF_RANGE.1),
                    self.smoothing_catch_up.clamp(BETA_RANGE.0, BETA_RANGE.1),
                );
                [
                    p(c, b),
                    p(c * 0.8, b * 0.8),
                    p(c * 0.4, (b * 3.0).min(BETA_RANGE.1)),
                ]
            }
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            backend: BackendKind::Auto,
            device_index: 0,
            gain: 1.0,
            gain_x: 1.0,
            gain_y: 1.0,
            gain_z: 1.0,
            smoothing: SmoothingPreset::Stable,
            smoothing_responsiveness: 1.0,
            smoothing_catch_up: 0.01,
            median_window: 3,
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
    gain_x: 1.0,
    gain_y: 1.0,
    gain_z: 1.0,
    smoothing: SmoothingPreset::Stable,
    smoothing_responsiveness: 1.0,
    smoothing_catch_up: 0.01,
    median_window: 3,
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

unsafe extern "C" fn get_gain_x() -> f32 {
    current().gain_x
}
unsafe extern "C" fn set_gain_x(v: f32) {
    rw!().gain_x = v;
}

unsafe extern "C" fn get_gain_y() -> f32 {
    current().gain_y
}
unsafe extern "C" fn set_gain_y(v: f32) {
    rw!().gain_y = v;
}

unsafe extern "C" fn get_gain_z() -> f32 {
    current().gain_z
}
unsafe extern "C" fn set_gain_z(v: f32) {
    rw!().gain_z = v;
}

unsafe extern "C" fn get_smoothing_responsiveness() -> f32 {
    current().smoothing_responsiveness
}
unsafe extern "C" fn set_smoothing_responsiveness(v: f32) {
    rw!().smoothing_responsiveness = v;
}

unsafe extern "C" fn get_smoothing_catch_up() -> f32 {
    current().smoothing_catch_up
}
unsafe extern "C" fn set_smoothing_catch_up(v: f32) {
    rw!().smoothing_catch_up = v;
}

unsafe extern "C" fn get_smoothing() -> c_int {
    current().smoothing.to_i32()
}
unsafe extern "C" fn set_smoothing(v: c_int) {
    rw!().smoothing = SmoothingPreset::from_i32(v);
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

unsafe extern "C" fn get_median_window() -> c_int {
    current().median_window
}
unsafe extern "C" fn set_median_window(v: c_int) {
    rw!().median_window = v;
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

static SMOOTHING_VALUES: EnumValues<5> = EnumValues([
    c"Stable".as_ptr(),
    c"Normal".as_ptr(),
    c"Reactive".as_ptr(),
    c"Custom".as_ptr(),
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
pub unsafe fn register_settings(api: &MsgPluginAPI, endpoint_id: u32, webcam_names: &[String]) {
    let Some(register) = api.RegisterSetting else {
        return;
    };

    // When the webcam backend enumerated devices at load time, DeviceIndex
    // becomes a dropdown labelled with the real product names (the index
    // semantics — SDL order — stay identical, so ini values carry over).
    // The labels are leaked: the host keeps the pointers for the process
    // lifetime, exactly like the &'static arrays below.
    let (device_values, device_max): (*mut *const c_char, c_int) = if webcam_names.is_empty() {
        (std::ptr::null_mut(), 7)
    } else {
        let mut ptrs: Vec<*const c_char> = webcam_names
            .iter()
            .map(|n| {
                let cleaned: String = n.chars().filter(|c| *c != '\0').collect();
                let label =
                    std::ffi::CString::new(cleaned).expect("NULs filtered out of camera name");
                Box::leak(label.into_boxed_c_str()).as_ptr()
            })
            .collect();
        ptrs.push(std::ptr::null());
        let max = (webcam_names.len() - 1) as c_int;
        (Box::leak(ptrs.into_boxed_slice()).as_mut_ptr(), max)
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
            c"Camera",
            c"Camera used by the Webcam backend (Kinects always use the first Kinect found)",
            0,
            device_max,
            0,
            device_values,
            get_device_index,
            set_device_index,
        ),
        make_float_setting(
            c"Gain",
            c"Gain (all axes)",
            c"Multiplier on the head-motion delta before it's applied to the camera. One value for the three axes: left/right, up/down and near/far alike.",
            0.0,
            5.0,
            0.05,
            1.0,
            get_gain,
            set_gain,
        ),
        make_float_setting(
            c"GainX",
            c"Gain trim, left/right",
            c"Per-axis trim multiplying the master Gain. 1.00 leaves it alone; lower it when one direction moves too much for your cabinet.",
            GAIN_TRIM_RANGE.0,
            GAIN_TRIM_RANGE.1,
            0.05,
            1.0,
            get_gain_x,
            set_gain_x,
        ),
        make_float_setting(
            c"GainY",
            c"Gain trim, up/down",
            c"Per-axis trim multiplying the master Gain. 1.00 leaves it alone; lower it when one direction moves too much for your cabinet.",
            GAIN_TRIM_RANGE.0,
            GAIN_TRIM_RANGE.1,
            0.05,
            1.0,
            get_gain_y,
            set_gain_y,
        ),
        make_float_setting(
            c"GainZ",
            c"Gain trim, near/far",
            c"Per-axis trim multiplying the master Gain. 1.00 leaves it alone; near/far is the one most often worth calming, since it re-skews the whole projection.",
            GAIN_TRIM_RANGE.0,
            GAIN_TRIM_RANGE.1,
            0.05,
            1.0,
            get_gain_z,
            set_gain_z,
        ),
        make_int_setting(
            c"Smoothing",
            c"Smoothing",
            c"Stable is the field-tested default (kills every tremor for a touch of lag), Normal follows quicker, Reactive follows fast moves closest. Each preset is tuned per axis already: near/far is smoothed hardest, since it is the noisiest reading and the one that re-skews the whole projection. Custom hands you the two knobs below and keeps that same per-axis shape.",
            0,
            3,
            SmoothingPreset::Stable.to_i32(),
            SMOOTHING_VALUES.0.as_ptr().cast_mut(),
            get_smoothing,
            set_smoothing,
        ),
        make_float_setting(
            c"SmoothingResponsiveness",
            c"Custom smoothing: responsiveness",
            c"Only used when Smoothing is set to Custom. While you hold still: low = rock steady with a touch of lag, high = follows sooner and may tremble. The up/down and near/far axes follow this at the same ratios the presets use.",
            MIN_CUTOFF_RANGE.0,
            MIN_CUTOFF_RANGE.1,
            0.05,
            1.0,
            get_smoothing_responsiveness,
            set_smoothing_responsiveness,
        ),
        make_float_setting(
            c"SmoothingCatchUp",
            c"Custom smoothing: motion catch-up",
            c"Only used when Smoothing is set to Custom. When you move fast: higher = catches up quicker so the view sticks to your head. 0 turns the catch-up off and leaves plain smoothing.",
            BETA_RANGE.0,
            BETA_RANGE.1,
            0.01,
            0.01,
            get_smoothing_catch_up,
            set_smoothing_catch_up,
        ),
        make_int_setting(
            c"MedianWindow",
            c"Median Window",
            c"Frames of median pre-filtering that erase tracking spikes (1 = off); each extra frame adds ~17 ms of latency at 60 fps",
            1,
            9,
            3,
            std::ptr::null_mut(),
            get_median_window,
            set_median_window,
        ),
        make_bool_setting(
            c"InvertX",
            c"Invert X (left/right)",
            c"Flip the left/right response (for mirrored or unusual camera mountings)",
            false,
            get_invert_x,
            set_invert_x,
        ),
        make_bool_setting(
            c"InvertY",
            c"Invert Y (up/down)",
            c"Flip the up/down response",
            false,
            get_invert_y,
            set_invert_y,
        ),
        make_bool_setting(
            c"InvertZ",
            c"Invert Z (near/far)",
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
            c"Baseline Offset X, left/right (mm)",
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
            c"Baseline Offset Y, up/down (mm)",
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
            c"Baseline Offset Z, near/far (mm)",
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Every preset keeps the same per-axis shape: up/down is calmer than
    /// left/right, near/far calmer still. If a preset ever flattens, the
    /// projection starts breathing on the axis that skews it.
    #[test]
    fn presets_keep_near_far_the_calmest_axis() {
        for preset in [
            SmoothingPreset::Stable,
            SmoothingPreset::Normal,
            SmoothingPreset::Reactive,
        ] {
            let cfg = Config {
                smoothing: preset,
                ..Config::default()
            };
            let [x, y, z] = cfg.one_euro_params();
            assert!(y.min_cutoff_hz < x.min_cutoff_hz, "{preset:?}");
            assert!(z.min_cutoff_hz < y.min_cutoff_hz, "{preset:?}");
        }
    }

    /// Custom drives the axes from the two settings and keeps that same
    /// shape, instead of flattening all three onto one pair of numbers.
    #[test]
    fn custom_smoothing_follows_the_two_knobs() {
        let cfg = Config {
            smoothing: SmoothingPreset::Custom,
            smoothing_responsiveness: 2.0,
            smoothing_catch_up: 0.1,
            ..Config::default()
        };
        let [x, y, z] = cfg.one_euro_params();
        assert!((x.min_cutoff_hz - 2.0).abs() < 1e-6);
        assert!((x.beta - 0.1).abs() < 1e-6);
        assert!(y.min_cutoff_hz < x.min_cutoff_hz);
        assert!(z.min_cutoff_hz < y.min_cutoff_hz);
    }

    /// Out-of-range values reaching the config (a hand-edited ini, a host
    /// that does not clamp) must not produce a filter that behaves wildly.
    #[test]
    fn custom_smoothing_clamps_what_it_is_given() {
        let cfg = Config {
            smoothing: SmoothingPreset::Custom,
            smoothing_responsiveness: 999.0,
            smoothing_catch_up: 999.0,
            ..Config::default()
        };
        let [x, _, z] = cfg.one_euro_params();
        assert!((x.min_cutoff_hz - MIN_CUTOFF_RANGE.1).abs() < 1e-6);
        assert!(x.beta <= BETA_RANGE.1);
        // Near/far triples the catch-up, so it is the one that would run away.
        assert!(z.beta <= BETA_RANGE.1);
    }

    /// The preset round-trips through the i32 the host stores, Custom
    /// included — a value the old builds never wrote.
    #[test]
    fn smoothing_preset_round_trips() {
        for preset in [
            SmoothingPreset::Stable,
            SmoothingPreset::Normal,
            SmoothingPreset::Reactive,
            SmoothingPreset::Custom,
        ] {
            assert_eq!(SmoothingPreset::from_i32(preset.to_i32()), preset);
        }
    }

    /// A config that only ever set the master gain must behave exactly as it
    /// did before the trims existed.
    #[test]
    fn gain_trims_default_to_no_change() {
        let cfg = Config::default();
        assert!((cfg.gain_x - 1.0).abs() < 1e-6);
        assert!((cfg.gain_y - 1.0).abs() < 1e-6);
        assert!((cfg.gain_z - 1.0).abs() < 1e-6);
    }
}
