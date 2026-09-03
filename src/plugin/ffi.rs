//! C ABI surface exposed to VPX.
//!
//! Two symbols, named after the plugin id (`HeadTracking`) declared in
//! `plugin.cfg`. VPX resolves them via `dlsym("HeadTrackingPluginLoad")` /
//! `dlsym("HeadTrackingPluginUnload")` — see `MsgPluginManager.cpp`.

use std::ffi::c_void;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::ptr;

use parking_lot::Mutex;
use tracing::{error, info, warn};

use std::time::{Duration, Instant};

use super::messages::{
    VPXPI_EVT_ON_ACTION_CHANGED, VPXPI_EVT_ON_GAME_END, VPXPI_EVT_ON_GAME_START,
    VPXPI_EVT_ON_PREPARE_FRAME, VPXPI_MSG_GET_API, VPXPI_NAMESPACE,
};
use super::vpx_sys::{
    MsgPluginAPI, VPXAction_VPXACTION_Lockbar, VPXActionEvent, VPXPluginAPI, VPXViewSetupDef,
};
use crate::camera::mapping::{MappingParams, ViewMode, pose_delta_to_view_delta};
use crate::config;
use crate::tracker::session::TrackerSession;
use crate::tracker::{Pose, TrackingFault};

/// Lifecycle state captured at `PluginLoad` and torn down at `PluginUnload`.
struct PluginState {
    msg_api: *const MsgPluginAPI,
    vpx_api: *mut VPXPluginAPI,
    /// Per-user preference path (`VPinballX.ini` home), copied out of
    /// `GetVpxInfo` at load time (host strings are copied immediately).
    pref_path: Option<std::path::PathBuf>,
    /// Reserved for cross-thread API calls (`RunOnMainThread`, `SendMsg`).
    #[allow(dead_code)]
    endpoint_id: u32,
    msg_ids: SubscribedMsgs,
}

#[derive(Default)]
struct SubscribedMsgs {
    get_vpx_api: u32,
    on_game_start: u32,
    on_game_end: u32,
    on_prepare_frame: u32,
    on_action_changed: u32,
}

// SAFETY: pointers live for the lifetime of the host process and are only
// dereferenced on threads driven by the host (which guarantees serialization
// for non-`RunOnMainThread` API calls).
unsafe impl Send for PluginState {}

static STATE: Mutex<Option<PluginState>> = Mutex::new(None);

/// Live tracker for the current game session. `Some` between `OnGameStart`
/// and `OnGameEnd`, otherwise `None`.
struct GameSession {
    tracker: TrackerSession,
    /// First valid pose seen this game, plus the matching `(viewX, viewY, viewZ)`
    /// snapshot. Subsequent frames apply the head-motion delta on top of this
    /// baseline so the table's authored POV is the neutral resting state.
    baseline: Option<Baseline>,
    /// View delta applied on the previous frame — the easing state used to
    /// settle back toward the baseline when tracking is lost.
    applied: [f32; 3],
    /// Timestamp of the newest pose sample and when it last changed —
    /// an unchanged sample for [`TRACKING_STALE`] means the tracker lost
    /// the player (backends keep publishing nothing, the ArcSwap holds).
    last_pose_ts: u64,
    last_pose_seen: Instant,
    /// Lockbar-button hold tracking for the long-press recenter.
    lockbar_held_since: Option<Instant>,
    recentered_this_hold: bool,
    /// One-shot text pushed as a native VPX notification on the next
    /// frame (startup summary; built off the main thread).
    pending_note: Option<std::ffi::CString>,
    /// Set once the live anchor calibration's camera-pose notification has
    /// been pushed (the tracker thread fills the result asynchronously).
    calib_notified: bool,
}

/// Hold the (already mapped everywhere) lockbar button this long to
/// recenter the head-tracking baseline.
const RECENTER_HOLD: Duration = Duration::from_secs(2);

/// A pose sample unchanged for this long counts as lost tracking.
const TRACKING_STALE: Duration = Duration::from_millis(400);

/// Per-frame decay applied to the view delta while tracking is lost —
/// ~0.94^60 ≈ 2 % after one second at 60 fps: a calm glide home.
const LOST_DECAY: f32 = 0.94;

/// How long the startup notification stays on screen — it carries the
/// active camera plus the setup reminders, so give it time to be read.
const STARTUP_NOTE_MS: i32 = 15_000;

#[derive(Clone, Copy)]
struct Baseline {
    pose: Pose,
    view_xyz: [f32; 3],
}

static GAME: Mutex<Option<GameSession>> = Mutex::new(None);

/// `dlsym` target invoked by VPX after loading the cdylib.
///
/// # Safety
/// Called by the host C++ code with a valid `MsgPluginAPI` pointer that lives
/// for the duration of the plugin session. We must not panic across the FFI
/// boundary, hence the `catch_unwind` shim.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn HeadTrackingPluginLoad(session_id: u32, api: *const MsgPluginAPI) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        init_tracing_once();
        let host = os_info::get();
        info!(
            session_id,
            version = env!("CARGO_PKG_VERSION"),
            os = %host.os_type(),
            os_version = %host.version(),
            arch = host.architecture().unwrap_or("unknown"),
            "HeadTracking plugin: load"
        );

        if api.is_null() {
            error!("MsgPluginAPI pointer is null at load time; aborting init");
            return;
        }

        // SAFETY: caller contract — `api` is non-null and valid for the
        // lifetime of the plugin session.
        if let Err(err) = unsafe { do_load(session_id, api) } {
            error!(?err, "HeadTracking plugin failed to initialize");
        }
    }));
}

/// `dlsym` target invoked by VPX before unloading the cdylib.
///
/// # Safety
/// Must release every `GetMsgID` and unsubscribe every `SubscribeMsg` made
/// during load — otherwise VPX's reference counts leak across reloads.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn HeadTrackingPluginUnload() {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        info!("HeadTracking plugin: unload");
        // Drop any in-flight game session before tearing down the message bus.
        GAME.lock().take();
        // SAFETY: matches the load contract.
        unsafe { do_unload() };
    }));
}

unsafe fn do_load(session_id: u32, api_ptr: *const MsgPluginAPI) -> Result<(), LoadError> {
    // SAFETY: caller guarantees `api_ptr` is valid for at least the duration
    // of this call (and beyond — the host keeps it live until unload).
    let api = unsafe { &*api_ptr };

    let get_msg_id = api.GetMsgID.ok_or(LoadError::MissingFunction("GetMsgID"))?;
    let subscribe = api
        .SubscribeMsg
        .ok_or(LoadError::MissingFunction("SubscribeMsg"))?;
    let broadcast = api
        .BroadcastMsg
        .ok_or(LoadError::MissingFunction("BroadcastMsg"))?;

    // Allocate message IDs we care about.
    // SAFETY: `get_msg_id` is a valid C function pointer per the API contract,
    // and the namespace/name pointers are static null-terminated strings.
    let msg_ids = SubscribedMsgs {
        get_vpx_api: unsafe { get_msg_id(VPXPI_NAMESPACE.as_ptr(), VPXPI_MSG_GET_API.as_ptr()) },
        on_game_start: unsafe {
            get_msg_id(VPXPI_NAMESPACE.as_ptr(), VPXPI_EVT_ON_GAME_START.as_ptr())
        },
        on_game_end: unsafe {
            get_msg_id(VPXPI_NAMESPACE.as_ptr(), VPXPI_EVT_ON_GAME_END.as_ptr())
        },
        on_prepare_frame: unsafe {
            get_msg_id(
                VPXPI_NAMESPACE.as_ptr(),
                VPXPI_EVT_ON_PREPARE_FRAME.as_ptr(),
            )
        },
        on_action_changed: unsafe {
            get_msg_id(
                VPXPI_NAMESPACE.as_ptr(),
                VPXPI_EVT_ON_ACTION_CHANGED.as_ptr(),
            )
        },
    };

    // Resolve the VPXPluginAPI via the GetAPI broadcast — the host responds
    // synchronously by writing the API pointer into our out-parameter.
    let mut vpx_api: *mut VPXPluginAPI = ptr::null_mut();
    // SAFETY: BroadcastMsg writes the api pointer into `vpx_api` if a host
    // is listening on the GetAPI channel.
    unsafe {
        broadcast(
            session_id,
            msg_ids.get_vpx_api,
            (&raw mut vpx_api).cast::<c_void>(),
        );
    }
    if vpx_api.is_null() {
        return Err(LoadError::NoVpxApi);
    }
    info!("VPX plugin API resolved");

    // Copy the host paths now — the plugin later reads VPinballX.ini
    // (cabinet lockbar geometry) from them.
    let pref_path = {
        let mut info = super::vpx_sys::VPXInfo::default();
        // SAFETY: vpx_api verified non-null; `info` is a valid out-pointer.
        if let Some(get_info) = unsafe { (*vpx_api).GetVpxInfo } {
            unsafe { get_info(&raw mut info) };
        }
        let to_path = |p: *const std::ffi::c_char| {
            if p.is_null() {
                None
            } else {
                // SAFETY: host guarantees a NUL-terminated string for the
                // duration of the call; we copy it right away.
                let s = unsafe { std::ffi::CStr::from_ptr(p) }.to_string_lossy();
                (!s.is_empty()).then(|| std::path::PathBuf::from(s.into_owned()))
            }
        };
        let _ = info.path; // install path: nothing reads it today
        to_path(info.prefPath)
    };
    info!(?pref_path, "host preference path");

    // Wire the tracing → VPX console bridge as soon as MsgPluginAPI is
    // available. From this point on every `info!` / `warn!` / `error!`
    // emitted by the plugin appears in VPX's plugin log panel as well
    // as on stderr. SAFETY: `api` is a live `&MsgPluginAPI` for the
    // duration of the plugin session, see top-of-file contract.
    unsafe { super::logging::resolve_and_install(api, session_id) };

    // Subscribe to game lifecycle + per-frame hook.
    // SAFETY: callbacks are FFI-safe (extern "C" fn with the documented
    // signature), userData is null because we route all state through globals.
    unsafe {
        subscribe(
            session_id,
            msg_ids.on_game_start,
            Some(on_game_start),
            ptr::null_mut(),
        );
        subscribe(
            session_id,
            msg_ids.on_game_end,
            Some(on_game_end),
            ptr::null_mut(),
        );
        subscribe(
            session_id,
            msg_ids.on_prepare_frame,
            Some(on_prepare_frame),
            ptr::null_mut(),
        );
        subscribe(
            session_id,
            msg_ids.on_action_changed,
            Some(on_action_changed),
            ptr::null_mut(),
        );
    }

    // Register all `[Plugin.HeadTracking]` settings. The host calls our
    // Set callbacks immediately with the value parsed from VPinballX.ini
    // (or the declared default when the key is missing), so by the time
    // OnGameStart fires the config snapshot is already populated.
    // SAFETY: api_ptr is non-null and live for the plugin session.
    // Enumerate the webcams once at load so the Camera setting can show
    // real product names instead of a bare index.
    #[cfg(feature = "webcam")]
    let webcam_names = crate::tracker::webcam::list_cameras();
    #[cfg(not(feature = "webcam"))]
    let webcam_names: Vec<String> = Vec::new();
    if !webcam_names.is_empty() {
        info!(cameras = ?webcam_names, "webcams enumerated for the settings dropdown");
    }
    unsafe { config::register_settings(&*api_ptr, session_id, &webcam_names) };

    *STATE.lock() = Some(PluginState {
        msg_api: api_ptr,
        vpx_api,
        pref_path,
        endpoint_id: session_id,
        msg_ids,
    });
    Ok(())
}

unsafe fn do_unload() {
    // Tear down the logging bridge first: the host will reclaim the
    // LoggingPluginAPI struct as soon as we return from this function,
    // and any tracing event fired during the rest of the unload (we
    // emit a couple ourselves) must not dereference a freed pointer.
    super::logging::clear();

    let Some(state) = STATE.lock().take() else {
        warn!("unload called but plugin state was already empty");
        return;
    };

    // SAFETY: stored pointer was non-null at load time and the host keeps it
    // alive at least until the unload returns.
    let api = unsafe { &*state.msg_api };
    let unsubscribe = api.UnsubscribeMsg;
    let release = api.ReleaseMsgID;

    if let (Some(unsubscribe), Some(release)) = (unsubscribe, release) {
        // SAFETY: we registered each of these subscriptions in `do_load`.
        unsafe {
            unsubscribe(
                state.msg_ids.on_game_start,
                Some(on_game_start),
                ptr::null_mut(),
            );
            unsubscribe(
                state.msg_ids.on_game_end,
                Some(on_game_end),
                ptr::null_mut(),
            );
            unsubscribe(
                state.msg_ids.on_prepare_frame,
                Some(on_prepare_frame),
                ptr::null_mut(),
            );
            unsubscribe(
                state.msg_ids.on_action_changed,
                Some(on_action_changed),
                ptr::null_mut(),
            );
            release(state.msg_ids.on_game_start);
            release(state.msg_ids.on_game_end);
            release(state.msg_ids.on_prepare_frame);
            release(state.msg_ids.on_action_changed);
            release(state.msg_ids.get_vpx_api);
        }
    } else {
        error!("VPX did not expose UnsubscribeMsg/ReleaseMsgID at unload time");
    }
}

extern "C" fn on_game_start(_msg_id: u32, _context: *mut c_void, _data: *mut c_void) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        info!("VPX event: OnGameStart");
        let cfg = config::current();
        info!(
            backend = ?cfg.backend,
            device_index = cfg.device_index,
            gain = cfg.gain,
            gain_trim = ?[cfg.gain_x, cfg.gain_y, cfg.gain_z],
            smoothing = ?cfg.smoothing,
            "spawning tracker session with current config"
        );
        match TrackerSession::spawn(&cfg) {
            Ok(tracker) => {
                let backend = tracker.backend_name();
                // One-shot startup notification: which camera the plugin is
                // tracking on, the derived camera pose when a fixed anchor
                // calibration exists, and the host settings worth checking —
                // all without the player opening a single menu.
                let pending_note = startup_note(tracker.device_label(), backend, &cfg);
                *GAME.lock() = Some(GameSession {
                    tracker,
                    baseline: None,
                    applied: [0.0; 3],
                    last_pose_ts: 0,
                    last_pose_seen: Instant::now(),
                    lockbar_held_since: None,
                    recentered_this_hold: false,
                    pending_note,
                    calib_notified: false,
                });
                // A continuously moving camera invalidates VPX's static
                // prerender pass — disable it like the in-game POV page does.
                let vpx_ptr = STATE.lock().as_ref().map_or(ptr::null_mut(), |s| s.vpx_api);
                if !vpx_ptr.is_null() {
                    // SAFETY: API pointer valid for the plugin session.
                    if let Some(disable) = unsafe { (*vpx_ptr).DisableStaticPrerendering } {
                        // SAFETY: plain int argument per the FFI contract.
                        unsafe { disable(1) };
                    }
                }
                info!(backend, "tracker session active (static prerender off)");
            }
            Err(err) => {
                warn!(?err, "tracker session failed to start; running passthrough");
            }
        }
    }));
}

extern "C" fn on_game_end(_msg_id: u32, _context: *mut c_void, _data: *mut c_void) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        info!("VPX event: OnGameEnd");
        // Drop joins the tracker thread and tears the device down.
        GAME.lock().take();
    }));
}

extern "C" fn on_prepare_frame(_msg_id: u32, _context: *mut c_void, _data: *mut c_void) {
    let _ = catch_unwind(AssertUnwindSafe(apply_pose_to_view));
}

/// Observe the lockbar button (mapped on every cabinet): holding it
/// [`RECENTER_HOLD`] recenters the head-tracking baseline. The event is
/// observed, never consumed — VPX and the table keep seeing the button.
extern "C" fn on_action_changed(_msg_id: u32, _context: *mut c_void, data: *mut c_void) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        if data.is_null() {
            return;
        }
        // SAFETY: the host guarantees `data` points at a live VPXActionEvent
        // for the duration of this synchronous callback.
        let ev = unsafe { &*data.cast::<VPXActionEvent>() };
        if ev.action != VPXAction_VPXACTION_Lockbar {
            return;
        }
        let mut game_guard = GAME.lock();
        let Some(game) = game_guard.as_mut() else {
            return;
        };
        if ev.isPressed != 0 {
            if game.lockbar_held_since.is_none() {
                game.lockbar_held_since = Some(Instant::now());
                game.recentered_this_hold = false;
            }
        } else {
            game.lockbar_held_since = None;
            game.recentered_this_hold = false;
        }
    }));
}

fn apply_pose_to_view() {
    // 1. Snapshot the VPX API pointer (brief lock on STATE).
    let vpx_api_ptr = match STATE.lock().as_ref() {
        Some(s) => s.vpx_api,
        None => return,
    };
    if vpx_api_ptr.is_null() {
        return;
    }

    // 2. Hold the GAME lock for the rest. The tracker pose lookup is lock-free
    //    (ArcSwap), and OnPrepareFrame is single-threaded by VPX, so there is
    //    no contention.
    let mut game_guard = GAME.lock();
    let Some(game) = game_guard.as_mut() else {
        return;
    };

    // SAFETY: vpx_api_ptr was set non-null at load time and the host keeps
    // the API live through the plugin session.
    let vpx = unsafe { &*vpx_api_ptr };
    let Some(getter) = vpx.GetActiveViewSetup else {
        return;
    };
    let Some(setter) = vpx.SetActiveViewSetup else {
        return;
    };

    // One-shot startup notification (built at game start). Long enough to
    // read the setup reminders without pausing the game.
    if let Some(note) = game.pending_note.take()
        && let Some(notify) = vpx.PushNotification
    {
        // SAFETY: NUL-terminated CString kept alive across the call.
        unsafe { notify(note.as_ptr(), STARTUP_NOTE_MS) };
    }

    // The tracker thread gave up. Say it on screen: a session that cannot
    // reach its tracking stream used to degrade to something that half worked,
    // which reached us months later as a tracking bug rather than as the busy
    // device it was. Taken, so it fires once.
    if let Some(fault) = game.tracker.take_fault()
        && let Some(note) = fault_note(fault)
        && let Some(notify) = vpx.PushNotification
    {
        // SAFETY: NUL-terminated CString kept alive across the call.
        unsafe { notify(note.as_ptr(), STARTUP_NOTE_MS) };
    }

    // Camera-pose notification, once the tracker thread's RGB anchor
    // calibration lands (a few seconds into the game, or never).
    if !game.calib_notified
        && let Some(calib) = game.tracker.calibration()
    {
        game.calib_notified = true;
        // Read the host's own numbers back so the notification can show what
        // the pose was computed from, not just what came out of it.
        let mut view = VPXViewSetupDef::default();
        // SAFETY: getter follows the FFI contract; we own `view`.
        unsafe { getter(&raw mut view) };
        if let Some(note) =
            camera_pose_note(game.tracker.backend_name(), &calib, view.screenInclination)
            && let Some(notify) = vpx.PushNotification
        {
            // SAFETY: NUL-terminated CString kept alive across the call.
            unsafe { notify(note.as_ptr(), STARTUP_NOTE_MS) };
        }
    }

    // Long-press recenter: re-baseline on the CURRENT head position and
    // reset the smoothing filter so the new neutral isn't dragged from
    // the old smoothed state. One recenter per hold.
    if let Some(held) = game.lockbar_held_since
        && !game.recentered_this_hold
        && held.elapsed() >= RECENTER_HOLD
    {
        game.baseline = None;
        game.applied = [0.0; 3];
        game.tracker.reset_filter();
        game.recentered_this_hold = true;
        info!("lockbar long-press: head-tracking recentered");
        if let Some(notify) = vpx.PushNotification {
            // SAFETY: static NUL-terminated message, length per contract.
            unsafe { notify(c"Head tracking recentered".as_ptr(), 2000) };
        }
    }

    let pose = game.tracker.latest_pose();

    // Staleness: backends only publish when they SEE the player, so an
    // unchanged sample means lost tracking (looked away, left the cab).
    if pose.timestamp_us != game.last_pose_ts {
        game.last_pose_ts = pose.timestamp_us;
        game.last_pose_seen = Instant::now();
    }
    let lost = pose.confidence <= 0.0 || game.last_pose_seen.elapsed() > TRACKING_STALE;

    let mut view = VPXViewSetupDef::default();
    // SAFETY: `view` is a valid out-pointer; getter follows the FFI contract.
    unsafe { getter(&raw mut view) };

    let Some(baseline) = game.baseline else {
        if lost {
            return; // nothing to do until the first stable pose
        }
        // First frame with a real pose: capture both anchors and skip
        // applying any delta — the table's authored POV stands.
        game.baseline = Some(Baseline {
            pose: *pose,
            view_xyz: [view.viewX, view.viewY, view.viewZ],
        });
        info!(
            x = pose.position_mm[0],
            y = pose.position_mm[1],
            z = pose.position_mm[2],
            view_mode = view.viewMode,
            "tracker baseline captured"
        );
        if ViewMode::from_i32(view.viewMode) != ViewMode::Window
            && let Some(notify) = vpx.PushNotification
        {
            // Camera/Legacy translate the whole render — the table slides on
            // the screen. True in-table parallax needs the Window layout.
            // SAFETY: static NUL-terminated string.
            unsafe {
                notify(
                    c"Head tracking: set this table's POV layout to 'Window' for true in-table parallax".as_ptr(),
                    8000,
                )
            };
        }
        return;
    };

    // Read the live config every frame. The lock is uncontended (only the
    // VPX settings UI ever takes a write lock, and that's rare), so this
    // is essentially a memcpy.
    let cfg = config::current();

    if lost {
        // Glide calmly back toward the authored POV instead of freezing
        // on the last offset.
        game.applied = game.applied.map(|v| v * LOST_DECAY);
    } else {
        // Apply the user's BaselineOffset before computing the delta so
        // the offset acts as a manual recenter of the neutral pose, not
        // a post-gain bias.
        let mut adjusted_baseline_pose = baseline.pose;
        adjusted_baseline_pose.position_mm[0] += cfg.baseline_offset_x_mm;
        adjusted_baseline_pose.position_mm[1] += cfg.baseline_offset_y_mm;
        adjusted_baseline_pose.position_mm[2] += cfg.baseline_offset_z_mm;

        // The layout mode and the player→view rotation both come from the
        // HOST's view setup — nothing about the screen geometry is asked
        // of the user.
        let params = MappingParams {
            invert: [cfg.invert_x, cfg.invert_y, cfg.invert_z],
            mode: ViewMode::from_i32(view.viewMode),
            window_rot_rad: window_player_rotation(&view),
        };
        let delta = pose_delta_to_view_delta(&pose, &adjusted_baseline_pose, &params);
        // Master gain times the per-axis trim. Keeping the master as one
        // number means a config that only ever set `Gain` behaves exactly as
        // it did; the trims default to 1.0 and cost a multiply.
        game.applied = [
            delta.dx * cfg.gain * cfg.gain_x,
            delta.dy * cfg.gain * cfg.gain_y,
            delta.dz * cfg.gain * cfg.gain_z,
        ];
    }

    view.viewX = baseline.view_xyz[0] + game.applied[0];
    view.viewY = baseline.view_xyz[1] + game.applied[1];
    view.viewZ = baseline.view_xyz[2] + game.applied[2];

    // SAFETY: setter follows the FFI contract; we own `view`.
    unsafe { setter(&raw mut view) };
}

/// The rotation VPX's `SetViewPosFromPlayerPosition` applies between the
/// real-world player frame and the stored `viewX/Y/Z` (playfield-plane)
/// frame: `atan2(windowTopZOfs − windowBottomZOfs, table_length) −
/// screenInclination`. The table length isn't exposed, but
/// `GetRealToVirtualScale` makes `CMTOVPU(realToVirtualScale ·
/// screenWidth_cm)` the HYPOTENUSE `√(L² + ΔZ²)`, so the slope is
/// `asin(ΔZ / hyp)`. Degenerate host data (unset screen size or scale)
/// falls back to the physical inclination alone.
fn window_player_rotation(view: &VPXViewSetupDef) -> f32 {
    if ViewMode::from_i32(view.viewMode) != ViewMode::Window {
        return 0.0;
    }
    let dz = view.windowTopZOfs - view.windowBottomZOfs;
    let hyp = crate::camera::units::mm_to_vpu(view.realToVirtualScale * view.screenWidth * 10.0);
    let slope = if hyp > 1.0 && hyp.abs() > dz.abs() {
        (dz / hyp).asin()
    } else {
        0.0
    };
    slope - view.screenInclination.to_radians()
}

/// Build the one-shot startup notification: the camera the plugin tracks
/// on and the VPX settings the head-tracking experience depends on. The
/// camera-pose line arrives later, once the live anchor calibration lands.
/// The player-facing wording for a [`TrackingFault`].
///
/// Every sentence a player reads is built in this module, not in the backend
/// that detected the problem — a backend returns a cause, and the cause is
/// worded here. That is the seam a translation catalogue plugs into later:
/// one `match` per language rather than prose scattered across the trackers.
fn fault_note(fault: TrackingFault) -> Option<std::ffi::CString> {
    let text = match fault {
        TrackingFault::IrStreamBusy => {
            "Head tracking off: the Kinect's infrared stream cannot be opened because its \
             camera stream is already in use. Close whatever else is holding the Kinect \
             (the head-tracking demo, a capture tool) and restart the table."
        }
    };
    std::ffi::CString::new(text).ok()
}

fn startup_note(device: &str, _backend: &str, _cfg: &config::Config) -> Option<std::ffi::CString> {
    let text = format!(
        "Head tracking: {device} active\
         \nCheck your VPX setup: lockbar width + screen inclination \
         (Cabinet settings), POV layout 'Window' with rotation 0, \
         cabinet autofit enabled",
    );
    std::ffi::CString::new(text).ok()
}

/// Camera pose derived from the LIVE anchor calibration + the host's
/// lockbar width, formatted for the VPX notification.
#[cfg(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam"))]
fn camera_pose_note(
    backend: &str,
    calib: &crate::tracker::session::CameraCalibration,
    screen_incl_deg: f32,
) -> Option<std::ffi::CString> {
    let cfg = config::current();
    let pref_path = STATE.lock().as_ref()?.pref_path.clone();
    let lockbar_mm = pref_path
        .as_deref()
        .map(crate::plugin::host_settings::read_cab_geometry)
        .unwrap_or_default()
        .lockbar_width_mm;
    // Real factory intrinsics when the device provides them (Kinect v2);
    // demo-validated nominals otherwise (±0-3 % against tape measures).
    let [fx, fy, cx, cy] = calib.calibration_intrinsics.unwrap_or_else(|| {
        let fx = if backend.starts_with("kinect-v1") {
            525.0
        } else if cfg.webcam_focal_px > 0.0 {
            cfg.webcam_focal_px
        } else {
            calib.frame_w as f32 * crate::tracker::pipeline::WEBCAM_FX_PER_WIDTH
        };
        [
            fx,
            fx,
            calib.frame_w as f32 * 0.5,
            calib.frame_h as f32 * 0.5,
        ]
    });
    let intr = anchor::CameraIntrinsics { fx, fy, cx, cy };
    let lockbar = if lockbar_mm > 0.0 {
        lockbar_mm
    } else {
        crate::calibration::LOCKBAR_WIDTH_MM
    };
    let pose = anchor::camera_pose(&calib.geometry, &intr, lockbar)?;
    // The log gets every field with its own name, so a support thread can
    // read the pose without the reader having to know the struct.
    let fields = pose
        .report()
        .into_iter()
        .map(|f| format!("{}={}", f.label, f.value))
        .collect::<Vec<_>>()
        .join(" ");
    info!(
        %fields,
        lockbar_mm = lockbar,
        screen_incl_deg,
        fx,
        score = calib.score,
        "live anchor calibration: camera pose"
    );
    // Same sentence the demo prints under its table — one wording, one place
    // to be wrong. A webcam's focal is a nominal guess, so its distances are
    // marked approximate.
    let approx = if backend.starts_with("webcam") {
        "approx. "
    } else {
        ""
    };
    let mut text = format!(
        "Head tracking: {approx}{}\nFrom VPX: lockbar {:.1} cm, playfield incline {:.1}\u{b0}",
        pose.describe(),
        lockbar / 10.0,
        screen_incl_deg,
    );
    // Out of square means the rails and the lockbar did not come back
    // perpendicular, which only two things can cause: a focal length that does
    // not belong to the frame the outline was found in, or an outline that
    // does not follow the real cabinet. Explicitly NOT the lockbar width —
    // that scales the distances and cannot bend the shape. Worth interrupting
    // for, because nothing else in the game will ever say so.
    if (pose.rect_angle_deg - 90.0).abs() > 3.0 {
        // A Kinect reports its own focal, so on those there is only one
        // suspect left. A webcam's is a nominal guess and stays one.
        let suspects = if calib.calibration_intrinsics.is_some() {
            "the outline it found does not follow the real rails"
        } else {
            "either the outline it found does not follow the real rails, or \
             this camera's focal length is not the assumed one"
        };
        text.push_str(&format!(
            "\nThe cabinet rebuilds at {:.0}\u{b0} instead of 90\u{b0}, so these figures are \
             off: {suspects}. Recalibrating, or sharing a capture from the demo, is what \
             fixes it — the lockbar width is not the cause.",
            pose.rect_angle_deg,
        ));
    }
    std::ffi::CString::new(text).ok()
}

#[cfg(not(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam")))]
fn camera_pose_note(
    _backend: &str,
    _calib: &crate::tracker::session::CameraCalibration,
    _screen_incl_deg: f32,
) -> Option<std::ffi::CString> {
    None
}

fn init_tracing_once() {
    use std::sync::OnceLock;
    static GUARD: OnceLock<()> = OnceLock::new();
    GUARD.get_or_init(|| {
        use tracing_subscriber::layer::SubscriberExt;
        use tracing_subscriber::util::SubscriberInitExt;

        // Two layers stacked on the registry:
        //   * `fmt` to stderr — kept for headless dev (`headtracking-demo`)
        //     and as a fallback when VPX's LoggingPluginAPI isn't reachable
        //     (no host listener, plugin loaded outside VPX, etc.)
        //   * `VpxLogLayer` — forwards every event into VPX's console
        //     once `super::logging::resolve_and_install` has run.
        // Order doesn't matter: each layer decides independently whether
        // to emit. The env filter applies globally.
        let filter = tracing_subscriber::EnvFilter::try_from_env("HEADTRACKING_LOG")
            .unwrap_or_else(|_| {
                tracing_subscriber::EnvFilter::new(super::logging::DEFAULT_LOG_FILTER)
            });
        let fmt_layer = tracing_subscriber::fmt::layer().with_target(false);
        let _ = tracing_subscriber::registry()
            .with(filter)
            .with(fmt_layer)
            .with(super::logging::VpxLogLayer)
            .try_init();
    });
}

#[derive(Debug, thiserror::Error)]
enum LoadError {
    #[error("MsgPluginAPI is missing required function: {0}")]
    MissingFunction(&'static str),
    #[error("host did not expose VPXPluginAPI (no responder for VPX/GetAPI)")]
    NoVpxApi,
}

#[cfg(test)]
mod tests {
    use super::{TrackingFault, fault_note};

    /// Every cause must have wording a player can act on. `fault_note`
    /// matches exhaustively, so a new variant cannot compile without one —
    /// but it can compile with an empty or useless one, and a blank
    /// notification reads as "nothing happened" rather than "tracking is off".
    #[test]
    fn every_tracking_fault_has_something_to_say() {
        // One variant today. The loop is the point: it is the list a new
        // `TrackingFault` has to join, and the assertion below is what stops
        // it shipping without wording.
        #[allow(clippy::single_element_loop, reason = "grows with the enum")]
        for fault in [TrackingFault::IrStreamBusy] {
            let note = fault_note(fault).unwrap_or_else(|| panic!("no wording for {fault:?}"));
            let text = note.to_str().expect("utf-8");
            assert!(text.len() > 40, "{fault:?}: {text:?}");
            assert!(
                text.starts_with("Head tracking off"),
                "a fault note has to open by saying tracking stopped: {text:?}"
            );
        }
    }
}
