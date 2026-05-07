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

use super::messages::{
    VPXPI_EVT_ON_GAME_END, VPXPI_EVT_ON_GAME_START, VPXPI_EVT_ON_PREPARE_FRAME, VPXPI_MSG_GET_API,
    VPXPI_NAMESPACE,
};
use super::vpx_sys::{MsgPluginAPI, VPXPluginAPI, VPXViewSetupDef};
use crate::camera::mapping::pose_delta_to_view_delta;
use crate::config;
use crate::tracker::Pose;
use crate::tracker::session::TrackerSession;

/// Lifecycle state captured at `PluginLoad` and torn down at `PluginUnload`.
struct PluginState {
    msg_api: *const MsgPluginAPI,
    vpx_api: *mut VPXPluginAPI,
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
}

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
        info!(
            session_id,
            version = env!("CARGO_PKG_VERSION"),
            os = std::env::consts::OS,
            arch = std::env::consts::ARCH,
            family = std::env::consts::FAMILY,
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
    }

    // Register all `[Plugin.HeadTracking]` settings. The host calls our
    // Set callbacks immediately with the value parsed from VPinballX.ini
    // (or the declared default when the key is missing), so by the time
    // OnGameStart fires the config snapshot is already populated.
    // SAFETY: api_ptr is non-null and live for the plugin session.
    unsafe { config::register_settings(&*api_ptr, session_id) };

    *STATE.lock() = Some(PluginState {
        msg_api: api_ptr,
        vpx_api,
        endpoint_id: session_id,
        msg_ids,
    });
    Ok(())
}

unsafe fn do_unload() {
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
            release(state.msg_ids.on_game_start);
            release(state.msg_ids.on_game_end);
            release(state.msg_ids.on_prepare_frame);
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
            min_cutoff_hz = cfg.min_cutoff_hz,
            beta = cfg.beta,
            "spawning tracker session with current config"
        );
        match TrackerSession::spawn(&cfg) {
            Ok(tracker) => {
                let backend = tracker.backend_name();
                *GAME.lock() = Some(GameSession {
                    tracker,
                    baseline: None,
                });
                info!(backend, "tracker session active");
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

    let pose = game.tracker.latest_pose();
    if pose.confidence <= 0.0 {
        return;
    }

    // SAFETY: vpx_api_ptr was set non-null at load time and the host keeps
    // the API live through the plugin session.
    let vpx = unsafe { &*vpx_api_ptr };
    let Some(getter) = vpx.GetActiveViewSetup else {
        return;
    };
    let Some(setter) = vpx.SetActiveViewSetup else {
        return;
    };

    let mut view = VPXViewSetupDef::default();
    // SAFETY: `view` is a valid out-pointer; getter follows the FFI contract.
    unsafe { getter(&raw mut view) };

    let baseline = match game.baseline {
        Some(b) => b,
        None => {
            // First frame with a real pose: capture both anchors and skip
            // applying any delta — the table's authored POV stands.
            let new_baseline = Baseline {
                pose: *pose,
                view_xyz: [view.viewX, view.viewY, view.viewZ],
            };
            game.baseline = Some(new_baseline);
            info!(
                x = pose.position_mm[0],
                y = pose.position_mm[1],
                z = pose.position_mm[2],
                "tracker baseline captured"
            );
            return;
        }
    };

    // Read the live config every frame. The lock is uncontended (only the
    // VPX settings UI ever takes a write lock, and that's rare), so this
    // is essentially a memcpy.
    let cfg = config::current();

    // Apply the user's BaselineOffset before computing the delta so the
    // offset acts as a manual recenter of the neutral pose, not a
    // post-gain bias.
    let mut adjusted_baseline_pose = baseline.pose;
    adjusted_baseline_pose.position_mm[0] += cfg.baseline_offset_x_mm;
    adjusted_baseline_pose.position_mm[1] += cfg.baseline_offset_y_mm;
    adjusted_baseline_pose.position_mm[2] += cfg.baseline_offset_z_mm;

    let delta = pose_delta_to_view_delta(&pose, &adjusted_baseline_pose);
    view.viewX = baseline.view_xyz[0] + delta.dx * cfg.gain;
    view.viewY = baseline.view_xyz[1] + delta.dy * cfg.gain;
    view.viewZ = baseline.view_xyz[2] + delta.dz * cfg.gain;

    // SAFETY: setter follows the FFI contract; we own `view`.
    unsafe { setter(&raw mut view) };
}

fn init_tracing_once() {
    use std::sync::OnceLock;
    static GUARD: OnceLock<()> = OnceLock::new();
    GUARD.get_or_init(|| {
        // For now: route to stderr so VPX captures it in its console log.
        // Phase 2 will switch to the LoggingPlugin.h API once we resolve it.
        let _ = tracing_subscriber::fmt()
            .with_env_filter(
                tracing_subscriber::EnvFilter::try_from_env("HEADTRACKING_LOG")
                    .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
            )
            .with_target(false)
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
