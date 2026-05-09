//! Static byte-strings for VPX message namespaces / event names.
//!
//! These mirror the `#define` constants in `MsgPlugin.h`, `VPXPlugin.h` and
//! `LoggingPlugin.h`. We keep them as Rust-side `&CStr` so they can be passed
//! directly to `MsgPluginAPI::GetMsgID` without per-call allocation.

use std::ffi::CStr;

macro_rules! cstr {
    ($s:literal) => {{
        // SAFETY: literal is null-terminated and contains no interior NULs.
        unsafe { CStr::from_bytes_with_nul_unchecked(concat!($s, "\0").as_bytes()) }
    }};
}

// MsgPlugin.h
pub const MSGPI_NAMESPACE: &CStr = cstr!("MsgPlugin");

// VPXPlugin.h — namespaces & events we subscribe to.
pub const VPXPI_NAMESPACE: &CStr = cstr!("VPX");
pub const VPXPI_MSG_GET_API: &CStr = cstr!("GetAPI");
pub const VPXPI_EVT_ON_GAME_START: &CStr = cstr!("OnGameStart");
pub const VPXPI_EVT_ON_GAME_END: &CStr = cstr!("OnGameEnd");
pub const VPXPI_EVT_ON_PREPARE_FRAME: &CStr = cstr!("OnPrepareFrame");

// LoggingPlugin.h
// NB: upstream `#define LOGPI_NAMESPACE "Login"` — yes, the typo (Login,
// not Logging) is what VPX broadcasts on. Match it verbatim or our
// `BroadcastMsg` will never reach the host's logging endpoint.
pub const LOGPI_NAMESPACE: &CStr = cstr!("Login");
pub const LOGPI_MSG_GET_API: &CStr = cstr!("GetAPI");
