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
//
// Since vpinball 595d1fd (2026-09-05, "Plugin: add version marker") every
// message name ends in `:1`. `GetMsgID` keys on the literal, so the old
// unversioned names never reach a current 10.8.1 host.
pub const VPXPI_NAMESPACE: &CStr = cstr!("VPX");
pub const VPXPI_MSG_GET_API: &CStr = cstr!("GetAPI:1");
pub const VPXPI_EVT_ON_GAME_START: &CStr = cstr!("OnGameStart:1");
pub const VPXPI_EVT_ON_GAME_END: &CStr = cstr!("OnGameEnd:1");
pub const VPXPI_EVT_ON_PREPARE_FRAME: &CStr = cstr!("OnPrepareFrame:1");
pub const VPXPI_EVT_ON_ACTION_CHANGED: &CStr = cstr!("OnActionChanged:1");

// LoggingPlugin.h
// NB: upstream `#define LOGPI_NAMESPACE "Login"` — yes, the typo (Login,
// not Logging) is what VPX broadcasts on. Match it verbatim or our
// `BroadcastMsg` will never reach the host's logging endpoint.
pub const LOGPI_NAMESPACE: &CStr = cstr!("Login");
pub const LOGPI_MSG_GET_API: &CStr = cstr!("GetAPI:1");

/// Dispatch-table version current 10.8.1 hosts write into `MsgPluginAPI`,
/// `VPXPluginAPI` and `LoggingPluginAPI`. A different value means the
/// function pointers are not where this build expects them.
pub const VPX_API_VERSION: i32 = 1;

#[cfg(test)]
mod tests {
    use super::*;

    fn header(name: &str) -> String {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("third_party/vpx-plugin-headers")
            .join(name);
        std::fs::read_to_string(&path).unwrap_or_else(|err| panic!("read {}: {err}", path.display()))
    }

    fn defined_string(header: &str, macro_name: &str) -> String {
        let needle = format!("#define {macro_name}");
        let line = header
            .lines()
            .find(|line| line.contains(&needle))
            .unwrap_or_else(|| panic!("vendored headers have no {macro_name}"));
        line.split('"')
            .nth(1)
            .unwrap_or_else(|| panic!("{macro_name} is not a string macro: {line}"))
            .to_string()
    }

    #[test]
    fn rust_message_names_match_vendored_headers() {
        let vpx = header("VPXPlugin.h");
        let log = header("LoggingPlugin.h");
        for (header, macro_name, rust_name) in [
            (&vpx, "VPXPI_MSG_GET_API", VPXPI_MSG_GET_API),
            (&vpx, "VPXPI_EVT_ON_GAME_START", VPXPI_EVT_ON_GAME_START),
            (&vpx, "VPXPI_EVT_ON_GAME_END", VPXPI_EVT_ON_GAME_END),
            (
                &vpx,
                "VPXPI_EVT_ON_PREPARE_FRAME",
                VPXPI_EVT_ON_PREPARE_FRAME,
            ),
            (
                &vpx,
                "VPXPI_EVT_ON_ACTION_CHANGED",
                VPXPI_EVT_ON_ACTION_CHANGED,
            ),
            (&log, "LOGPI_MSG_GET_API", LOGPI_MSG_GET_API),
        ] {
            assert_eq!(
                rust_name.to_str().unwrap(),
                defined_string(header, macro_name),
                "{macro_name}"
            );
        }
    }

    #[test]
    fn vendored_dispatch_tables_are_api_v1() {
        for name in ["MsgPlugin.h", "VPXPlugin.h", "LoggingPlugin.h"] {
            assert!(
                header(name).contains("int version; // Must be 1"),
                "{name} is not the versioned 10.8.1 dispatch table"
            );
        }
    }
}
