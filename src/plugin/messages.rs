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

// VPXPlugin.h — namespace & events we subscribe to.
pub const VPXPI_NAMESPACE: &CStr = cstr!("VPX");

// LoggingPlugin.h
// NB: upstream `#define LOGPI_NAMESPACE "Login"` — yes, the typo (Login,
// not Logging) is what VPX broadcasts on. Match it verbatim or our
// `BroadcastMsg` will never reach the host's logging endpoint.
pub const LOGPI_NAMESPACE: &CStr = cstr!("Login");

/// Dispatch-table version current 10.8.1 hosts write into `MsgPluginAPI`,
/// `VPXPluginAPI` and `LoggingPluginAPI`. A different value means the
/// function pointers are not where this build expects them.
pub const VPX_API_VERSION: i32 = 1;

/// Message names the plugin uses, for one revision of the host API.
/// `GetMsgID` keys on the literal, so a name from the wrong set never
/// reaches the host.
#[derive(Debug, Clone, Copy)]
pub struct MessageNames {
    pub vpx_get_api: &'static CStr,
    pub on_game_start: &'static CStr,
    pub on_game_end: &'static CStr,
    pub on_prepare_frame: &'static CStr,
    pub on_action_changed: &'static CStr,
    pub log_get_api: &'static CStr,
}

/// Since vpinball 595d1fd (2026-09-05, "Plugin: add version marker") every
/// message name ends in `:1` (`third_party/vpx-plugin-headers/`).
pub const NAMES_V1: MessageNames = MessageNames {
    vpx_get_api: cstr!("GetAPI:1"),
    on_game_start: cstr!("OnGameStart:1"),
    on_game_end: cstr!("OnGameEnd:1"),
    on_prepare_frame: cstr!("OnPrepareFrame:1"),
    on_action_changed: cstr!("OnActionChanged:1"),
    log_get_api: cstr!("GetAPI:1"),
};

/// Names spoken by older hosts, such as the 10.8.1-5436 pre-release of
/// 2026-08-21 (`third_party/vpx-plugin-headers-v0/`).
pub const NAMES_LEGACY: MessageNames = MessageNames {
    vpx_get_api: cstr!("GetAPI"),
    on_game_start: cstr!("OnGameStart"),
    on_game_end: cstr!("OnGameEnd"),
    on_prepare_frame: cstr!("OnPrepareFrame"),
    on_action_changed: cstr!("OnActionChanged"),
    log_get_api: cstr!("GetAPI"),
};

#[cfg(test)]
mod tests {
    use super::*;

    fn header(dir: &str, name: &str) -> String {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("third_party")
            .join(dir)
            .join(name);
        std::fs::read_to_string(&path)
            .unwrap_or_else(|err| panic!("read {}: {err}", path.display()))
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

    fn assert_names_match(dir: &str, names: &MessageNames) {
        let vpx = header(dir, "VPXPlugin.h");
        let log = header(dir, "LoggingPlugin.h");
        for (header, macro_name, rust_name) in [
            (&vpx, "VPXPI_NAMESPACE", VPXPI_NAMESPACE),
            (&vpx, "VPXPI_MSG_GET_API", names.vpx_get_api),
            (&vpx, "VPXPI_EVT_ON_GAME_START", names.on_game_start),
            (&vpx, "VPXPI_EVT_ON_GAME_END", names.on_game_end),
            (&vpx, "VPXPI_EVT_ON_PREPARE_FRAME", names.on_prepare_frame),
            (&vpx, "VPXPI_EVT_ON_ACTION_CHANGED", names.on_action_changed),
            (&log, "LOGPI_NAMESPACE", LOGPI_NAMESPACE),
            (&log, "LOGPI_MSG_GET_API", names.log_get_api),
        ] {
            assert_eq!(
                rust_name.to_str().unwrap(),
                defined_string(header, macro_name),
                "{dir}: {macro_name}"
            );
        }
    }

    #[test]
    fn v1_message_names_match_vendored_headers() {
        assert_names_match("vpx-plugin-headers", &NAMES_V1);
    }

    #[test]
    fn legacy_message_names_match_legacy_headers() {
        assert_names_match("vpx-plugin-headers-v0", &NAMES_LEGACY);
    }

    #[test]
    fn vendored_dispatch_tables_are_api_v1() {
        for name in ["MsgPlugin.h", "VPXPlugin.h", "LoggingPlugin.h"] {
            assert!(
                header("vpx-plugin-headers", name).contains("int version; // Must be 1"),
                "{name} is not the versioned 10.8.1 dispatch table"
            );
        }
    }

    #[test]
    fn legacy_dispatch_tables_are_unversioned() {
        for name in ["MsgPlugin.h", "VPXPlugin.h", "LoggingPlugin.h"] {
            assert!(
                !header("vpx-plugin-headers-v0", name).contains("int version;"),
                "{name} in vpx-plugin-headers-v0 is not the unversioned table"
            );
        }
    }
}
