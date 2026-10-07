//! Which revision of the VPX plugin API the host speaks.
//!
//! vpinball `595d1fd` (2026-09-05) changed the plugin ABI in one go: every
//! message name gained a `:1` suffix and `MsgPluginAPI`, `VPXPluginAPI` and
//! `LoggingPluginAPI` gained a leading `int version`. The 10.8.1-5436
//! pre-release (2026-08-21) and every earlier build still speak the old,
//! unversioned API. We support both: the rest of the plugin only ever sees
//! tables in the current layout, and legacy tables are copied into it here.
//!
//! # How the revision is detected
//!
//! Nothing outside the table tells the two apart. `PluginLoad` keeps the same
//! exported name and the same `(uint32_t, MsgPluginAPI*)` signature,
//! `plugin.cfg` carries no API field, and the only host query (`GetVpxInfo`)
//! sits behind the very table whose layout is in question. So the table has
//! to identify itself, through its first 32-bit word:
//!
//! - a current host writes `version = 1` there (`MsgPluginManager` and
//!   `VPXPluginAPIImpl` constructors);
//! - an older host has a function pointer there (`GetPluginEndpoint`,
//!   `GetVpxInfo` or `Log`), i.e. a code address inside the host image.
//!
//! The low 32 bits of a code address are never 1: that would be an odd
//! address exactly one byte past a 4 GiB boundary. aarch64 instructions are
//! 4-byte aligned, optimizing x86/x86-64 compilers align function entries to
//! 16, and on 32-bit hosts address 1 sits in the never-mapped null page. More
//! generally no 16-aligned address has low bits in `1..=15`, so those values
//! can only be a version number. 0 and `2..=15` are refused (a future table
//! revision we cannot read must not be called through), everything else is
//! the legacy layout.
//!
//! Only 4 bytes are read: on 64-bit the 4 bytes after `version` are padding
//! with unspecified content, so the whole pointer-sized word cannot be used.
//! All supported targets are little-endian, so those 4 bytes are the low half
//! of a legacy pointer.
//!
//! Once `MsgPluginAPI` has picked a revision, the `VPXPluginAPI` and
//! `LoggingPluginAPI` tables returned under that revision's message names
//! must sniff the same way, or the plugin refuses to go further.

use std::ffi::c_void;

use super::messages::{MessageNames, NAMES_LEGACY, NAMES_V1, VPX_API_VERSION};
use super::vpx_sys::{LoggingPluginAPI, MsgPluginAPI, VPXPluginAPI};
use super::vpx_sys_v0 as v0;

const _: () = assert!(
    cfg!(target_endian = "little"),
    "API detection reads the low half of a pointer"
);

/// Plugin API revision of the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HostApi {
    /// vpinball 595d1fd (2026-09-05) and later: `:1` names, `version == 1`.
    V1,
    /// Older hosts, including the 10.8.1-5436 pre-release: plain names, no
    /// version field.
    Legacy,
}

impl HostApi {
    pub fn names(self) -> &'static MessageNames {
        match self {
            HostApi::V1 => &NAMES_V1,
            HostApi::Legacy => &NAMES_LEGACY,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            HostApi::V1 => "versioned API v1 (VPX 10.8.1 since 2026-09-05)",
            HostApi::Legacy => "legacy unversioned API (VPX 10.8.1 before 2026-09-05)",
        }
    }
}

/// What the first 32-bit word of a dispatch table says.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sniff {
    Api(HostApi),
    /// A version number this build does not know.
    Unknown(i32),
}

/// Classify the first 32-bit word of a dispatch table (see module docs).
pub fn classify(first_word: i32) -> Sniff {
    match first_word {
        VPX_API_VERSION => Sniff::Api(HostApi::V1),
        0..=15 => Sniff::Unknown(first_word),
        _ => Sniff::Api(HostApi::Legacy),
    }
}

/// # Safety
/// `table` must point at a live host dispatch table (either layout: both
/// start with at least 4 readable, 4-aligned bytes).
pub unsafe fn sniff(table: *const c_void) -> Sniff {
    // SAFETY: per the contract above.
    classify(unsafe { table.cast::<i32>().read() })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum AdoptError {
    #[error("{table} carries unknown API version {version}")]
    UnknownVersion { table: &'static str, version: i32 },
    #[error("{table} is {found:?} but MsgPluginAPI is {expected:?}")]
    Mismatch {
        table: &'static str,
        expected: HostApi,
        found: HostApi,
    },
}

fn expect(
    table: &'static str,
    ptr: *const c_void,
    expected: Option<HostApi>,
) -> Result<HostApi, AdoptError> {
    // SAFETY: callers pass a non-null host table.
    match unsafe { sniff(ptr) } {
        Sniff::Unknown(version) => Err(AdoptError::UnknownVersion { table, version }),
        Sniff::Api(found) => match expected {
            Some(expected) if expected != found => Err(AdoptError::Mismatch {
                table,
                expected,
                found,
            }),
            _ => Ok(found),
        },
    }
}

// A legacy table is copied into a current-layout one that lives as long as
// the host pointers it holds: the plugin stores it like a host pointer. It
// is leaked (about 250 bytes per plugin load) because the logging bridge may
// still read it while the plugin unloads.
fn leak<T>(value: T) -> *mut T {
    Box::leak(Box::new(value))
}

/// Resolve the `MsgPluginAPI` handed to `PluginLoad`.
///
/// # Safety
/// `ptr` must be the non-null pointer the host passed to `PluginLoad`.
pub unsafe fn adopt_msg_api(
    ptr: *const MsgPluginAPI,
) -> Result<(HostApi, *const MsgPluginAPI), AdoptError> {
    match expect("MsgPluginAPI", ptr.cast(), None)? {
        HostApi::V1 => Ok((HostApi::V1, ptr)),
        HostApi::Legacy => {
            // SAFETY: the host table has the legacy layout.
            let old = unsafe { &*ptr.cast::<v0::MsgPluginAPI>() };
            Ok((HostApi::Legacy, leak(msg_api_from_legacy(old)).cast_const()))
        }
    }
}

/// Resolve the table returned by `VPX/GetAPI`.
///
/// # Safety
/// `ptr` must be the non-null pointer the host returned for `api`'s name.
pub unsafe fn adopt_vpx_api(
    api: HostApi,
    ptr: *mut c_void,
) -> Result<*mut VPXPluginAPI, AdoptError> {
    match expect("VPXPluginAPI", ptr, Some(api))? {
        HostApi::V1 => Ok(ptr.cast()),
        HostApi::Legacy => {
            // SAFETY: the host table has the legacy layout.
            let old = unsafe { &*ptr.cast::<v0::VPXPluginAPI>() };
            Ok(leak(vpx_api_from_legacy(old)))
        }
    }
}

/// Resolve the table returned by `Login/GetAPI`.
///
/// # Safety
/// `ptr` must be the non-null pointer the host returned for `api`'s name.
pub unsafe fn adopt_logging_api(
    api: HostApi,
    ptr: *mut c_void,
) -> Result<*const LoggingPluginAPI, AdoptError> {
    match expect("LoggingPluginAPI", ptr, Some(api))? {
        HostApi::V1 => Ok(ptr.cast_const().cast()),
        HostApi::Legacy => {
            // SAFETY: the host table has the legacy layout.
            let old = unsafe { &*ptr.cast::<v0::LoggingPluginAPI>() };
            Ok(leak(LoggingPluginAPI {
                version: VPX_API_VERSION,
                Log: old.Log,
            })
            .cast_const())
        }
    }
}

// The conversions list every field without `..Default::default()`: a field
// added to the current headers fails to compile here until it is handled.
// The field types are shared with `vpx_sys`, so a signature that differs
// between the two header sets fails to compile too.

fn msg_api_from_legacy(old: &v0::MsgPluginAPI) -> MsgPluginAPI {
    MsgPluginAPI {
        version: VPX_API_VERSION,
        GetPluginEndpoint: old.GetPluginEndpoint,
        GetEndpointInfo: old.GetEndpointInfo,
        GetMsgID: old.GetMsgID,
        SubscribeMsg: old.SubscribeMsg,
        UnsubscribeMsg: old.UnsubscribeMsg,
        BroadcastMsg: old.BroadcastMsg,
        SendMsg: old.SendMsg,
        ReleaseMsgID: old.ReleaseMsgID,
        RegisterSetting: old.RegisterSetting,
        SaveSetting: old.SaveSetting,
        RunOnMainThread: old.RunOnMainThread,
        FlushPendingCallbacks: old.FlushPendingCallbacks,
    }
}

fn vpx_api_from_legacy(old: &v0::VPXPluginAPI) -> VPXPluginAPI {
    VPXPluginAPI {
        version: VPX_API_VERSION,
        GetVpxInfo: old.GetVpxInfo,
        GetTableInfo: old.GetTableInfo,
        PushNotification: old.PushNotification,
        UpdateNotification: old.UpdateNotification,
        DisableStaticPrerendering: old.DisableStaticPrerendering,
        GetActiveViewSetup: old.GetActiveViewSetup,
        SetActiveViewSetup: old.SetActiveViewSetup,
        GetInputState: old.GetInputState,
        SetInputState: old.SetInputState,
        GetGameTime: old.GetGameTime,
        CreateTexture: old.CreateTexture,
        UpdateTexture: old.UpdateTexture,
        GetTextureInfo: old.GetTextureInfo,
        DeleteTexture: old.DeleteTexture,
        // Not in the 2026-08-20 headers (some later legacy builds append
        // it; we never call it).
        RunScript: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::{offset_of, size_of};

    #[test]
    fn classify_separates_versions_from_pointers() {
        assert_eq!(classify(1), Sniff::Api(HostApi::V1));
        for v in [0, 2, 3, 15] {
            assert_eq!(classify(v), Sniff::Unknown(v));
        }
        // Low halves of plausible code addresses, including ones with the
        // top bit set (negative as i32).
        for addr in [0x0040_1000_u32, 0x4012_3450, 0x8000_0010, 0xffff_fff0] {
            assert_eq!(
                classify(addr as i32),
                Sniff::Api(HostApi::Legacy),
                "{addr:#x}"
            );
        }
    }

    extern "C" fn dummy() {}

    /// Fill every pointer slot of `T` with a distinct non-null value.
    fn patterned<T: Default>() -> T {
        let mut value = T::default();
        let slots = size_of::<T>() / size_of::<usize>();
        let p = (&raw mut value).cast::<usize>();
        for i in 0..slots {
            // SAFETY: `T` is a table of function pointers; any non-null
            // value is a valid bit pattern (never called).
            unsafe { p.add(i).write((i + 1) << 4) };
        }
        value
    }

    fn bytes<T>(value: &T) -> &[u8] {
        // SAFETY: plain-data FFI struct.
        unsafe { std::slice::from_raw_parts((value as *const T).cast(), size_of::<T>()) }
    }

    #[test]
    fn sniff_reads_real_tables() {
        let v1 = MsgPluginAPI {
            version: 1,
            ..Default::default()
        };
        let legacy = v0::MsgPluginAPI {
            // SAFETY: a real code address; never called.
            GetPluginEndpoint: Some(unsafe {
                std::mem::transmute::<
                    extern "C" fn(),
                    unsafe extern "C" fn(*const std::ffi::c_char) -> u32,
                >(dummy)
            }),
            ..Default::default()
        };
        unsafe {
            assert_eq!(sniff((&raw const v1).cast()), Sniff::Api(HostApi::V1));
            assert_eq!(
                sniff((&raw const legacy).cast()),
                Sniff::Api(HostApi::Legacy)
            );
        }
    }

    /// Legacy layout == current layout minus the leading `version` slot
    /// (and minus the trailing `RunScript` for VPXPluginAPI). The bindgen
    /// layout asserts in `vpx_bindings_v0.rs` tie the legacy structs to the
    /// old headers; this ties them to the current ones.
    #[test]
    fn legacy_layouts_are_current_minus_version() {
        let m = offset_of!(MsgPluginAPI, GetPluginEndpoint);
        assert_eq!(m, size_of::<usize>(), "version slot is pointer-sized");
        assert_eq!(size_of::<v0::MsgPluginAPI>(), size_of::<MsgPluginAPI>() - m);
        assert_eq!(
            offset_of!(v0::MsgPluginAPI, GetMsgID),
            offset_of!(MsgPluginAPI, GetMsgID) - m
        );
        assert_eq!(
            offset_of!(v0::MsgPluginAPI, FlushPendingCallbacks),
            offset_of!(MsgPluginAPI, FlushPendingCallbacks) - m
        );

        let v = offset_of!(VPXPluginAPI, GetVpxInfo);
        assert_eq!(v, size_of::<usize>());
        assert_eq!(
            size_of::<v0::VPXPluginAPI>(),
            offset_of!(VPXPluginAPI, RunScript) - v
        );
        assert_eq!(
            offset_of!(v0::VPXPluginAPI, SetActiveViewSetup),
            offset_of!(VPXPluginAPI, SetActiveViewSetup) - v
        );

        let l = offset_of!(LoggingPluginAPI, Log);
        assert_eq!(l, size_of::<usize>());
        assert_eq!(
            size_of::<v0::LoggingPluginAPI>(),
            size_of::<LoggingPluginAPI>() - l
        );
    }

    /// Field names in declaration order, read from the derived `Debug`.
    fn field_names<T: std::fmt::Debug + Default>() -> Vec<String> {
        let dbg = format!("{:?}", T::default());
        let body = &dbg[dbg.find('{').unwrap() + 1..dbg.rfind('}').unwrap()];
        body.split(',')
            .filter_map(|f| f.split(':').next())
            .map(|f| f.trim().to_string())
            .filter(|f| !f.is_empty())
            .collect()
    }

    #[test]
    fn legacy_fields_are_current_fields_minus_version() {
        let mut msg = vec!["version".to_string()];
        msg.extend(field_names::<v0::MsgPluginAPI>());
        assert_eq!(field_names::<MsgPluginAPI>(), msg);

        let mut vpx = vec!["version".to_string()];
        vpx.extend(field_names::<v0::VPXPluginAPI>());
        vpx.push("RunScript".to_string());
        assert_eq!(field_names::<VPXPluginAPI>(), vpx);

        let mut log = vec!["version".to_string()];
        log.extend(field_names::<v0::LoggingPluginAPI>());
        assert_eq!(field_names::<LoggingPluginAPI>(), log);
    }

    /// Every slot lands where the current layout expects it: catches two
    /// same-signature fields swapped in the hand-written conversion.
    #[test]
    fn conversions_keep_every_pointer_in_place() {
        let old = patterned::<v0::MsgPluginAPI>();
        let new = msg_api_from_legacy(&old);
        assert_eq!(new.version, 1);
        let m = offset_of!(MsgPluginAPI, GetPluginEndpoint);
        assert_eq!(&bytes(&new)[m..], bytes(&old));

        let old = patterned::<v0::VPXPluginAPI>();
        let new = vpx_api_from_legacy(&old);
        assert_eq!(new.version, 1);
        assert!(new.RunScript.is_none());
        let v = offset_of!(VPXPluginAPI, GetVpxInfo);
        let end = offset_of!(VPXPluginAPI, RunScript);
        assert_eq!(&bytes(&new)[v..end], bytes(&old));
    }

    #[test]
    fn adopt_converts_legacy_and_keeps_v1() {
        let old = patterned::<v0::MsgPluginAPI>();
        let (api, ptr) = unsafe { adopt_msg_api((&raw const old).cast()) }.unwrap();
        assert_eq!(api, HostApi::Legacy);
        assert_eq!(unsafe { (*ptr).version }, 1);
        assert_eq!(
            unsafe { (*ptr).GetMsgID }.map(|f| f as usize),
            old.GetMsgID.map(|f| f as usize)
        );

        let v1 = MsgPluginAPI {
            version: 1,
            ..Default::default()
        };
        let (api, ptr) = unsafe { adopt_msg_api(&raw const v1) }.unwrap();
        assert_eq!(api, HostApi::V1);
        assert_eq!(ptr, &raw const v1);

        // A legacy VPXPluginAPI under v1 names (or the reverse) is refused.
        let mut vpx_old = patterned::<v0::VPXPluginAPI>();
        let err = unsafe { adopt_vpx_api(HostApi::V1, (&raw mut vpx_old).cast()) }.unwrap_err();
        assert!(matches!(err, AdoptError::Mismatch { .. }));
        let mut vpx_v2 = VPXPluginAPI {
            version: 2,
            ..Default::default()
        };
        let err = unsafe { adopt_vpx_api(HostApi::V1, (&raw mut vpx_v2).cast()) }.unwrap_err();
        assert_eq!(
            err,
            AdoptError::UnknownVersion {
                table: "VPXPluginAPI",
                version: 2
            }
        );

        let mut log_old = patterned::<v0::LoggingPluginAPI>();
        let log = unsafe { adopt_logging_api(HostApi::Legacy, (&raw mut log_old).cast()) }.unwrap();
        assert_eq!(unsafe { (*log).version }, 1);
    }
}
