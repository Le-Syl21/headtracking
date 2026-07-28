//! Bridge from `tracing` to VPX's `LoggingPluginAPI`.
//!
//! At PluginLoad we broadcast `Login/GetAPI` (the upstream typo —
//! `LOGPI_NAMESPACE = "Login"` per `LoggingPlugin.h:19`) and store the
//! returned `LoggingPluginAPI*` in an `AtomicPtr`. Every `tracing`
//! event we emit then runs through [`VpxLogLayer`], which reads the
//! pointer, formats the event into a UTF-8 C string and calls
//! `api.Log(source, func, line, level, msg)`.
//!
//! The bridge is **purely additive**: the existing `tracing_subscriber::fmt`
//! layer to stderr stays installed, so logs still surface for headless
//! debugging (`HEADTRACKING_LOG=debug headtracking-demo`) and we don't
//! lose anything if VPX's logging API is unavailable.
//!
//! Threading: VPX's general API contract is "main-thread only", but
//! the logging callback is plain `Log(source, func, line, level, message)` with no shared
//! state we touch — every concrete `LoggingPlugin` impl in upstream
//! VPX writes to a Mutex-guarded buffer. We treat it as MT-safe and
//! call it from whichever thread emits the tracing event (tracker
//! worker, frame callback, etc.). If this turns out to be wrong in
//! practice, the fix is to route through `MsgPluginAPI::RunOnMainThread`
//! — but that would gate every log call behind a deferred call and is
//! a heavy hammer for a debug aid.

use std::ffi::{CStr, CString};
use std::fmt::Write;
use std::ptr;
use std::sync::atomic::{AtomicPtr, Ordering};

use tracing::{Event, Subscriber};
use tracing_subscriber::layer::{Context, Layer};
use tracing_subscriber::registry::LookupSpan;

use crate::plugin::vpx_sys::LoggingPluginAPI;

/// Plugin identity handed to VPX as the log line's `source` field.
const LOG_SOURCE: &CStr = c"HeadTracking";

// LPI_LVL_* — verbatim from `LoggingPlugin.h`.
const LPI_LVL_DEBUG: u32 = 0x00;
const LPI_LVL_INFO: u32 = 0x10;
const LPI_LVL_WARN: u32 = 0x20;
const LPI_LVL_ERROR: u32 = 0x40;

static LOGGING_API: AtomicPtr<LoggingPluginAPI> = AtomicPtr::new(ptr::null_mut());

/// Register a resolved `LoggingPluginAPI` so subsequent tracing events
/// get mirrored into VPX's console. Idempotent and overwrite-on-set —
/// if a previous PluginLoad left a pointer behind, the new one wins.
pub fn install(api: *const LoggingPluginAPI) {
    LOGGING_API.store(api as *mut _, Ordering::Release);
}

/// Drop the registered API. Called from PluginUnload before the host
/// reclaims the API memory.
pub fn clear() {
    LOGGING_API.store(ptr::null_mut(), Ordering::Release);
}

/// `tracing` layer that forwards each event to VPX's logging API.
pub struct VpxLogLayer;

impl<S> Layer<S> for VpxLogLayer
where
    S: Subscriber + for<'a> LookupSpan<'a>,
{
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        let api_ptr = LOGGING_API.load(Ordering::Acquire);
        if api_ptr.is_null() {
            return;
        }
        // SAFETY: the pointer is live between `install` and `clear`;
        // both calls happen on the main thread inside the plugin
        // session lock, so by the time we observe a non-null pointer
        // the API struct is fully initialised. `LoggingPluginAPI` is
        // `#[repr(C)] Copy`, no Drop, no interior mutability.
        let log_fn = match unsafe { (*api_ptr).Log } {
            Some(f) => f,
            None => return,
        };

        let meta = event.metadata();
        let level = level_to_lpi(*meta.level());

        let mut buf = String::with_capacity(128);
        let mut visitor = MessageVisitor(&mut buf);
        event.record(&mut visitor);

        // CString::new rejects interior NULs; substitute defensively.
        // Tracing fields can in principle carry arbitrary bytes via
        // `record_debug`.
        if buf.contains('\0') {
            buf = buf.replace('\0', " ");
        }
        let Ok(message) = CString::new(buf) else {
            return;
        };

        // VPX ≥ 2026-06 tags each line with source/func/line itself, so we
        // hand the tracing target over as `func` (its module path — the
        // closest thing to a call site we have here) rather than prefixing
        // the message text.
        let func = CString::new(meta.target()).unwrap_or_default();
        let line = meta.line().unwrap_or(0) as i32;

        // SAFETY: contract per LoggingPlugin.h —
        // Log(source, func, line, level, NUL-terminated UTF-8 message).
        unsafe {
            log_fn(
                LOG_SOURCE.as_ptr(),
                func.as_ptr(),
                line,
                level,
                message.as_ptr(),
            );
        }
    }
}

fn level_to_lpi(level: tracing::Level) -> u32 {
    if level == tracing::Level::ERROR {
        LPI_LVL_ERROR
    } else if level == tracing::Level::WARN {
        LPI_LVL_WARN
    } else if level == tracing::Level::INFO {
        LPI_LVL_INFO
    } else {
        // DEBUG + TRACE both map to DEBUG — VPX's API has no TRACE.
        LPI_LVL_DEBUG
    }
}

struct MessageVisitor<'a>(&'a mut String);

impl tracing::field::Visit for MessageVisitor<'_> {
    fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
        if field.name() == "message" {
            self.0.push_str(value);
        } else {
            let _ = write!(self.0, " {}={value}", field.name());
        }
    }
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            let _ = write!(self.0, "{value:?}");
        } else {
            let _ = write!(self.0, " {}={value:?}", field.name());
        }
    }
}

/// Resolve `Login/GetAPI` against the host and stash the returned
/// pointer for [`VpxLogLayer`] to consume. Failure (no host listener,
/// null reply) is non-fatal: tracing keeps working via the stderr
/// layer.
///
/// # Safety
/// `msg_api` must point at a live `MsgPluginAPI` for the duration of
/// the call, and `endpoint_id` must be the plugin's session id.
pub unsafe fn resolve_and_install(
    msg_api: &crate::plugin::vpx_sys::MsgPluginAPI,
    endpoint_id: u32,
) {
    let Some(get_msg_id) = msg_api.GetMsgID else {
        return;
    };
    let Some(broadcast) = msg_api.BroadcastMsg else {
        return;
    };
    // SAFETY: hand-written &CStr literals (see messages.rs); valid C strings.
    let id = unsafe {
        get_msg_id(
            crate::plugin::messages::LOGPI_NAMESPACE.as_ptr(),
            crate::plugin::messages::LOGPI_MSG_GET_API.as_ptr(),
        )
    };
    let mut api: *mut LoggingPluginAPI = ptr::null_mut();
    // SAFETY: BroadcastMsg writes the API pointer into our out-param
    // if a host endpoint is registered on Login/GetAPI; otherwise the
    // pointer stays null.
    unsafe {
        broadcast(endpoint_id, id, (&raw mut api).cast());
    }
    if !api.is_null() {
        install(api);
    }
    // We don't ReleaseMsgID for getLoggingApiId here: the upstream
    // pattern (LPI_IMPLEMENT_CPP) does it, but the id is cheap and
    // releasing it doesn't unsubscribe the host endpoint, so it's
    // optional. Skip for code-size simplicity.
}
