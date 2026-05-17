//! `headtracking-demo`: standalone validation harness for the head-tracker
//! pipeline.
//!
//! Detects connected Kinect v1 / v2 sensors and webcams, exposes a dropdown
//! to pick the active input. The centre pane shows the live RGB feed with
//! the YuNet face bbox overlaid; the bottom panel splits into a tracing
//! log on the left and the VPX-style view delta on the right.
//!
//! Run with `cargo run --release -p headtracking-demo`.

// Suppress the inherited Windows console for release builds — tracing
// writes to `headtracking-demo.log` next to the binary AND to the
// in-app log panel, so the console window adds nothing and only
// confuses end users. Debug builds keep stderr → console for the dev
// loop (`cargo run`).
#![cfg_attr(
    all(target_os = "windows", not(debug_assertions)),
    windows_subsystem = "windows"
)]

use std::collections::VecDeque;
use std::io::{self, IsTerminal as _, Write};
use std::sync::Arc;
use std::time::{Duration, Instant};

use eframe::egui::{
    self, Align, CentralPanel, Color32, ColorImage, ComboBox, Layout, Pos2, Rect, RichText,
    ScrollArea, Sense, Stroke, TextureHandle, TopBottomPanel, Vec2,
};
use parking_lot::Mutex;
use tracing::{error, info, warn};
use tracing_subscriber::fmt::MakeWriter;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;

const DEPTH_MIN_MM: f32 = 500.0;
const DEPTH_MAX_MM: f32 = 2_500.0;

const LOG_BUFFER_LINES: usize = 1_000;

fn main() -> eframe::Result {
    let logs: Arc<Mutex<VecDeque<String>>> =
        Arc::new(Mutex::new(VecDeque::with_capacity(LOG_BUFFER_LINES)));
    init_tracing(Arc::clone(&logs));

    // Banner: lets a beta tester paste a single line that pins the
    // build they're running when something misbehaves.
    let host = os_info::get();
    info!(
        version = env!("CARGO_PKG_VERSION"),
        os = %host.os_type(),
        os_version = %host.version(),
        arch = host.architecture().unwrap_or("unknown"),
        "headtracking-demo starting"
    );

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 800.0])
            .with_title("headtracking-demo"),
        ..Default::default()
    };

    eframe::run_native(
        "headtracking-demo",
        options,
        Box::new(move |_cc| Ok(Box::new(App::new(logs)))),
    )
}

// ============================================================ Backend dropdown

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Backend {
    None,
    KinectV1,
    KinectV2,
    /// Index in the enumerated webcam list.
    Webcam(u32),
}

#[derive(Debug, Clone)]
struct BackendEntry {
    backend: Backend,
    label: String,
}

/// Probe USB for connected sensors. Always returns `None (off)` first; the
/// other entries are added when the corresponding library reports a device.
///
/// Logs each backend's enumeration outcome at INFO level. For triage of
/// the "Kinect v1 listed in Device Manager but won't open" case on
/// Windows:
///   * set `FREENECT_LOG_LEVEL=spew` (or `flood`) before launching to
///     make libfreenect itself emit its full USB transcript;
///   * set `HEADTRACKING_LOG=libfreenect=debug,info` so the demo
///     surfaces those lines (they'll appear with the `libfreenect:`
///     prefix in both the stderr stream and the in-app log panel).
fn detect_backends() -> Vec<BackendEntry> {
    info!("scan: probing USB backends");
    let mut out = vec![BackendEntry {
        backend: Backend::None,
        label: "None (off)".to_string(),
    }];

    // ---- Kinect v2 (libfreenect2)
    match freenect2::Context::new() {
        Ok(ctx) => {
            let n = ctx.enumerate();
            info!(count = n, "scan: libfreenect2 enumerated devices");
            if n > 0 {
                out.push(BackendEntry {
                    backend: Backend::KinectV2,
                    label: "Kinect v2".to_string(),
                });
            }
        }
        Err(e) => warn!(?e, "scan: kinect v2 context init failed"),
    }

    // ---- Kinect v1 (libfreenect)
    match freenect::Context::new() {
        Ok(ctx) => {
            let n = ctx.enumerate();
            // Distinguish two states explicitly:
            //   n == 0  : libusb saw nothing matching VID 045E:02ae/02bf
            //             → no driver bound, or driver bound by another
            //             stack libusb can't drive. On Windows the
            //             bundled `setup\setup.ps1` installs the WinUSB
            //             INFs that fix this — the demo offers a
            //             one-click banner that launches it elevated.
            //   n  > 0  : libusb saw the device descriptor and can
            //             open it.
            if n > 0 {
                info!(count = n, "scan: kinect v1 detected");
                out.push(BackendEntry {
                    backend: Backend::KinectV1,
                    label: "Kinect v1".to_string(),
                });
            } else {
                info!(
                    "scan: kinect v1 — libfreenect counted 0 devices. \
                     On Windows that usually means the WinUSB driver \
                     hasn't been bound to the three Xbox NUI sub-devices \
                     yet — click the in-app 'Install Kinect drivers' \
                     banner button (or run setup\\setup.ps1 elevated). \
                     Check the libfreenect log lines just above (set \
                     FREENECT_LOG_LEVEL=spew for full USB trace)."
                );
            }
        }
        Err(e) => warn!(?e, "scan: kinect v1 context init failed"),
    }

    // ---- Webcam via SDL3
    match webcam::list() {
        Ok(cams) => {
            info!(count = cams.len(), "scan: SDL3 enumerated cameras");
            for cam in cams {
                let label = if cam.name.is_empty() {
                    format!("Webcam #{}", cam.id)
                } else {
                    format!("Webcam: {}", cam.name)
                };
                info!(index = cam.id, name = %cam.name, "scan: webcam entry");
                out.push(BackendEntry {
                    backend: Backend::Webcam(cam.id),
                    label,
                });
            }
        }
        Err(e) => warn!(?e, "scan: webcam enumerate failed"),
    }

    info!(entries = out.len() - 1, "scan: complete");
    out
}

// ==================================================== Kinect access helper
//
// A Kinect only shows up in `detect_backends` if libusb can actually open
// it, and that has an OS-level prerequisite:
//   * Linux  — a udev rule giving each Kinect USB node 0666 (libfreenect
//              ships `66-kinect.rules` for v1 PIDs, libfreenect2 ships
//              `90-kinect2.rules` for v2 PIDs); without them
//              `freenect{,2}::Context::enumerate()` probes each candidate,
//              hits `LIBUSB_ERROR_ACCESS`, and silently drops it.
//   * Windows — a libusb-capable kernel driver (WinUSB) bound to the
//              Kinect interfaces; a fresh Windows binds nothing, the
//              sensor sits in Device Manager as "Other device".
//   * macOS  — nothing; libusb opens the device as-is.
// So: if a Kinect is on the USB bus but absent from the dropdown, offer a
// one-click fix (drop the udev rule via pkexec/sudo on Linux; spawn the
// bundled `setup\setup.ps1` WinUSB installer elevated via a hidden
// PowerShell trampoline that calls `Start-Process -Verb RunAs` on Windows).

// Banner copy — picked at compile time so the message names the real fix.
#[cfg(target_os = "linux")]
const KINECT_ACCESS_PROBLEM: &str =
    "— libfreenect / libfreenect2 need a udev rule (0666) to open the sensor.";
#[cfg(target_os = "linux")]
const KINECT_ACCESS_BUTTON: &str = "Install udev rule (asks for password)";
#[cfg(target_os = "linux")]
const KINECT_ACCESS_OK_NOTE: &str = "udev rule installed. Click 'rescan' — if the sensor still doesn't show up, unplug/replug it first.";

#[cfg(target_os = "windows")]
const KINECT_ACCESS_PROBLEM: &str = "— Windows binds no usable driver out of the box; libusb/libfreenect can't see it until WinUSB is installed on the Kinect interfaces.";
#[cfg(target_os = "windows")]
const KINECT_ACCESS_BUTTON: &str = "Install Kinect drivers (UAC prompt)";
#[cfg(target_os = "windows")]
const KINECT_ACCESS_OK_NOTE: &str = "driver installer launched — confirm the UAC prompt, let it finish (~10-30 s), then click 'rescan'. v2 also needs a dedicated USB 3.0 root port.";

#[cfg(not(any(target_os = "linux", target_os = "windows")))]
const KINECT_ACCESS_PROBLEM: &str = "";
#[cfg(not(any(target_os = "linux", target_os = "windows")))]
const KINECT_ACCESS_BUTTON: &str = "";
#[cfg(not(any(target_os = "linux", target_os = "windows")))]
const KINECT_ACCESS_OK_NOTE: &str = "";

/// `true` when at least one Kinect on the USB bus lacks its OS access
/// prerequisite (Linux udev rule / Windows WinUSB binding), so the fix
/// banner is worth showing. Always `false` on macOS.
///
/// We deliberately *don't* short-circuit on "a Kinect is already in the
/// dropdown" — on Linux, libfreenect's v1 enumeration (`freenect_num_devices`)
/// doesn't need device access, so a v1 with no udev rule still shows up in
/// the dropdown but errors out at open time with EACCES. The presence-in-
/// dropdown signal is therefore unreliable; `kinect_present_but_not_set_up`
/// is the only authoritative source.
fn compute_kinect_access_hint() -> bool {
    if !kinect_present_but_not_set_up() {
        return false;
    }
    warn!(
        "a Kinect is on the USB bus but not accessible — the demo offers a one-click fix \
         ({})",
        if cfg!(target_os = "windows") {
            "WinUSB driver install"
        } else {
            "udev rule install"
        }
    );
    true
}

/// Linux: any Kinect v1 (Xbox 360 model 1414: `02ae`/`02ad`/`02b0`;
/// Kinect-for-Windows model 1473: `02c2`/`02be`/`02bf`) or Kinect v2
/// (`02c4`/`02d8`/`02d9`) USB device is plugged in (read from sysfs without
/// privileges) and at least one of the present PIDs isn't covered by any
/// udev rule under the standard rules directories. Returns true if so —
/// the banner needs to fire.
#[cfg(target_os = "linux")]
fn kinect_present_but_not_set_up() -> bool {
    use std::collections::HashSet;

    // PIDs libfreenect / libfreenect2 need 0666 on to open over libusb.
    const KINECT_PIDS: &[&str] = &[
        "02ae", "02ad", "02b0", // v1 Xbox 360 (1414): camera, audio, motor
        "02c2", "02be", "02bf", // v1 Kinect for Windows (1473): camera, audio, motor
        "02c4", "02d8", "02d9", // v2: sensor, firmware-update, adapter hub
    ];

    let present: HashSet<String> = std::fs::read_dir("/sys/bus/usb/devices")
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|entry| {
            let dir = entry.path();
            let id = |name: &str| {
                std::fs::read_to_string(dir.join(name))
                    .ok()
                    .map(|s| s.trim().to_ascii_lowercase())
            };
            if id("idVendor").as_deref() != Some("045e") {
                return None;
            }
            let pid = id("idProduct")?;
            KINECT_PIDS.contains(&pid.as_str()).then_some(pid)
        })
        .collect();
    if present.is_empty() {
        return false;
    }

    const RULES_DIRS: &[&str] = &[
        "/etc/udev/rules.d",
        "/run/udev/rules.d",
        "/usr/local/lib/udev/rules.d",
        "/usr/lib/udev/rules.d",
        "/lib/udev/rules.d",
    ];
    // A PID is "covered" iff some `.rules` file mentions BOTH `045e` and
    // that PID — same per-file conjunction the v2-only check used,
    // generalised to the union of detected PIDs.
    let mut covered: HashSet<String> = HashSet::new();
    for path in RULES_DIRS
        .iter()
        .flat_map(std::fs::read_dir)
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("rules"))
    {
        let Ok(txt) = std::fs::read_to_string(&path) else {
            continue;
        };
        let lower = txt.to_ascii_lowercase();
        if !lower.contains("045e") {
            continue;
        }
        for pid in &present {
            if lower.contains(pid.as_str()) {
                covered.insert(pid.clone());
            }
        }
    }
    // Banner fires if at least one present Kinect PID has no rule.
    !present.is_subset(&covered)
}

/// Windows: a Kinect (v1 sub-device or v2 sensor) is present on the USB
/// bus but no libusb-capable driver (WinUSB / libusbK) is bound to any of
/// its interfaces. Queried via PowerShell's `Get-PnpDevice` — no extra
/// crate, and it works on stock Windows 8.1+.
#[cfg(target_os = "windows")]
fn kinect_present_but_not_set_up() -> bool {
    use std::os::windows::process::CommandExt;
    use std::process::Command;

    // PID lists: v1 = Camera/Audio/Motor across models 1414 & 1473/KfW;
    // v2 = sensor 02C4, firmware-update 02D8, NuiSensor Adaptor 02D9.
    const SCRIPT: &str = "\
        $ms = 'VID_045E&PID_(02AE|02BF|02AD|02BE|02B0|02C2|02C4|02D8|02D9)'; \
        $d = @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | \
               Where-Object { $_.InstanceId -match $ms }); \
        $drv = @($d | Where-Object { $_.Service -in 'WinUSB','WinUsb','libusbK','libusb0' }); \
        Write-Output (\"present={0} winusb={1}\" -f $d.Count, $drv.Count)";
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;

    let out = Command::new("powershell")
        .args(["-NoProfile", "-NonInteractive", "-Command", SCRIPT])
        .creation_flags(CREATE_NO_WINDOW)
        .output();
    let Ok(out) = out else {
        warn!("could not run powershell Get-PnpDevice — skipping Kinect driver check");
        return false;
    };
    let stdout = String::from_utf8_lossy(&out.stdout);
    let field = |key: &str| -> u32 {
        stdout
            .split_whitespace()
            .find_map(|tok| tok.strip_prefix(key)?.parse().ok())
            .unwrap_or(0)
    };
    let present = field("present=");
    let winusb = field("winusb=");
    present > 0 && winusb == 0
}

#[cfg(not(any(target_os = "linux", target_os = "windows")))]
fn kinect_present_but_not_set_up() -> bool {
    false
}

/// Run the one-click fix for the current platform. `Ok(())` means it was
/// kicked off (the user still has to confirm an elevation prompt and wait);
/// `Err` carries a fallback hint for the UI to display.
///
/// * Linux: stage a combined v1 + v2 rules file (`90-kinect.rules`) and
///   install it into `/etc/udev/rules.d/` via `pkexec` then `sudo`, then
///   reload udev.
/// * Windows: locate `setup\setup.ps1` from the release layout and spawn
///   a hidden non-elevated PowerShell that calls
///   `Start-Process -Verb RunAs` on `powershell.exe -ExecutionPolicy
///   Bypass -File setup.ps1` (UAC prompt). The elevated script binds
///   WinUSB to every known Kinect VID/PID — see the project README.
#[cfg(target_os = "linux")]
fn fix_kinect_access() -> Result<(), String> {
    use std::io::ErrorKind;
    use std::process::Command;

    // Combined v1 + v2 rules — merge of libfreenect's 66-kinect.rules
    // (v1 PIDs) and libfreenect2's 90-kinect2.rules (v2 PIDs), inlined
    // so the installer works from a release build that doesn't ship
    // the submodules. MODE 0666 to drop the GROUP=video requirement
    // libfreenect's upstream rule uses (no group membership to manage).
    const RULES: &str = "\
# Kinect v1 + v2 — USB access for libfreenect / libfreenect2.
# Installed by headtracking-demo. Upstream sources: libfreenect
# 66-kinect.rules and libfreenect2 90-kinect2.rules, merged here.
# v1 Xbox 360 (model 1414):
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02ae\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02ad\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02b0\", MODE=\"0666\"
# v1 Kinect for Windows (model 1473):
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02c2\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02be\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02bf\", MODE=\"0666\"
# v2 sensor + firmware-update + adapter hub:
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02c4\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02d8\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02d9\", MODE=\"0666\"
";
    let staged = std::env::temp_dir().join("headtracking-90-kinect.rules");
    std::fs::write(&staged, RULES).map_err(|e| format!("staging rules file: {e}"))?;
    let inner = format!(
        "set -e; install -m 0644 '{}' /etc/udev/rules.d/90-kinect.rules; \
         udevadm control --reload-rules; udevadm trigger",
        staged.display()
    );
    let manual = format!("sudo sh -c '{inner}'");

    let mut last: Option<String> = None;
    for elevator in ["pkexec", "sudo"] {
        match Command::new(elevator)
            .arg("sh")
            .arg("-c")
            .arg(&inner)
            .status()
        {
            Ok(s) if s.success() => return Ok(()),
            Ok(s) => last = Some(format!("`{elevator}` exited with {s}")),
            Err(e) if e.kind() == ErrorKind::NotFound => continue,
            Err(e) => last = Some(format!("could not run `{elevator}`: {e}")),
        }
    }
    Err(match last {
        Some(why) => format!("{why}.\nRun it yourself, then click 'rescan':\n{manual}"),
        None => format!("no `pkexec` or `sudo` on PATH.\nRun this, then click 'rescan':\n{manual}"),
    })
}

#[cfg(target_os = "windows")]
fn fix_kinect_access() -> Result<(), String> {
    use std::os::windows::process::CommandExt;
    use std::process::Command;
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;

    let exe = std::env::current_exe().map_err(|e| format!("locating the executable: {e}"))?;
    let dir = exe.parent().unwrap_or_else(|| std::path::Path::new("."));
    // Release-ZIP layout puts the script under `setup\`; tolerate a flat
    // layout and a `bin\` subdir too.
    let candidates = [
        dir.join("setup").join("setup.ps1"),
        dir.join("setup.ps1"),
        dir.join("..").join("setup").join("setup.ps1"),
    ];
    let Some(script) = candidates.iter().find(|p| p.is_file()) else {
        return Err(format!(
            "couldn't find setup\\setup.ps1 next to {} — re-download the full release ZIP, or \
             bind WinUSB by hand with Zadig (see the project README, \"Manual Zadig \
             fallback\"): v1 → Xbox NUI Audio/Camera/Motor; v2 → Xbox NUI Sensor (045E:02C4).",
            dir.display()
        ));
    };
    let workdir = script.parent().unwrap_or(dir);

    // Two-step trampoline: spawn a hidden, *non*-elevated PowerShell
    // whose only job is to call `Start-Process -Verb RunAs` on
    // `powershell.exe -ExecutionPolicy Bypass -File setup.ps1`. The
    // RunAs verb is what pops the UAC consent dialog; the spawned
    // elevated PowerShell opens its own visible console (we don't pass
    // `-WindowStyle Hidden`) so the user can read the script's output
    // and the type-`yes` prompt. `-ExecutionPolicy Bypass` is required
    // because a fresh Windows defaults to Restricted/RemoteSigned and
    // would otherwise refuse our unsigned local script.
    //
    // We use this trampoline instead of a direct Win32 ShellExecuteW
    // call to keep the dependency tree std-only — the price is one
    // ephemeral PowerShell process (~100 ms) before UAC fires.
    let inner = format!(
        "$ErrorActionPreference='Stop'; Start-Process powershell -Verb RunAs \
         -WorkingDirectory '{workdir}' \
         -ArgumentList '-NoProfile','-ExecutionPolicy','Bypass','-File','{script}'",
        workdir = workdir.display(),
        script = script.display(),
    );
    let status = Command::new("powershell.exe")
        .args(["-NoProfile", "-NonInteractive", "-Command", &inner])
        .creation_flags(CREATE_NO_WINDOW)
        .status()
        .map_err(|e| format!("spawning powershell.exe: {e}"))?;
    if status.success() {
        return Ok(());
    }
    // Most common failure: the user clicked "No" on the UAC prompt,
    // which makes `Start-Process -Verb RunAs` throw under
    // `$ErrorActionPreference='Stop'` → launcher exits non-zero.
    Err(format!(
        "launcher PowerShell exited with {status} (often = UAC cancelled, or PowerShell \
         missing). Open an elevated PowerShell yourself and run:\n\
         powershell -NoProfile -ExecutionPolicy Bypass -File \"{}\"",
        script.display()
    ))
}

#[cfg(not(any(target_os = "linux", target_os = "windows")))]
fn fix_kinect_access() -> Result<(), String> {
    Err("nothing to set up — libusb opens the Kinect without a driver on this platform".to_string())
}

// ============================================================ App state

struct App {
    selected: Backend,
    available: Vec<BackendEntry>,
    active: Option<Active>,
    error: Option<String>,
    logs: Arc<Mutex<VecDeque<String>>>,
    /// A Kinect is on the USB bus but the OS-level access prerequisite
    /// isn't set up (Linux: missing libfreenect2 udev rule; Windows: no
    /// libusb/WinUSB driver bound) — show the one-click fix banner. Always
    /// `false` on macOS, where libusb needs nothing.
    kinect_access_hint: bool,
    /// Outcome of the last "fix it" click: `Ok` carries a follow-up note,
    /// `Err` carries a fallback hint (manual command line / "re-download
    /// the release ZIP"). Cleared on rescan.
    kinect_access_result: Option<Result<String, String>>,
    /// Outcome of the last "Screenshot" click — kept until the next click
    /// (or backend change) so the user has time to read the saved path.
    /// `Ok` carries the full saved path, `Err` carries the failure reason.
    screenshot_status: Option<Result<std::path::PathBuf, String>>,
}

impl App {
    fn label_for(&self, backend: Backend) -> String {
        self.available
            .iter()
            .find(|e| e.backend == backend)
            .map(|e| e.label.clone())
            .unwrap_or_else(|| match backend {
                Backend::None => "None (off)".to_string(),
                Backend::KinectV1 => "Kinect v1".to_string(),
                Backend::KinectV2 => "Kinect v2".to_string(),
                Backend::Webcam(i) => format!("Webcam #{i}"),
            })
    }
}

struct Active {
    backend: Backend,
    intrinsics: Intrinsics,
    rgb_texture: Option<TextureHandle>,
    /// 1€-smoothed head pose. Same shape as the raw `HeadPixel` but the
    /// position values come out of [`OneEuroPose3D`] so the crosshair and
    /// the VPX delta panel stop jittering at the pixel level.
    last_head: Option<HeadPixel>,
    baseline: Option<Baseline>,
    inner: Inner,
    /// `Some` only when [`Inner::KinectV1`] — drives the motorised base.
    v1_controls: Option<V1Controls>,
    pose_filter: filter_alias::OneEuroPose3D,
    started_at: Instant,
    /// Run lockbar detection on each depth frame and overlay it.
    lockbar_enabled: bool,
    last_lockbar: Option<headtracking::calibration::LockbarObservation>,
    /// `Some` when face detection is enabled (currently auto-enabled for
    /// the webcam backend). Cheap to keep around — YuNet runs in <10 ms
    /// at 320×320 on CPU.
    face_detector: Option<face::Detector>,
    last_faces: Vec<face::FaceDetection>,
    /// Latest RGB888 frame (width, height, bytes) — kept so the
    /// "Screenshot" button can write it to disk without re-grabbing
    /// from the device. `None` until the first frame arrives.
    last_rgb_frame: Option<(u32, u32, Vec<u8>)>,
}

mod filter_alias {
    // The plugin's filter module isn't exposed as a sibling crate, but it's
    // a standalone in-tree module — duplicate it here would mean another
    // copy of identical code. Instead, headtracking-demo pulls it directly via the
    // workspace's `headtracking` crate path.
    pub use headtracking::filter::{OneEuroParams, OneEuroPose3D};
}

/// Build the 3-axis 1€ filter for the head pose. X/Y get the library
/// defaults; Z gets a tighter `min_cutoff` because depth-camera readings
/// are inherently noisier (the median over a small pixel window
/// fluctuates as the face bbox shifts a pixel or two between frames).
fn make_pose_filter() -> filter_alias::OneEuroPose3D {
    let xy = filter_alias::OneEuroParams::default();
    let z = filter_alias::OneEuroParams {
        min_cutoff_hz: 0.4,
        beta: 0.05,
        derivative_cutoff_hz: 1.0,
    };
    filter_alias::OneEuroPose3D::new_per_axis([xy, xy, z])
}

/// State for the Kinect v1 tilt + LED panel. The desired values are kept
/// here so the user can drag the slider freely; we only push commands to
/// the device on `drag_stopped` / combo change to avoid hammering the
/// fragile motor gears.
struct V1Controls {
    desired_tilt_deg: f32,
    last_sent_tilt_deg: f32,
    selected_led: freenect::LedState,
    last_sent_led: freenect::LedState,
    last_state: Option<freenect::TiltState>,
    last_refresh: Instant,
}

impl V1Controls {
    fn new() -> Self {
        Self {
            desired_tilt_deg: 0.0,
            last_sent_tilt_deg: 0.0,
            selected_led: freenect::LedState::Green,
            last_sent_led: freenect::LedState::Green,
            last_state: None,
            // Seed with a stale instant so the first poll triggers a refresh.
            last_refresh: Instant::now() - Duration::from_secs(60),
        }
    }
}

#[derive(Clone, Copy)]
struct Intrinsics {
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
}

enum Inner {
    KinectV2 {
        device: freenect2::Device,
        _ctx: freenect2::Context,
    },
    KinectV1 {
        device: freenect::Device,
        _ctx: freenect::Context,
    },
    Webcam {
        camera: webcam::Camera,
    },
}

impl Inner {
    /// `true` when this input pipeline produces 3D head poses (depth blob
    /// for Kinect, face landmarks + IOD triangulation for webcam).
    fn has_head_tracker(&self) -> bool {
        matches!(
            self,
            Inner::KinectV1 { .. } | Inner::KinectV2 { .. } | Inner::Webcam { .. }
        )
    }
}

/// Pick the largest detected face by bounding-box area. The largest face is
/// usually the one closest to the camera, which on a pincab is the player.
fn pick_largest_face(faces: &[face::FaceDetection]) -> Option<&face::FaceDetection> {
    faces.iter().max_by(|a, b| {
        let area_a = a.width * a.height;
        let area_b = b.width * b.height;
        area_a
            .partial_cmp(&area_b)
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

/// Anchor the head pose to the face detector's bbox: scale the face center
/// from the RGB pixel grid into the depth pixel grid, sample a window of
/// valid depth values there, take the median (robust to outliers), then
/// deproject through the IR intrinsics. Returns `None` when not enough
/// valid depth pixels land inside the face window.
///
/// Naive linear rescale between the two pixel grids — on the Kinect v2 the
/// IR and RGB sensors are physically offset, so the sampled window can
/// drift a few pixels off the face for very close subjects. Good enough
/// to land on the head; libfreenect2's Registration is the proper fix and
/// gets wired up later.
fn head_from_face_depth(
    face: &face::FaceDetection,
    rgb_w: u32,
    rgb_h: u32,
    depth_data: &[f32],
    depth_w: u32,
    depth_h: u32,
    intr: &Intrinsics,
) -> Option<HeadPixel> {
    if rgb_w == 0 || rgb_h == 0 || depth_w == 0 || depth_h == 0 {
        return None;
    }
    let scale_x = depth_w as f32 / rgb_w as f32;
    let scale_y = depth_h as f32 / rgb_h as f32;
    let face_cx = face.x + face.width * 0.5;
    let face_cy = face.y + face.height * 0.5;
    let depth_cx = face_cx * scale_x;
    let depth_cy = face_cy * scale_y;
    let half_w = ((face.width * 0.4 * scale_x) as i32).clamp(4, 24);
    let half_h = ((face.height * 0.4 * scale_y) as i32).clamp(4, 24);
    let cx = depth_cx as i32;
    let cy = depth_cy as i32;
    let mut samples: Vec<f32> = Vec::with_capacity(((2 * half_w + 1) * (2 * half_h + 1)) as usize);
    for dv in -half_h..=half_h {
        let v = cy + dv;
        if v < 0 || v >= depth_h as i32 {
            continue;
        }
        let row = (v as usize) * depth_w as usize;
        for du in -half_w..=half_w {
            let u = cx + du;
            if u < 0 || u >= depth_w as i32 {
                continue;
            }
            let z = depth_data[row + u as usize];
            if (DEPTH_MIN_MM..=DEPTH_MAX_MM).contains(&z) {
                samples.push(z);
            }
        }
    }
    if samples.len() < 16 {
        return None;
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let depth_mm = samples[samples.len() / 2];

    let zf = f64::from(depth_mm);
    let x_mm = (f64::from(depth_cx - intr.cx) * zf / f64::from(intr.fx)) as f32;
    let y_mm = (f64::from(depth_cy - intr.cy) * zf / f64::from(intr.fy)) as f32;

    Some(HeadPixel {
        u: depth_cx.max(0.0) as u32,
        v: depth_cy.max(0.0) as u32,
        depth_mm,
        x_mm,
        y_mm,
    })
}

/// Build a [`HeadPixel`] from a face detection. Z is triangulated from the
/// interpupillary pixel distance assuming a 63 mm physical IOD and a
/// nominal `fx ≈ 0.85 × frame_width` (typical 60° HFOV webcam). These
/// numbers are placeholders until `ht-calibrate` measures the real focal
/// length via the lockbar fiducial.
fn face_to_head(face: &face::FaceDetection, frame_w: u32, frame_h: u32) -> HeadPixel {
    const IOD_MM: f32 = 63.0;
    let fx = 0.85 * frame_w as f32;
    let fy = fx;
    let cx = (frame_w as f32) / 2.0;
    let cy = (frame_h as f32) / 2.0;

    let dx = face.left_eye_x - face.right_eye_x;
    let dy = face.left_eye_y - face.right_eye_y;
    let pixel_iod = (dx * dx + dy * dy).sqrt().max(1.0);
    let depth_mm = IOD_MM * fx / pixel_iod;

    // Eye-midpoint as the head pixel.
    let u = (face.left_eye_x + face.right_eye_x) * 0.5;
    let v = (face.left_eye_y + face.right_eye_y) * 0.5;

    let x_mm = (u - cx) * depth_mm / fx;
    let y_mm = (v - cy) * depth_mm / fy;

    HeadPixel {
        u: u.max(0.0) as u32,
        v: v.max(0.0) as u32,
        depth_mm,
        x_mm,
        y_mm,
    }
}

#[derive(Debug, Clone, Copy)]
struct HeadPixel {
    /// Pixel coords inside the source frame (depth grid for Kinect, RGB
    /// frame for webcam). Surface only — used by the status label, no
    /// longer drives any overlay.
    u: u32,
    v: u32,
    depth_mm: f32,
    x_mm: f32,
    y_mm: f32,
}

#[derive(Debug, Clone, Copy)]
struct Baseline {
    x_mm: f32,
    y_mm: f32,
    z_mm: f32,
}

impl App {
    fn new(logs: Arc<Mutex<VecDeque<String>>>) -> Self {
        let available = detect_backends();
        let kinect_access_hint = compute_kinect_access_hint();
        Self {
            selected: Backend::None,
            available,
            active: None,
            error: None,
            logs,
            kinect_access_hint,
            kinect_access_result: None,
            screenshot_status: None,
        }
    }

    fn refresh_available(&mut self) {
        // Drop the active device first — libfreenect[2] can't reliably
        // enumerate while a sibling context holds an open device on Linux,
        // and `webcam::force_refresh` below requires no live camera handle.
        if let Some(old) = self.active.take() {
            info!(backend = ?old.backend, "closing backend before scan");
            drop(old);
        }
        // Cycle SDL3's camera subsystem so a freshly plugged webcam shows
        // up. Mandatory on Windows (MediaFoundation has no hot-plug);
        // harmless on Linux/macOS — those backends already track hot-plug,
        // re-init just yields the same list.
        if let Err(e) = webcam::force_refresh() {
            info!(?e, "webcam subsystem refresh failed");
        }
        self.selected = Backend::None;
        self.available = detect_backends();
        self.kinect_access_hint = compute_kinect_access_hint();
        self.kinect_access_result = None;
    }

    fn ensure_active(&mut self) {
        let needs_change = match (&self.active, self.selected) {
            (Some(a), sel) => a.backend != sel,
            (None, Backend::None) => false,
            (None, _) => true,
        };
        if !needs_change {
            return;
        }
        if let Some(old) = self.active.take() {
            info!(backend = ?old.backend, "closing backend");
            drop(old);
        }
        self.error = None;
        if matches!(self.selected, Backend::None) {
            return;
        }
        match open_backend(self.selected) {
            Ok(active) => {
                info!(
                    backend = ?active.backend,
                    fx = active.intrinsics.fx,
                    fy = active.intrinsics.fy,
                    cx = active.intrinsics.cx,
                    cy = active.intrinsics.cy,
                    "backend opened"
                );
                self.active = Some(active);
            }
            Err(e) => {
                error!(?e, "failed to open backend");
                self.error = Some(e);
                self.selected = Backend::None;
            }
        }
    }

    fn poll(&mut self, egui_ctx: &egui::Context) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        match &mut active.inner {
            Inner::KinectV2 { device, .. } => {
                if let Some(rgb) = device.poll_rgb() {
                    // Convert BGRX → RGB888 once; both the face detector
                    // and the screenshot button want packed RGB.
                    let rgb888 = bgrx_to_rgb888(&rgb.data);
                    if let Some(detector) = active.face_detector.as_ref() {
                        active.last_faces = detector.detect(&rgb888, rgb.width, rgb.height);
                    }
                    let img = bgrx_to_color_image(rgb.width, rgb.height, &rgb.data);
                    upload_texture(egui_ctx, &mut active.rgb_texture, img);
                    active.last_rgb_frame = Some((rgb.width, rgb.height, rgb888));
                }
                if let Some(depth) = device.poll_depth() {
                    // Prefer face-anchored depth sampling: the face detector
                    // tells us *where* the head is on RGB; we just read the
                    // depth there. No face → no pose this frame. The old
                    // closest-blob fallback was unreliable enough that
                    // having no pose is more honest than having a wrong
                    // one.
                    let head = pick_largest_face(&active.last_faces).and_then(|face| {
                        head_from_face_depth(
                            face,
                            1920,
                            1080,
                            &depth.data,
                            depth.width,
                            depth.height,
                            &active.intrinsics,
                        )
                    });
                    let smoothed = smooth_head(head, &mut active.pose_filter, active.started_at);
                    capture_baseline(&mut active.baseline, smoothed);
                    active.last_head = smoothed;
                    if active.lockbar_enabled {
                        active.last_lockbar = headtracking::calibration::detect_lockbar(
                            &depth.data,
                            depth.width,
                            depth.height,
                            &headtracking::calibration::LockbarParams::default(),
                        );
                    }
                }
            }
            Inner::KinectV1 { device, .. } => {
                if let Some(rgb) = device.poll_rgb() {
                    if let Some(detector) = active.face_detector.as_ref() {
                        active.last_faces = detector.detect(&rgb.data, rgb.width, rgb.height);
                    }
                    let img = rgb888_to_color_image(rgb.width, rgb.height, &rgb.data);
                    upload_texture(egui_ctx, &mut active.rgb_texture, img);
                    active.last_rgb_frame = Some((rgb.width, rgb.height, rgb.data));
                }
                if let Some(depth) = device.poll_depth() {
                    // libfreenect ships u16 mm; widen for the shared algo.
                    let f32_data: Vec<f32> = depth.data.iter().map(|&v| f32::from(v)).collect();
                    // Face-anchored depth only — see v2 branch for rationale.
                    let head = pick_largest_face(&active.last_faces).and_then(|face| {
                        head_from_face_depth(
                            face,
                            640,
                            480,
                            &f32_data,
                            depth.width,
                            depth.height,
                            &active.intrinsics,
                        )
                    });
                    let smoothed = smooth_head(head, &mut active.pose_filter, active.started_at);
                    capture_baseline(&mut active.baseline, smoothed);
                    active.last_head = smoothed;
                    if active.lockbar_enabled {
                        active.last_lockbar = headtracking::calibration::detect_lockbar(
                            &f32_data,
                            depth.width,
                            depth.height,
                            &headtracking::calibration::LockbarParams::default(),
                        );
                    }
                }
            }
            Inner::Webcam { camera } => {
                if let Some(rgb) = camera.poll_rgb() {
                    // Face detection on the raw camera frame (before the
                    // ColorImage conversion strips the contiguous bytes).
                    if let Some(detector) = active.face_detector.as_mut() {
                        active.last_faces = detector.detect(&rgb.data, rgb.width, rgb.height);
                        if let Some(face) = pick_largest_face(&active.last_faces) {
                            let head = face_to_head(face, rgb.width, rgb.height);
                            let smoothed =
                                smooth_head(Some(head), &mut active.pose_filter, active.started_at);
                            capture_baseline(&mut active.baseline, smoothed);
                            active.last_head = smoothed;
                        } else {
                            active.last_head = None;
                        }
                    }
                    let img = rgb888_to_color_image(rgb.width, rgb.height, &rgb.data);
                    upload_texture(egui_ctx, &mut active.rgb_texture, img);
                    active.last_rgb_frame = Some((rgb.width, rgb.height, rgb.data));
                }
            }
        }
    }
}

/// Apply the 1€ filter to the head pose in millimetres. The pixel coords
/// `u`, `v` are passed through unchanged — they record where on the depth
/// frame we sampled, not a re-projected smoothed point.
fn smooth_head(
    raw: Option<HeadPixel>,
    filter: &mut filter_alias::OneEuroPose3D,
    started_at: Instant,
) -> Option<HeadPixel> {
    let mut head = raw?;
    let t_us = started_at.elapsed().as_micros() as u64;
    let smoothed = filter.update([head.x_mm, head.y_mm, head.depth_mm], t_us);
    head.x_mm = smoothed[0];
    head.y_mm = smoothed[1];
    head.depth_mm = smoothed[2];
    Some(head)
}

impl App {
    /// Render the Kinect v1 tilt + LED panel just below the toolbar.
    /// No-op when the active backend is anything else.
    fn show_v1_controls(&mut self, egui_ctx: &egui::Context) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        let (Inner::KinectV1 { device, .. }, Some(controls)) =
            (&mut active.inner, active.v1_controls.as_mut())
        else {
            return;
        };

        // Refresh tilt + accel every 500 ms (USB roundtrip).
        if controls.last_refresh.elapsed() >= Duration::from_millis(500) {
            match device.tilt_state() {
                Ok(state) => controls.last_state = Some(state),
                Err(e) => warn!(?e, "kinect v1: tilt_state refresh failed"),
            }
            controls.last_refresh = Instant::now();
        }

        TopBottomPanel::top("v1-controls").show(egui_ctx, |ui| {
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                ui.label(RichText::new("Kinect v1").strong());
                ui.separator();
                let response = ui.add(
                    egui::Slider::new(
                        &mut controls.desired_tilt_deg,
                        freenect::TILT_MIN_DEG..=freenect::TILT_MAX_DEG,
                    )
                    .text("tilt °")
                    .step_by(1.0),
                );
                let drag_release = response.drag_stopped();
                let typed_commit = response.lost_focus();
                if (drag_release || typed_commit)
                    && (controls.desired_tilt_deg - controls.last_sent_tilt_deg).abs() > 0.01
                {
                    if let Err(e) = device.set_tilt_degrees(controls.desired_tilt_deg) {
                        warn!(?e, "set_tilt failed");
                    } else {
                        controls.last_sent_tilt_deg = controls.desired_tilt_deg;
                        info!(angle = controls.desired_tilt_deg, "tilt command sent");
                    }
                }

                ui.separator();
                ui.label("LED:");
                let prev_led = controls.selected_led;
                ComboBox::from_id_salt("led")
                    .selected_text(led_label(controls.selected_led))
                    .show_ui(ui, |ui| {
                        for led in LED_OPTIONS {
                            ui.selectable_value(&mut controls.selected_led, *led, led_label(*led));
                        }
                    });
                if controls.selected_led != prev_led {
                    if let Err(e) = device.set_led(controls.selected_led) {
                        warn!(?e, "set_led failed");
                    } else {
                        controls.last_sent_led = controls.selected_led;
                    }
                }

                ui.separator();
                if let Some(state) = controls.last_state {
                    ui.label(
                        RichText::new(format!(
                            "current {:>+5.1}°  status {:?}  accel ({:>+5.2}, {:>+5.2}, {:>+5.2}) m/s²",
                            state.angle_deg,
                            state.status,
                            state.accel_mks[0],
                            state.accel_mks[1],
                            state.accel_mks[2],
                        ))
                        .monospace()
                        .color(Color32::GRAY),
                    );
                } else {
                    ui.label(RichText::new("waiting for motor state…").color(Color32::GRAY));
                }
            });
            ui.add_space(2.0);
        });
    }
}

const LED_OPTIONS: &[freenect::LedState] = &[
    freenect::LedState::Off,
    freenect::LedState::Green,
    freenect::LedState::Red,
    freenect::LedState::Yellow,
    freenect::LedState::BlinkGreen,
    freenect::LedState::BlinkRedYellow,
];

fn led_label(state: freenect::LedState) -> &'static str {
    match state {
        freenect::LedState::Off => "off",
        freenect::LedState::Green => "green",
        freenect::LedState::Red => "red",
        freenect::LedState::Yellow => "yellow",
        freenect::LedState::BlinkGreen => "blink green",
        freenect::LedState::BlinkRedYellow => "blink red/yellow",
    }
}

fn upload_texture(ctx: &egui::Context, slot: &mut Option<TextureHandle>, img: ColorImage) {
    match slot.as_mut() {
        Some(tex) => tex.set(img, egui::TextureOptions::LINEAR),
        None => *slot = Some(ctx.load_texture("rgb", img, egui::TextureOptions::LINEAR)),
    }
}

fn capture_baseline(slot: &mut Option<Baseline>, head: Option<HeadPixel>) {
    if slot.is_some() {
        return;
    }
    let Some(head) = head else { return };
    let baseline = Baseline {
        x_mm: head.x_mm,
        y_mm: head.y_mm,
        z_mm: head.depth_mm,
    };
    *slot = Some(baseline);
    info!(
        x_mm = baseline.x_mm,
        y_mm = baseline.y_mm,
        z_mm = baseline.z_mm,
        "baseline captured"
    );
}

impl eframe::App for App {
    fn update(&mut self, egui_ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.ensure_active();
        self.poll(egui_ctx);

        // ----- Top toolbar
        TopBottomPanel::top("toolbar").show(egui_ctx, |ui| {
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Input:");
                let selected_label = self.label_for(self.selected);
                ComboBox::from_id_salt("backend")
                    .selected_text(selected_label)
                    .show_ui(ui, |ui| {
                        let entries = self.available.clone();
                        for entry in &entries {
                            ui.selectable_value(
                                &mut self.selected,
                                entry.backend,
                                &entry.label,
                            );
                        }
                    });
                if ui.small_button("rescan").clicked() {
                    self.refresh_available();
                }
                // Screenshot: writes the latest RGB frame next to the
                // binary as `<backend-slug>_<YYYYMMDD-HHMMSS>.png`.
                // Disabled until a frame has been received.
                let shot_ready = self
                    .active
                    .as_ref()
                    .is_some_and(|a| a.last_rgb_frame.is_some());
                let shot_resp = ui.add_enabled(
                    shot_ready,
                    egui::Button::new("📷 screenshot").small(),
                );
                if shot_resp.clicked()
                    && let Some(active) = self.active.as_ref()
                    && let Some((w, h, bytes)) = active.last_rgb_frame.as_ref()
                {
                    let slug = backend_slug(active.backend);
                    self.screenshot_status = Some(save_rgb_screenshot(&slug, *w, *h, bytes));
                    match &self.screenshot_status {
                        Some(Ok(p)) => info!(path = %p.display(), "screenshot saved"),
                        Some(Err(e)) => error!(error = %e, "screenshot failed"),
                        None => {}
                    }
                }
                if let Some(status) = &self.screenshot_status {
                    match status {
                        Ok(path) => ui.label(
                            RichText::new(format!(
                                "saved → {}",
                                path.file_name()
                                    .and_then(|s| s.to_str())
                                    .unwrap_or("(?)")
                            ))
                            .color(Color32::from_rgb(0x90, 0xee, 0x90))
                            .small(),
                        )
                        .on_hover_text(path.display().to_string()),
                        Err(e) => ui.colored_label(Color32::LIGHT_RED, format!("save failed: {e}")),
                    };
                }
                ui.separator();
                if let Some(active) = self.active.as_mut()
                    && active.inner.has_head_tracker()
                {
                    ui.checkbox(&mut active.lockbar_enabled, "lockbar");
                    if !active.lockbar_enabled {
                        active.last_lockbar = None;
                    }
                    if let Some(bar) = active.last_lockbar {
                        ui.label(
                            RichText::new(format!(
                                "row {}, width {} px, depth {:.0} mm (σ {:.1})",
                                bar.row,
                                bar.width_px(),
                                bar.mean_depth_mm,
                                bar.depth_stddev_mm,
                            ))
                            .color(Color32::from_rgb(0xff, 0x40, 0x80))
                            .monospace()
                            .size(11.0),
                        );
                    }
                    ui.separator();
                }
                if let Some(active) = self.active.as_ref() {
                    let label = self.label_for(active.backend);
                    if !active.inner.has_head_tracker() {
                        ui.label(
                            RichText::new(format!(
                                "{label}  |  capture only — head tracking pending"
                            ))
                            .color(Color32::GRAY),
                        );
                    } else if let Some(head) = active.last_head {
                        ui.label(
                            RichText::new(format!(
                                "{label}  |  distance {:.0} mm  |  pixel ({}, {})  |  3D ({:.0}, {:.0}, {:.0}) mm",
                                head.depth_mm,
                                head.u,
                                head.v,
                                head.x_mm,
                                head.y_mm,
                                head.depth_mm
                            ))
                            .monospace(),
                        );
                    } else {
                        ui.label(
                            RichText::new(format!("{label}  |  waiting for head detection…"))
                                .color(Color32::GRAY),
                        );
                    }
                } else if let Some(err) = &self.error {
                    ui.colored_label(Color32::LIGHT_RED, err);
                } else if self.available.len() <= 1 {
                    ui.label(RichText::new("no input detected — plug a Kinect and click 'rescan'").color(Color32::GRAY));
                } else {
                    ui.label(RichText::new("select an input").color(Color32::GRAY));
                }
            });
            ui.add_space(4.0);
        });

        // ----- Kinect access nudge (Kinect on the bus but no udev rule /
        // ----- WinUSB driver — offer the one-click fix)
        if self.kinect_access_hint || self.kinect_access_result.is_some() {
            let amber = Color32::from_rgb(0xff, 0xc4, 0x40);
            let mut do_fix = false;
            TopBottomPanel::top("kinect-access").show(egui_ctx, |ui| {
                ui.add_space(3.0);
                match &self.kinect_access_result {
                    None => {
                        ui.horizontal_wrapped(|ui| {
                            ui.label(
                                RichText::new("⚠ Kinect plugged in but not accessible")
                                    .color(amber)
                                    .strong(),
                            );
                            ui.label(RichText::new(KINECT_ACCESS_PROBLEM).color(Color32::GRAY));
                            if ui.button(KINECT_ACCESS_BUTTON).clicked() {
                                do_fix = true;
                            }
                        });
                    }
                    Some(Ok(msg)) => {
                        ui.label(
                            RichText::new(format!("✓ {msg}"))
                                .color(Color32::from_rgb(0x6c, 0xc7, 0x6c)),
                        );
                    }
                    Some(Err(detail)) => {
                        ui.label(
                            RichText::new("Couldn't do it automatically:")
                                .color(amber)
                                .strong(),
                        );
                        ui.label(RichText::new(detail).monospace().size(12.0));
                    }
                }
                ui.add_space(3.0);
            });
            if do_fix {
                match fix_kinect_access() {
                    Ok(()) => {
                        info!("Kinect access fix started");
                        self.kinect_access_hint = false;
                        self.kinect_access_result = Some(Ok(KINECT_ACCESS_OK_NOTE.to_string()));
                    }
                    Err(e) => {
                        error!(error = %e, "Kinect access fix failed");
                        self.kinect_access_result = Some(Err(e));
                    }
                }
            }
        }

        // ----- Optional Kinect v1 controls (tilt + LED)
        self.show_v1_controls(egui_ctx);

        // ----- Bottom split: logs (left) + VPX delta panel (right)
        TopBottomPanel::bottom("debug-panels")
            .resizable(true)
            .default_height(220.0)
            .min_height(80.0)
            .show(egui_ctx, |ui| {
                ui.add_space(4.0);
                ui.columns(2, |cols| {
                    // Left: tracing event log
                    cols[0].horizontal(|ui| {
                        ui.label(RichText::new("logs").strong());
                        if ui.small_button("clear").clicked() {
                            self.logs.lock().clear();
                        }
                    });
                    ScrollArea::vertical()
                        .id_salt("log-scroll")
                        .auto_shrink([false; 2])
                        .stick_to_bottom(true)
                        .show(&mut cols[0], |ui| {
                            let logs = self.logs.lock();
                            ui.with_layout(Layout::top_down(Align::LEFT), |ui| {
                                for line in logs.iter() {
                                    ui.label(RichText::new(line).monospace().size(12.0));
                                }
                            });
                        });

                    // Right: VPX delta panel
                    let mut reset_baseline = false;
                    cols[1].horizontal(|ui| {
                        ui.label(RichText::new("VPX output (Δ view)").strong());
                        if ui.small_button("reset baseline").clicked() {
                            reset_baseline = true;
                        }
                    });
                    cols[1].add_space(2.0);
                    if reset_baseline && let Some(active) = self.active.as_mut() {
                        active.baseline = None;
                    }
                    if let Some(active) = self.active.as_ref() {
                        if !active.inner.has_head_tracker() {
                            cols[1].label(
                                RichText::new(
                                    "this input has no head tracker yet\n\
                                     (face detection / monocular depth comes\n\
                                     with the webcam tracker — P3 roadmap)",
                                )
                                .color(Color32::GRAY)
                                .monospace(),
                            );
                            return;
                        }
                        match (active.baseline, active.last_head) {
                            (Some(base), Some(head)) => {
                                let dx_mm = head.x_mm - base.x_mm;
                                let dy_mm = head.y_mm - base.y_mm;
                                let dz_mm = head.depth_mm - base.z_mm;
                                let (vx, vy, vz) =
                                    pose_delta_to_view_delta_vpu(dx_mm, dy_mm, dz_mm);
                                cols[1].label(
                                    RichText::new(format!(
                                        "baseline (mm)  ({:>6.0}, {:>6.0}, {:>6.0})\n\
                                         current  (mm)  ({:>6.0}, {:>6.0}, {:>6.0})\n\
                                         Δ pose   (mm)  ({:>+6.0}, {:>+6.0}, {:>+6.0})\n\n\
                                         Δ view  (VPU)  ({:>+6.2}, {:>+6.2}, {:>+6.2})\n\
                                                       viewX += {:>+6.2}\n\
                                                       viewY += {:>+6.2}\n\
                                                       viewZ += {:>+6.2}",
                                        base.x_mm,
                                        base.y_mm,
                                        base.z_mm,
                                        head.x_mm,
                                        head.y_mm,
                                        head.depth_mm,
                                        dx_mm,
                                        dy_mm,
                                        dz_mm,
                                        vx,
                                        vy,
                                        vz,
                                        vx,
                                        vy,
                                        vz,
                                    ))
                                    .monospace()
                                    .size(12.0),
                                );
                            }
                            _ => {
                                cols[1].label(
                                    RichText::new("waiting for baseline…")
                                        .color(Color32::GRAY)
                                        .monospace(),
                                );
                            }
                        }
                    } else {
                        cols[1].label(
                            RichText::new("input is off")
                                .color(Color32::GRAY)
                                .monospace(),
                        );
                    }
                });
            });

        // ----- Center: image with crosshair
        CentralPanel::default().show(egui_ctx, |ui| {
            let avail = ui.available_size();
            let aspect = match self.active.as_ref() {
                Some(active) => match (&active.inner, active.rgb_texture.as_ref()) {
                    (_, Some(tex)) => {
                        let s = tex.size_vec2();
                        if s.y > 0.0 { s.x / s.y } else { 16.0 / 9.0 }
                    }
                    (Inner::KinectV2 { .. }, None) => 1920.0 / 1080.0,
                    (Inner::KinectV1 { .. }, None) => 640.0 / 480.0,
                    (Inner::Webcam { .. }, None) => 640.0 / 480.0,
                },
                None => 16.0 / 9.0,
            };
            let (img_w, img_h) = if avail.x / avail.y > aspect {
                (avail.y * aspect, avail.y)
            } else {
                (avail.x, avail.x / aspect)
            };
            let (rect, _) = ui.allocate_exact_size(Vec2::new(img_w, img_h), Sense::hover());

            if let Some(active) = self.active.as_ref() {
                if let Some(tex) = &active.rgb_texture {
                    ui.painter().image(
                        tex.id(),
                        rect,
                        Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
                        Color32::WHITE,
                    );
                    if let Some(bar) = active.last_lockbar {
                        draw_lockbar(ui.painter(), rect, bar);
                    }
                    // Source dimensions for bbox normalisation come from
                    // the texture itself, NOT from the backend's spec.
                    // On macOS AVFoundation the camera often delivers
                    // frames at a size different from what
                    // `SDL_GetCameraFormat` reported at open time —
                    // using the spec here mapped boxes to the wrong
                    // place (or off-screen). The texture was built from
                    // the same buffer the detector saw, so its size is
                    // authoritative.
                    let src_size = tex.size_vec2();
                    for face in &active.last_faces {
                        draw_face_bbox(ui.painter(), rect, face, src_size);
                    }
                } else {
                    centered(ui, rect, "waiting for first RGB frame…");
                }
            } else {
                let msg = self
                    .error
                    .as_deref()
                    .unwrap_or("select an input device above to start streaming");
                centered(ui, rect, msg);
            }
        });

        egui_ctx.request_repaint();
    }
}

fn centered(ui: &mut egui::Ui, rect: Rect, text: &str) {
    ui.painter().rect_filled(rect, 4.0, Color32::from_gray(20));
    ui.painter().text(
        rect.center(),
        egui::Align2::CENTER_CENTER,
        text,
        egui::FontId::proportional(16.0),
        Color32::LIGHT_GRAY,
    );
}

/// Draw a face bbox on top of the displayed image. The bbox is in the
/// frame's pixel coordinates, scaled by the source frame size for the
/// active backend (RGB sensor resolution: 1920×1080 for v2, 640×480 for
/// v1, native cam res for webcam — the texture in `rect` already encodes
/// the right aspect, we just need source dimensions to normalise).
/// `src_size` is the actual pixel dimensions of the frame the detector
/// processed (= the texture's size, not the camera spec). Passing the
/// wrong source size mis-projects the bbox; cf. the macOS AVFoundation
/// vs SDL3 camera-format mismatch the demo hit on the FaceTime cam.
fn draw_face_bbox(painter: &egui::Painter, rect: Rect, face: &face::FaceDetection, src_size: Vec2) {
    let frame_w = src_size.x;
    let frame_h = src_size.y;
    if frame_w <= 0.0 || frame_h <= 0.0 {
        return;
    }
    let to_screen = |x: f32, y: f32| -> Pos2 {
        rect.left_top() + Vec2::new((x / frame_w) * rect.width(), (y / frame_h) * rect.height())
    };
    let p1 = to_screen(face.x, face.y);
    let p2 = to_screen(face.x + face.width, face.y);
    let p3 = to_screen(face.x + face.width, face.y + face.height);
    let p4 = to_screen(face.x, face.y + face.height);
    let red = Color32::from_rgb(0xff, 0x60, 0x60);
    painter.line_segment([p1, p2], Stroke::new(2.0, red));
    painter.line_segment([p2, p3], Stroke::new(2.0, red));
    painter.line_segment([p3, p4], Stroke::new(2.0, red));
    painter.line_segment([p4, p1], Stroke::new(2.0, red));
    painter.text(
        p1 + Vec2::new(2.0, -14.0),
        egui::Align2::LEFT_BOTTOM,
        format!("{:.0}%", face.confidence * 100.0),
        egui::FontId::monospace(11.0),
        red,
    );
}

fn draw_lockbar(
    painter: &egui::Painter,
    rect: Rect,
    bar: headtracking::calibration::LockbarObservation,
) {
    if bar.frame_width == 0 || bar.frame_height == 0 {
        return;
    }
    // Same caveat as the head crosshair: depth frame and RGB frame on the
    // Kinect v2 are not co-axial, so the bar visualisation is a few pixels
    // off the true RGB position. Good enough for "is it locked on?".
    let v_norm = bar.row as f32 / bar.frame_height as f32;
    let l_norm = bar.left_col as f32 / bar.frame_width as f32;
    let r_norm = bar.right_col as f32 / bar.frame_width as f32;
    let p_left = rect.left_top() + Vec2::new(l_norm * rect.width(), v_norm * rect.height());
    let p_right = rect.left_top() + Vec2::new(r_norm * rect.width(), v_norm * rect.height());
    let pink = Color32::from_rgb(0xff, 0x40, 0x80);
    painter.line_segment([p_left, p_right], Stroke::new(3.0, pink));
    // Tick marks at each end.
    painter.line_segment(
        [p_left + Vec2::new(0.0, -8.0), p_left + Vec2::new(0.0, 8.0)],
        Stroke::new(2.0, pink),
    );
    painter.line_segment(
        [
            p_right + Vec2::new(0.0, -8.0),
            p_right + Vec2::new(0.0, 8.0),
        ],
        Stroke::new(2.0, pink),
    );
}

// ============================================================ Backend opening

fn open_backend(b: Backend) -> Result<Active, String> {
    match b {
        Backend::None => Err("no backend selected".to_string()),
        Backend::KinectV2 => open_kinect_v2(),
        Backend::KinectV1 => open_kinect_v1(),
        Backend::Webcam(idx) => open_webcam(idx),
    }
}

fn open_kinect_v2() -> Result<Active, String> {
    let ctx = freenect2::Context::new().map_err(|e| format!("freenect2 Context::new: {e}"))?;
    let count = ctx.enumerate();
    if count <= 0 {
        return Err("no Kinect v2 found on USB".to_string());
    }
    let device = ctx
        .open_default()
        .map_err(|e| format!("freenect2 open_default: {e}"))?;
    device
        .start_streams(true, true)
        .map_err(|e| format!("freenect2 start_streams: {e}"))?;
    let p = device.ir_params();
    let detector = init_face_detector("kinect-v2");
    Ok(Active {
        backend: Backend::KinectV2,
        intrinsics: Intrinsics {
            fx: p.fx,
            fy: p.fy,
            cx: p.cx,
            cy: p.cy,
        },
        rgb_texture: None,
        last_head: None,
        baseline: None,
        inner: Inner::KinectV2 { device, _ctx: ctx },
        v1_controls: None,
        pose_filter: make_pose_filter(),
        started_at: Instant::now(),
        lockbar_enabled: false,
        last_lockbar: None,
        face_detector: detector,
        last_faces: Vec::new(),
        last_rgb_frame: None,
    })
}

fn open_kinect_v1() -> Result<Active, String> {
    info!("kinect v1 open: building context");
    let ctx = freenect::Context::new().map_err(|e| format!("freenect Context::new: {e}"))?;
    let count = ctx.enumerate();
    info!(count, "kinect v1 open: pre-open enumerate");
    if count <= 0 {
        return Err("no Kinect v1 found on USB".to_string());
    }
    info!(index = 0, "kinect v1 open: calling freenect_open_device");
    let mut device = ctx.open(0).map_err(|e| {
        // Surface a precise error on the path that's most likely to
        // bite Windows users. The wrapper Display impl already maps
        // -12 to a UsbDk/Zadig hint; copying it through verbatim
        // gives the demo log a single line a user can paste.
        let msg = format!("freenect open: {e}");
        warn!("{msg}");
        msg
    })?;
    info!("kinect v1 open: device handle acquired, starting streams");
    device
        .start_streams(true, true)
        .map_err(|e| format!("freenect start_streams: {e}"))?;

    // Seed the v1 controls with the device's current tilt so the slider
    // doesn't snap on first use. Failures are non-fatal — we just log.
    let mut controls = V1Controls::new();
    match device.tilt_state() {
        Ok(state) => {
            controls.desired_tilt_deg = state.angle_deg;
            controls.last_sent_tilt_deg = state.angle_deg;
            controls.last_state = Some(state);
            controls.last_refresh = Instant::now();
        }
        Err(e) => warn!(?e, "kinect v1: tilt_state read at open failed"),
    }
    if let Err(e) = device.set_led(controls.selected_led) {
        warn!(?e, "kinect v1: initial set_led failed");
    }

    let detector = init_face_detector("kinect-v1");
    Ok(Active {
        backend: Backend::KinectV1,
        intrinsics: Intrinsics {
            fx: freenect::FX,
            fy: freenect::FY,
            cx: freenect::CX,
            cy: freenect::CY,
        },
        rgb_texture: None,
        last_head: None,
        baseline: None,
        inner: Inner::KinectV1 { device, _ctx: ctx },
        v1_controls: Some(controls),
        pose_filter: make_pose_filter(),
        started_at: Instant::now(),
        lockbar_enabled: false,
        last_lockbar: None,
        face_detector: detector,
        last_faces: Vec::new(),
        last_rgb_frame: None,
    })
}

fn init_face_detector(backend_name: &'static str) -> Option<face::Detector> {
    match face::Detector::new() {
        Ok(d) => {
            info!(
                backend = backend_name,
                "face detector initialised (Ultraface)"
            );
            Some(d)
        }
        Err(e) => {
            warn!(
                backend = backend_name,
                ?e,
                "face detector failed to initialise; running without it"
            );
            None
        }
    }
}

fn open_webcam(index: u32) -> Result<Active, String> {
    let camera = webcam::Camera::open(index).map_err(|e| format!("webcam open: {e}"))?;
    let detector = init_face_detector("webcam");
    Ok(Active {
        backend: Backend::Webcam(index),
        // Without lockbar/disc calibration, fx ≈ 0.85 × frame_width is a
        // reasonable placeholder for a generic 60° HFOV webcam. The values
        // get replaced by ht-calibrate output when that lands.
        intrinsics: Intrinsics {
            fx: 0.0,
            fy: 0.0,
            cx: 0.0,
            cy: 0.0,
        },
        rgb_texture: None,
        last_head: None,
        baseline: None,
        inner: Inner::Webcam { camera },
        v1_controls: None,
        pose_filter: make_pose_filter(),
        started_at: Instant::now(),
        lockbar_enabled: false,
        last_lockbar: None,
        face_detector: detector,
        last_faces: Vec::new(),
        last_rgb_frame: None,
    })
}

// ============================================================ Image conversion

/// Convert a BGRX (Kinect v2) buffer to packed RGB888 — needed because the
/// face detector takes RGB888 frames. Allocates a fresh `Vec` of size
/// `width * height * 3`. ~6 MB for 1920×1080; copy cost is negligible
/// compared to the detector's ~10 ms inference.
fn bgrx_to_rgb888(bgrx: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bgrx.len() / 4 * 3);
    for chunk in bgrx.chunks_exact(4) {
        out.push(chunk[2]); // R from BGRX
        out.push(chunk[1]); // G
        out.push(chunk[0]); // B
    }
    out
}

fn bgrx_to_color_image(width: u32, height: u32, data: &[u8]) -> ColorImage {
    debug_assert_eq!(data.len(), (width * height * 4) as usize);
    let mut pixels = Vec::with_capacity((width * height) as usize);
    for chunk in data.chunks_exact(4) {
        // libfreenect2 ships pixels as B, G, R, X.
        pixels.push(Color32::from_rgb(chunk[2], chunk[1], chunk[0]));
    }
    ColorImage {
        size: [width as usize, height as usize],
        pixels,
    }
}

fn rgb888_to_color_image(width: u32, height: u32, data: &[u8]) -> ColorImage {
    debug_assert_eq!(data.len(), (width * height * 3) as usize);
    let mut pixels = Vec::with_capacity((width * height) as usize);
    for chunk in data.chunks_exact(3) {
        // libfreenect ships v1 video as R, G, B.
        pixels.push(Color32::from_rgb(chunk[0], chunk[1], chunk[2]));
    }
    ColorImage {
        size: [width as usize, height as usize],
        pixels,
    }
}

// ============================================================ Screenshot

/// Compact slug for a backend, used in screenshot filenames. Mirrors the
/// dropdown label but stripped of spaces / punctuation so the file is
/// easy to grep and predictable in shells.
fn backend_slug(b: Backend) -> String {
    match b {
        Backend::None => "demo".to_string(),
        Backend::KinectV1 => "kinect-v1".to_string(),
        Backend::KinectV2 => "kinect-v2".to_string(),
        Backend::Webcam(i) => format!("webcam-{i}"),
    }
}

/// Format a UNIX-epoch-seconds value as `YYYYMMDD-HHMMSS` in UTC.
/// Inlined to avoid pulling `time` / `chrono` for one timestamp.
/// Algorithm: Howard Hinnant's civil-from-days. Valid 1970-01-01 → 9999.
fn format_utc_stamp(secs: u64) -> String {
    let total_days = (secs / 86_400) as i64;
    let secs_in_day = secs % 86_400;
    let h = secs_in_day / 3600;
    let m = (secs_in_day / 60) % 60;
    let s = secs_in_day % 60;
    let days = total_days + 719_468;
    let era = days.div_euclid(146_097);
    let doe = days.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y_civil = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month_civil = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = y_civil + i64::from(mp >= 10);
    format!("{y:04}{month_civil:02}{d:02}-{h:02}{m:02}{s:02}")
}

/// Encode an RGB888 buffer as PNG and write it next to the running
/// executable. Returns the saved path on success. Used by the
/// "Screenshot" toolbar button.
fn save_rgb_screenshot(
    slug: &str,
    width: u32,
    height: u32,
    rgb888: &[u8],
) -> Result<std::path::PathBuf, String> {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|e| format!("clock before UNIX epoch: {e}"))?
        .as_secs();
    let stamp = format_utc_stamp(secs);
    let dir = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(std::path::Path::to_path_buf))
        .unwrap_or_else(|| std::path::PathBuf::from("."));
    let path = dir.join(format!("{slug}_{stamp}.png"));

    let file = std::fs::File::create(&path).map_err(|e| format!("create {path:?}: {e}"))?;
    let writer = std::io::BufWriter::new(file);
    let mut encoder = png::Encoder::new(writer, width, height);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    let mut wr = encoder
        .write_header()
        .map_err(|e| format!("png header: {e}"))?;
    wr.write_image_data(rgb888)
        .map_err(|e| format!("png write: {e}"))?;
    Ok(path)
}

// ============================================================ VPU mapping
//
// Mirrors `crate::camera::mapping::pose_delta_to_view_delta` from the plugin.
// Keep in sync with `src/camera/mapping.rs` if the axis convention changes.

const VPU_PER_MM: f64 = 50.0 / (25.4 * 1.0625);

fn pose_delta_to_view_delta_vpu(dx_mm: f32, dy_mm: f32, dz_mm: f32) -> (f32, f32, f32) {
    let to_vpu = |mm: f32| (f64::from(mm) * VPU_PER_MM) as f32;
    // Kinect Y points down (head going up → -Y) and Z grows away from the
    // sensor (head approaching → -Z). VPX Camera-mode Y is "forward away
    // from the player" and Z is upward.
    (to_vpu(dx_mm), -to_vpu(dz_mm), -to_vpu(dy_mm))
}

// ============================================================ Tracing capture

fn init_tracing(sink: Arc<Mutex<VecDeque<String>>>) {
    let env_filter = tracing_subscriber::EnvFilter::try_from_env("HEADTRACKING_LOG")
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));

    // Console layer — only when stderr is actually a terminal. On a fresh
    // Windows release build (windows_subsystem = "windows") stderr is not
    // attached, so we don't even register the layer; on a `cargo run` from
    // a shell it lights up as usual.
    let stderr_layer = std::io::stderr().is_terminal().then(|| {
        tracing_subscriber::fmt::layer()
            .with_target(false)
            .with_writer(std::io::stderr)
    });

    // File layer — `headtracking-demo.log` next to the binary (append).
    // Same on every OS so triage instructions ("send me your log") are
    // uniform. If the file can't be opened (e.g. read-only Program Files
    // install), we skip it silently — the in-app panel still works.
    let file_layer = open_log_file().map(|f| {
        tracing_subscriber::fmt::layer()
            .with_target(false)
            .with_ansi(false)
            .with_writer(FileSink {
                inner: Arc::new(Mutex::new(f)),
            })
    });

    let panel_layer = tracing_subscriber::fmt::layer()
        .with_target(false)
        .with_ansi(false)
        .with_writer(LogQueue { sink });

    tracing_subscriber::registry()
        .with(env_filter)
        .with(stderr_layer)
        .with(file_layer)
        .with(panel_layer)
        .init();
}

/// Open `headtracking-demo.log` next to the executable in append mode.
/// Returns `None` when the executable path can't be resolved or the file
/// can't be opened (read-only install dir, missing permissions) — the
/// caller drops the file layer in that case.
fn open_log_file() -> Option<std::fs::File> {
    let exe = std::env::current_exe().ok()?;
    let path = exe.parent()?.join("headtracking-demo.log");
    std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .ok()
}

/// `MakeWriter` over a shared `File`. Each formatted event locks the file
/// for the duration of its write — fine here, the demo is mostly
/// single-threaded and tracing events are small. Avoids pulling
/// `tracing-appender` just for this.
#[derive(Clone)]
struct FileSink {
    inner: Arc<Mutex<std::fs::File>>,
}

impl<'a> MakeWriter<'a> for FileSink {
    type Writer = FileGuard<'a>;
    fn make_writer(&'a self) -> FileGuard<'a> {
        FileGuard(self.inner.lock())
    }
}

struct FileGuard<'a>(parking_lot::MutexGuard<'a, std::fs::File>);

impl Write for FileGuard<'_> {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        self.0.write(data)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}

#[derive(Clone)]
struct LogQueue {
    sink: Arc<Mutex<VecDeque<String>>>,
}

impl<'a> MakeWriter<'a> for LogQueue {
    type Writer = LogWriter;
    fn make_writer(&'a self) -> LogWriter {
        LogWriter {
            sink: Arc::clone(&self.sink),
            buf: Vec::with_capacity(256),
        }
    }
}

struct LogWriter {
    sink: Arc<Mutex<VecDeque<String>>>,
    buf: Vec<u8>,
}

impl Write for LogWriter {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        self.buf.extend_from_slice(data);
        Ok(data.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        if self.buf.is_empty() {
            return Ok(());
        }
        let text = String::from_utf8_lossy(&self.buf).into_owned();
        self.buf.clear();
        let mut sink = self.sink.lock();
        for line in text.lines() {
            if line.is_empty() {
                continue;
            }
            if sink.len() >= LOG_BUFFER_LINES {
                sink.pop_front();
            }
            sink.push_back(line.to_string());
        }
        Ok(())
    }
}

impl Drop for LogWriter {
    fn drop(&mut self) {
        let _ = self.flush();
    }
}
