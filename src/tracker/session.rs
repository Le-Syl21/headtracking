//! Per-game tracker session: spawns a backend thread that streams `Pose`s
//! into an `ArcSwap`, which the render-thread `OnPrepareFrame` callback
//! reads without blocking.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;
#[cfg(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam"))]
use std::time::Instant;

use arc_swap::ArcSwap;
use tracing::{error, info, warn};

#[cfg(feature = "kinect-v1")]
use super::kinect_v1::KinectV1Backend;
#[cfg(feature = "kinect-v2")]
use super::kinect_v2::KinectV2Backend;
#[cfg(feature = "webcam")]
use super::webcam::WebcamBackend;
use super::{HeadTracker, Pose, TrackingFault};
use crate::config::{BackendKind, Config};
use crate::filter::{MedianGate, OneEuroParams, OneEuroPose3D};

/// Result of the RGB anchor-calibration phase run at session start.
#[cfg(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam"))]
#[derive(Debug, Clone, Copy)]
pub struct CameraCalibration {
    pub geometry: anchor::AnchorGeometry,
    pub frame_w: u32,
    pub frame_h: u32,
    /// Intrinsics `[fx, fy, cx, cy]` of the stream the anchor was detected
    /// in -- colour, or infrared on a Kinect v2 tracking in IR.
    pub calibration_intrinsics: Option<[f32; 4]>,
    pub score: f32,
}

/// Stub so the featureless build (unit tests) keeps the same session API.
#[cfg(not(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam")))]
#[derive(Debug, Clone, Copy)]
pub struct CameraCalibration;

/// Detection cadence during calibration — the cabinet is fixed, no rush.
#[cfg(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam"))]
const ANCHOR_INTERVAL: Duration = Duration::from_millis(300);
/// After the FIRST successful detection, keep improving for this long,
/// then freeze the best (the demo-validated best-of-warmup strategy).
#[cfg(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam"))]
const ANCHOR_WARMUP: Duration = Duration::from_millis(2500);
/// No detection at all after this long: give up — the plugin tracks in
/// relative mode exactly as before, just without the camera-pose note.
#[cfg(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam"))]
const ANCHOR_TIMEOUT: Duration = Duration::from_secs(6);

/// Run the anchor model on colour frames until the warmup closes. Returns
/// the best-scoring detection's geometry, or `None` (no model, no frames,
/// nothing recognized).
#[cfg(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam"))]
fn run_anchor_calibration(
    backend: &mut Box<dyn HeadTracker>,
    stop: &AtomicBool,
) -> Option<CameraCalibration> {
    let mut det = match anchor::AnchorDetector::new(backend.calibration_stream()) {
        Ok(d) => d,
        Err(e) => {
            warn!("anchor: detector init failed ({e}); skipping calibration");
            return None;
        }
    };
    let started = Instant::now();
    let mut first_hit: Option<Instant> = None;
    let mut best: Option<(f32, anchor::AnchorDetection, u32, u32)> = None;
    loop {
        if stop.load(Ordering::Relaxed) {
            return None;
        }
        match first_hit {
            None if started.elapsed() > ANCHOR_TIMEOUT => break,
            Some(t) if t.elapsed() > ANCHOR_WARMUP => break,
            _ => {}
        }
        let Some((w, h, rgb)) = backend.poll_calibration_frame() else {
            thread::sleep(Duration::from_millis(10));
            continue;
        };
        // `poll_calibration_frame` hands over packed RGB whatever the sensor
        // is: this path is throttled to `ANCHOR_INTERVAL` and only runs while
        // calibrating, so the repack it costs is not worth a layout to carry.
        if let Some(d) = det.detect(&rgb, w, h, anchor::PixelLayout::Rgb888) {
            first_hit.get_or_insert_with(Instant::now);
            if best.as_ref().is_none_or(|(s, ..)| d.score > *s) {
                best = Some((d.score, d, w, h));
            }
        }
        thread::sleep(ANCHOR_INTERVAL);
    }
    let (score, d, w, h) = best?;
    info!(score, w, h, "anchor: cabinet locked from live detection");
    Some(CameraCalibration {
        geometry: d.geometry(w, h),
        frame_w: w,
        frame_h: h,
        calibration_intrinsics: backend.calibration_intrinsics(),
        score,
    })
}

#[cfg(not(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam")))]
fn run_anchor_calibration(
    _backend: &mut Box<dyn HeadTracker>,
    _stop: &AtomicBool,
) -> Option<CameraCalibration> {
    None
}

/// Owns the tracker thread and exposes the latest pose.
///
/// `Drop` signals the thread to stop and joins it before the process tears
/// down the device.
pub struct TrackerSession {
    latest: Arc<ArcSwap<Pose>>,
    stop: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
    backend_name: &'static str,
    /// User-facing device label (webcam product name, Kinect model+stream).
    device_label: String,
    /// Set by the recenter path; the tracker loop consumes it and resets
    /// the One-Euro filter so the fresh baseline isn't dragged from the
    /// old smoothed position.
    reset_filter: Arc<AtomicBool>,
    /// Filled by the tracker thread once the RGB anchor-calibration phase
    /// completes successfully (never, if nothing was recognized).
    calibration: Arc<std::sync::Mutex<Option<CameraCalibration>>>,
    /// Why the tracker thread gave up, when it did. A cause rather than a
    /// sentence — the plugin owns the wording, so it can be translated. Read
    /// once and pushed as a native VPX notification: a session that cannot
    /// reach its tracking stream must say so on screen, not only in a log
    /// nobody opens.
    fault: Arc<std::sync::Mutex<Option<TrackingFault>>>,
}

impl TrackerSession {
    /// Spawn the tracker thread according to the live plugin configuration.
    /// `Auto` walks the fallback chain Kinect v2 → Kinect v1 → Webcam,
    /// returning the first one that opens cleanly. A specific backend
    /// fails fast if the device isn't available.
    pub fn spawn(cfg: &Config) -> Result<Self, SpawnError> {
        let device_index = cfg.device_index.max(0) as usize;
        let backend: Box<dyn HeadTracker> = match cfg.backend {
            BackendKind::KinectV2 => open_kinect_v2(device_index)?,
            BackendKind::KinectV1 => open_kinect_v1(device_index)?,
            BackendKind::Webcam => open_webcam(device_index)?,
            BackendKind::Auto => open_auto(device_index)?,
        };
        Self::spawn_with(backend, cfg)
    }

    fn spawn_with(mut backend: Box<dyn HeadTracker>, cfg: &Config) -> Result<Self, SpawnError> {
        let backend_name = backend.name();
        let device_label = backend.device_label();
        let latest = Arc::new(ArcSwap::from_pointee(Pose::ZERO));
        let stop = Arc::new(AtomicBool::new(false));
        let reset_filter = Arc::new(AtomicBool::new(false));
        let calibration = Arc::new(std::sync::Mutex::new(None));
        let fault = Arc::new(std::sync::Mutex::new(None));

        let to_params = |cfg: &Config| {
            cfg.one_euro_params().map(|a| OneEuroParams {
                min_cutoff_hz: a.min_cutoff_hz,
                beta: a.beta,
                derivative_cutoff_hz: 1.0,
            })
        };
        let mut active = cfg.smoothing;
        let initial = to_params(cfg);

        let latest_for_thread = Arc::clone(&latest);
        let stop_for_thread = Arc::clone(&stop);
        let reset_for_thread = Arc::clone(&reset_filter);
        let calibration_for_thread = Arc::clone(&calibration);
        let fault_for_thread = Arc::clone(&fault);
        let handle = thread::Builder::new()
            .name(format!("headtracking-{backend_name}"))
            .spawn(move || {
                info!(backend = backend_name, "tracker thread started");
                // Phase 1 — anchor calibration on the COLOUR stream (the
                // model wants RGB; on Kinects the tracking stream is IR).
                if let Some(calib) = run_anchor_calibration(&mut backend, &stop_for_thread) {
                    *calibration_for_thread.lock().expect("calibration mutex") = Some(calib);
                }
                // Phase 2 — hand the device to the tracking stream and run.
                // A backend that cannot get there stops here: half-working
                // tracking is worse than none, because it gets reported as a
                // tracking bug instead of as the busy device it is.
                if let Err(why) = backend.begin_tracking() {
                    warn!(backend = backend_name, ?why, "tracker thread stopping");
                    *fault_for_thread.lock().expect("fault mutex") = Some(why);
                    return;
                }
                let mut filter = OneEuroPose3D::new_per_axis(initial);
                // Spike gate ahead of the One-Euro filter; the window size
                // is a live setting (see MedianGate docs for the trade-off).
                let mut gate = MedianGate::new(crate::config::current().median_window_frames());
                while !stop_for_thread.load(Ordering::Relaxed) {
                    // The in-game settings page edits the preset LIVE:
                    // follow it without waiting for the next game.
                    let live = crate::config::current();
                    if live.smoothing != active {
                        active = live.smoothing;
                        filter.set_params_per_axis(to_params(&live));
                        info!(backend = backend_name, preset = ?active, "smoothing preset changed");
                    }
                    if gate.window() != live.median_window_frames() {
                        gate.set_window(live.median_window_frames());
                        info!(
                            backend = backend_name,
                            window = gate.window(),
                            "median window changed"
                        );
                    }
                    if reset_for_thread.swap(false, Ordering::Relaxed) {
                        filter.reset();
                        gate.reset();
                    }
                    if let Some(raw) = backend.poll() {
                        let gated = gate.push(raw.position_mm);
                        let smoothed = filter.update(gated, raw.timestamp_us);
                        latest_for_thread.store(Arc::new(Pose {
                            position_mm: smoothed,
                            ..raw
                        }));
                    } else {
                        // No new frame yet; brief sleep instead of a spin.
                        thread::sleep(Duration::from_millis(2));
                    }
                }
                backend.shutdown();
                info!(backend = backend_name, "tracker thread exited");
            })
            .map_err(SpawnError::Spawn)?;

        Ok(Self {
            latest,
            stop,
            handle: Some(handle),
            backend_name,
            device_label,
            reset_filter,
            calibration,
            fault,
        })
    }

    /// Snapshot of the most recent pose. Cheap (atomic load + Arc clone).
    /// Returns `Pose::ZERO` if no pose has arrived yet.
    pub fn latest_pose(&self) -> Arc<Pose> {
        self.latest.load_full()
    }

    pub fn backend_name(&self) -> &'static str {
        self.backend_name
    }

    /// User-facing device label for notifications.
    pub fn device_label(&self) -> &str {
        &self.device_label
    }

    /// Ask the tracker thread to reset its smoothing filter (recenter).
    pub fn reset_filter(&self) {
        self.reset_filter.store(true, Ordering::Relaxed);
    }

    /// Camera calibration from the session-start RGB anchor phase, once
    /// the tracker thread has produced one.
    pub fn calibration(&self) -> Option<CameraCalibration> {
        *self.calibration.lock().expect("calibration mutex")
    }

    /// Take the tracker thread's fault message, if it left one. Taken rather
    /// than read, so the notification fires once.
    pub fn take_fault(&self) -> Option<TrackingFault> {
        self.fault.lock().expect("fault mutex").take()
    }
}

impl Drop for TrackerSession {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take()
            && handle.join().is_err()
        {
            error!("tracker thread panicked during shutdown");
        }
    }
}

// ============================================================ Backend openers

#[cfg(feature = "kinect-v2")]
fn open_kinect_v2(_device_index: usize) -> Result<Box<dyn HeadTracker>, SpawnError> {
    // libfreenect2 only exposes `open_default()` today; multi-Kinect-v2
    // selection would need a cxx bridge change. Falling back to the first
    // device is fine for the pincab use-case.
    let b = KinectV2Backend::open().map_err(SpawnError::OpenKinectV2)?;
    Ok(Box::new(b))
}
#[cfg(not(feature = "kinect-v2"))]
fn open_kinect_v2(_: usize, _: bool) -> Result<Box<dyn HeadTracker>, SpawnError> {
    Err(SpawnError::BackendNotCompiled("kinect-v2"))
}

#[cfg(feature = "kinect-v1")]
fn open_kinect_v1(_device_index: usize) -> Result<Box<dyn HeadTracker>, SpawnError> {
    // TODO: thread `_device_index` through KinectV1Backend::open once we
    // need to support multi-v1 setups. libfreenect itself supports it.
    let b = KinectV1Backend::open().map_err(SpawnError::OpenKinectV1)?;
    Ok(Box::new(b))
}
#[cfg(not(feature = "kinect-v1"))]
fn open_kinect_v1(_: usize) -> Result<Box<dyn HeadTracker>, SpawnError> {
    Err(SpawnError::BackendNotCompiled("kinect-v1"))
}

#[cfg(feature = "webcam")]
fn open_webcam(device_index: usize) -> Result<Box<dyn HeadTracker>, SpawnError> {
    let b = WebcamBackend::open(device_index).map_err(SpawnError::OpenWebcam)?;
    Ok(Box::new(b))
}
#[cfg(not(feature = "webcam"))]
fn open_webcam(_: usize) -> Result<Box<dyn HeadTracker>, SpawnError> {
    Err(SpawnError::BackendNotCompiled("webcam"))
}

/// Walk the v2 → v1 → webcam fallback chain. Each failure is logged so
/// the user can tell from the VPX log why a particular backend was
/// skipped (no device, USB error, model load, …).
fn open_auto(device_index: usize) -> Result<Box<dyn HeadTracker>, SpawnError> {
    if let Ok(b) = open_kinect_v2(device_index).inspect_err(|e| {
        warn!(?e, "auto: kinect-v2 unavailable, trying next");
    }) {
        return Ok(b);
    }
    if let Ok(b) = open_kinect_v1(device_index).inspect_err(|e| {
        warn!(?e, "auto: kinect-v1 unavailable, trying next");
    }) {
        return Ok(b);
    }
    open_webcam(device_index).inspect_err(|e| {
        warn!(?e, "auto: webcam unavailable — no backend left");
    })
}

#[derive(Debug, thiserror::Error)]
pub enum SpawnError {
    #[error("backend not compiled into this build: {0}")]
    BackendNotCompiled(&'static str),

    #[cfg(feature = "kinect-v2")]
    #[error("Kinect v2 open failed: {0}")]
    OpenKinectV2(#[from] super::kinect_v2::Error),

    #[cfg(feature = "kinect-v1")]
    #[error("Kinect v1 open failed: {0}")]
    OpenKinectV1(#[from] super::kinect_v1::Error),

    #[cfg(feature = "webcam")]
    #[error("Webcam open failed: {0}")]
    OpenWebcam(#[from] super::webcam::Error),

    #[error("failed to spawn tracker thread: {0}")]
    Spawn(std::io::Error),
}
