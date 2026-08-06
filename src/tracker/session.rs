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
use super::{HeadTracker, Pose};
use crate::config::{BackendKind, Config};
use crate::filter::{MedianGate, OneEuroParams, OneEuroPose3D};

/// Result of the RGB anchor-calibration phase run at session start.
#[cfg(any(feature = "kinect-v1", feature = "kinect-v2", feature = "webcam"))]
#[derive(Debug, Clone, Copy)]
pub struct CameraCalibration {
    pub geometry: anchor::AnchorGeometry,
    pub frame_w: u32,
    pub frame_h: u32,
    /// Real colour intrinsics `[fx, fy, cx, cy]` when the device knows them.
    pub color_intrinsics: Option<[f32; 4]>,
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
    let mut det = match anchor::AnchorDetector::new() {
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
        let Some((w, h, rgb)) = backend.poll_calibration_rgb() else {
            thread::sleep(Duration::from_millis(10));
            continue;
        };
        if let Some(d) = det.detect(&rgb, w, h) {
            first_hit.get_or_insert_with(Instant::now);
            if best.as_ref().is_none_or(|(s, ..)| d.score > *s) {
                best = Some((d.score, d, w, h));
            }
        }
        thread::sleep(ANCHOR_INTERVAL);
    }
    let (score, d, w, h) = best?;
    info!(
        score,
        w, h, "anchor: cabinet locked from live RGB detection"
    );
    Some(CameraCalibration {
        geometry: d.geometry(w, h),
        frame_w: w,
        frame_h: h,
        color_intrinsics: backend.color_intrinsics(),
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
}

impl TrackerSession {
    /// Spawn the tracker thread according to the live plugin configuration.
    /// `Auto` walks the fallback chain Kinect v2 → Kinect v1 → Webcam,
    /// returning the first one that opens cleanly. A specific backend
    /// fails fast if the device isn't available.
    pub fn spawn(cfg: &Config) -> Result<Self, SpawnError> {
        let device_index = cfg.device_index.max(0) as usize;
        let ir = cfg.tracking_stream == crate::config::StreamPref::Auto;
        let backend: Box<dyn HeadTracker> = match cfg.backend {
            BackendKind::KinectV2 => open_kinect_v2(device_index, ir)?,
            BackendKind::KinectV1 => open_kinect_v1(device_index, ir)?,
            BackendKind::Webcam => open_webcam(device_index)?,
            BackendKind::Auto => open_auto(device_index, ir)?,
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
                backend.begin_tracking();
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
fn open_kinect_v2(_device_index: usize, ir: bool) -> Result<Box<dyn HeadTracker>, SpawnError> {
    // libfreenect2 only exposes `open_default()` today; multi-Kinect-v2
    // selection would need a cxx bridge change. Falling back to the first
    // device is fine for the pincab use-case.
    use crate::tracker::pipeline::TrackingStream;
    let stream = if ir {
        TrackingStream::Ir
    } else {
        TrackingStream::Rgb
    };
    let b = KinectV2Backend::open(stream).map_err(SpawnError::OpenKinectV2)?;
    Ok(Box::new(b))
}
#[cfg(not(feature = "kinect-v2"))]
fn open_kinect_v2(_: usize, _: bool) -> Result<Box<dyn HeadTracker>, SpawnError> {
    Err(SpawnError::BackendNotCompiled("kinect-v2"))
}

#[cfg(feature = "kinect-v1")]
fn open_kinect_v1(_device_index: usize, ir: bool) -> Result<Box<dyn HeadTracker>, SpawnError> {
    // TODO: thread `_device_index` through KinectV1Backend::open once we
    // need to support multi-v1 setups. libfreenect itself supports it.
    let b = KinectV1Backend::open(ir).map_err(SpawnError::OpenKinectV1)?;
    Ok(Box::new(b))
}
#[cfg(not(feature = "kinect-v1"))]
fn open_kinect_v1(_: usize, _: bool) -> Result<Box<dyn HeadTracker>, SpawnError> {
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
fn open_auto(device_index: usize, ir: bool) -> Result<Box<dyn HeadTracker>, SpawnError> {
    if let Ok(b) = open_kinect_v2(device_index, ir).inspect_err(|e| {
        warn!(?e, "auto: kinect-v2 unavailable, trying next");
    }) {
        return Ok(b);
    }
    if let Ok(b) = open_kinect_v1(device_index, ir).inspect_err(|e| {
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
