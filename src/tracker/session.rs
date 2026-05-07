//! Per-game tracker session: spawns a backend thread that streams `Pose`s
//! into an `ArcSwap`, which the render-thread `OnPrepareFrame` callback
//! reads without blocking.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

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
use crate::filter::{OneEuroParams, OneEuroPose3D};

/// Owns the tracker thread and exposes the latest pose.
///
/// `Drop` signals the thread to stop and joins it before the process tears
/// down the device.
pub struct TrackerSession {
    latest: Arc<ArcSwap<Pose>>,
    stop: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
    backend_name: &'static str,
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
        let latest = Arc::new(ArcSwap::from_pointee(Pose::ZERO));
        let stop = Arc::new(AtomicBool::new(false));

        // Snapshot filter params at spawn time; live tweaks land at the
        // *next* OnGameStart so the running thread keeps coherent state.
        let xy = OneEuroParams::default();
        let z = OneEuroParams {
            min_cutoff_hz: cfg.min_cutoff_hz,
            beta: cfg.beta,
            derivative_cutoff_hz: 1.0,
        };

        let latest_for_thread = Arc::clone(&latest);
        let stop_for_thread = Arc::clone(&stop);
        let handle = thread::Builder::new()
            .name(format!("headtracking-{backend_name}"))
            .spawn(move || {
                info!(backend = backend_name, "tracker thread started");
                let mut filter = OneEuroPose3D::new_per_axis([xy, xy, z]);
                while !stop_for_thread.load(Ordering::Relaxed) {
                    if let Some(raw) = backend.poll() {
                        let smoothed = filter.update(raw.position_mm, raw.timestamp_us);
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
fn open_kinect_v2(_: usize) -> Result<Box<dyn HeadTracker>, SpawnError> {
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
