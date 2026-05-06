//! Per-game tracker session: spawns a backend thread that streams `Pose`s
//! into an `ArcSwap`, which the render-thread `OnPrepareFrame` callback
//! reads without blocking.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use arc_swap::ArcSwap;
use tracing::{error, info};

#[cfg(feature = "kinect-v1")]
use super::kinect_v1::KinectV1Backend;
#[cfg(feature = "kinect-v2")]
use super::kinect_v2::KinectV2Backend;
use super::{HeadTracker, Pose};
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
    /// Spawn the tracker thread for the first compiled-in backend.
    /// Order of preference: Kinect v2 (more accurate), then Kinect v1. The
    /// `not(feature = "kinect-v2")` guard on the v1 arm avoids an
    /// unreachable-code warning when both features are enabled.
    pub fn spawn() -> Result<Self, SpawnError> {
        #[cfg(feature = "kinect-v2")]
        {
            return Self::spawn_kinect_v2();
        }
        #[cfg(all(feature = "kinect-v1", not(feature = "kinect-v2")))]
        {
            return Self::spawn_kinect_v1();
        }
        #[allow(unreachable_code)]
        Err(SpawnError::NoBackendCompiled)
    }

    #[cfg(feature = "kinect-v2")]
    fn spawn_kinect_v2() -> Result<Self, SpawnError> {
        let backend = KinectV2Backend::open().map_err(SpawnError::OpenKinectV2)?;
        Self::spawn_with(Box::new(backend))
    }

    #[cfg(feature = "kinect-v1")]
    // When v2 is also compiled in, `spawn()` always picks it; v1 stays
    // available for callers that explicitly disable the v2 feature.
    #[cfg_attr(feature = "kinect-v2", allow(dead_code))]
    fn spawn_kinect_v1() -> Result<Self, SpawnError> {
        let backend = KinectV1Backend::open().map_err(SpawnError::OpenKinectV1)?;
        Self::spawn_with(Box::new(backend))
    }

    // Kept generic over the backend so v1 / webcam backends can plug in
    // through the same path without each spawning their own thread; until
    // those land, only `spawn_kinect_v2` calls in.
    #[cfg_attr(
        not(any(feature = "kinect-v1", feature = "kinect-v2")),
        allow(dead_code)
    )]
    fn spawn_with(mut backend: Box<dyn HeadTracker>) -> Result<Self, SpawnError> {
        let backend_name = backend.name();
        let latest = Arc::new(ArcSwap::from_pointee(Pose::ZERO));
        let stop = Arc::new(AtomicBool::new(false));

        let latest_for_thread = Arc::clone(&latest);
        let stop_for_thread = Arc::clone(&stop);
        let handle = thread::Builder::new()
            .name(format!("headtracking-{backend_name}"))
            .spawn(move || {
                info!(backend = backend_name, "tracker thread started");
                // Per-axis 1€ params: X/Y use the library defaults; Z gets a
                // tighter cutoff because depth-camera readings are inherently
                // noisier (the median over a small bbox window fluctuates as
                // the face shifts a pixel or two between frames).
                let xy = OneEuroParams::default();
                let z = OneEuroParams {
                    min_cutoff_hz: 0.4,
                    beta: 0.05,
                    derivative_cutoff_hz: 1.0,
                };
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

#[derive(Debug, thiserror::Error)]
pub enum SpawnError {
    #[error("no head tracking backend was compiled in")]
    NoBackendCompiled,

    #[cfg(feature = "kinect-v2")]
    #[error("Kinect v2 open failed: {0}")]
    OpenKinectV2(#[from] super::kinect_v2::Error),

    #[cfg(feature = "kinect-v1")]
    #[error("Kinect v1 open failed: {0}")]
    OpenKinectV1(#[from] super::kinect_v1::Error),

    #[error("failed to spawn tracker thread: {0}")]
    Spawn(std::io::Error),
}
