//! Per-game tracker session: spawns a backend thread that streams `Pose`s
//! into an `ArcSwap`, which the render-thread `OnPrepareFrame` callback
//! reads without blocking.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use arc_swap::ArcSwap;
use tracing::{error, info};

#[cfg(feature = "kinect-v2")]
use super::kinect_v2::KinectV2Backend;
use super::{HeadTracker, Pose};

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
    /// Order of preference (matches feature flag priority): Kinect v2, ...
    pub fn spawn() -> Result<Self, SpawnError> {
        #[cfg(feature = "kinect-v2")]
        {
            return Self::spawn_kinect_v2();
        }
        #[allow(unreachable_code)]
        Err(SpawnError::NoBackendCompiled)
    }

    #[cfg(feature = "kinect-v2")]
    fn spawn_kinect_v2() -> Result<Self, SpawnError> {
        let backend = KinectV2Backend::open().map_err(SpawnError::OpenKinectV2)?;
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
                while !stop_for_thread.load(Ordering::Relaxed) {
                    if let Some(pose) = backend.poll() {
                        latest_for_thread.store(Arc::new(pose));
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
    OpenKinectV2(#[from] freenect2::Error),

    #[error("failed to spawn tracker thread: {0}")]
    Spawn(std::io::Error),
}
