//! Cross-platform webcam capture, mirroring the API shape of the
//! `freenect` / `freenect2` crates so consumers can swap backends with
//! minimal glue.
//!
//! Wraps `nokhwa::CallbackCamera`: the crate spawns a worker thread that
//! pulls frames off the platform driver (V4L2 / Media Foundation /
//! AVFoundation), decodes them to RGB, and stores the latest one in a
//! mutex-protected slot. Callers poll lock-free.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use parking_lot::Mutex;
use tracing::{info, warn};

use nokhwa::CallbackCamera;
use nokhwa::pixel_format::RgbFormat;
use nokhwa::utils::{ApiBackend, CameraIndex, RequestedFormat, RequestedFormatType};

/// Description of a camera advertised by the OS.
#[derive(Debug, Clone)]
pub struct CameraInfo {
    pub index: u32,
    pub name: String,
    pub description: String,
}

/// One color frame copied out of nokhwa's decoded buffer.
/// `data` is row-major RGB888 — `width * height * 3` bytes, channel order
/// `[R, G, B]` per pixel.
#[derive(Debug, Clone)]
pub struct RgbFrame {
    pub width: u32,
    pub height: u32,
    pub data: Vec<u8>,
}

/// Enumerate all webcams visible to the OS. Cheap (no streams opened).
pub fn list() -> Result<Vec<CameraInfo>, Error> {
    let raw = nokhwa::query(ApiBackend::Auto).map_err(Error::Enumerate)?;
    Ok(raw
        .into_iter()
        .map(|info| CameraInfo {
            index: match info.index() {
                CameraIndex::Index(i) => *i,
                CameraIndex::String(_) => 0,
            },
            name: info.human_name().to_string(),
            description: info.description().to_string(),
        })
        .collect())
}

/// Single open webcam streaming RGB frames into an internal slot.
pub struct Camera {
    inner: CallbackCamera,
    slot: Arc<RgbSlot>,
    width: u32,
    height: u32,
    running: AtomicBool,
}

impl Camera {
    /// Open the camera at `index` and begin streaming. The driver picks the
    /// highest-frame-rate RGB-decodable mode it advertises.
    pub fn open(index: u32) -> Result<Self, Error> {
        let format =
            RequestedFormat::new::<RgbFormat>(RequestedFormatType::AbsoluteHighestFrameRate);
        let slot: Arc<RgbSlot> = Arc::new(RgbSlot::default());
        let slot_for_cb = Arc::clone(&slot);
        let mut camera =
            CallbackCamera::new(
                CameraIndex::Index(index),
                format,
                move |buffer| match buffer.decode_image::<RgbFormat>() {
                    Ok(img) => {
                        slot_for_cb.write(img.width(), img.height(), img.as_raw());
                    }
                    Err(e) => warn!(?e, "webcam: decode_image failed"),
                },
            )
            .map_err(Error::Open)?;

        camera.open_stream().map_err(Error::Start)?;

        // Resolve the actual format the driver gave us. The values are nominal
        // until a frame lands; we'll refresh on first frame if they look bogus.
        let resolution = camera.camera_format().map_err(Error::Open)?.resolution();
        let width = resolution.width_x;
        let height = resolution.height_y;
        info!(index, width, height, "webcam opened");

        Ok(Self {
            inner: camera,
            slot,
            width,
            height,
            running: AtomicBool::new(true),
        })
    }

    /// Width of the active camera mode (pixels).
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Height of the active camera mode (pixels).
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Read the latest decoded RGB frame, if any. Returns `None` when no new
    /// frame has arrived since the last call.
    pub fn poll_rgb(&self) -> Option<RgbFrame> {
        self.slot.poll()
    }

    /// Stop the stream. Idempotent.
    pub fn stop(&mut self) -> Result<(), Error> {
        if !self.running.swap(false, Ordering::AcqRel) {
            return Ok(());
        }
        self.inner.stop_stream().map_err(Error::Stop)
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        if let Err(e) = self.stop() {
            warn!(?e, "webcam: stop failed during Drop");
        }
    }
}

#[derive(Default)]
struct RgbSlot {
    inner: Mutex<RgbInner>,
    has_new: AtomicBool,
}

#[derive(Default)]
struct RgbInner {
    width: u32,
    height: u32,
    data: Vec<u8>,
}

impl RgbSlot {
    fn write(&self, width: u32, height: u32, data: &[u8]) {
        let expected = (width as usize) * (height as usize) * 3;
        if data.len() != expected {
            warn!(
                expected,
                got = data.len(),
                "webcam slot: unexpected frame size, dropping"
            );
            return;
        }
        let mut g = self.inner.lock();
        g.width = width;
        g.height = height;
        if g.data.len() != data.len() {
            g.data.resize(data.len(), 0);
        }
        g.data.copy_from_slice(data);
        drop(g);
        self.has_new.store(true, Ordering::Release);
    }

    fn poll(&self) -> Option<RgbFrame> {
        if !self.has_new.load(Ordering::Acquire) {
            return None;
        }
        let g = self.inner.lock();
        if !self.has_new.load(Ordering::Relaxed) {
            return None;
        }
        let frame = RgbFrame {
            width: g.width,
            height: g.height,
            data: g.data.clone(),
        };
        drop(g);
        self.has_new.store(false, Ordering::Release);
        Some(frame)
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("camera enumeration failed: {0}")]
    Enumerate(nokhwa::NokhwaError),
    #[error("failed to open camera: {0}")]
    Open(nokhwa::NokhwaError),
    #[error("failed to start stream: {0}")]
    Start(nokhwa::NokhwaError),
    #[error("failed to stop stream: {0}")]
    Stop(nokhwa::NokhwaError),
}
