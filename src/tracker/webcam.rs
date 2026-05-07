//! Webcam head tracker backend (SDL3 capture + YuNet face detection).
//!
//! Algorithm:
//!
//! 1. Poll the webcam (SDL3 picks the camera's native pixel format and we
//!    convert to RGB888 inside the `webcam` crate).
//! 2. Run YuNet on the colour frame; pick the largest detected face.
//! 3. Triangulate Z from the inter-pupillary distance: `Z = IOD_mm · fx /
//!    IOD_px` where `fx ≈ 0.85 × frame_width` (60° HFOV nominal).
//! 4. Deproject the eye midpoint through pinhole to (X, Y) in millimetres.
//!
//! Without lockbar / disc calibration the focal estimate is rough; the
//! per-axis baseline offset and gain settings let the user trim the result
//! at runtime. `tools/ht-calibrate` will refine `fx` and the per-user IOD
//! later.

use std::time::Instant;

use tracing::info;

use webcam::{Camera, CameraInfo};

use super::face_depth::pick_largest_face;
use super::{HeadTracker, Pose};

/// Physical inter-pupillary distance assumed for Z triangulation. 63 mm is
/// the population mean. Per-user calibration overrides this later.
const IOD_MM: f32 = 63.0;
/// Coarse focal estimate as a fraction of the frame width. Matches the
/// 60° HFOV of typical UVC webcams; refined by `ht-calibrate`.
const FOCAL_RATIO: f32 = 0.85;

pub struct WebcamBackend {
    camera: Camera,
    detector: face::Detector,
    last_faces: Vec<face::FaceDetection>,
    /// Cached focal length (px) derived from the camera's resolution at
    /// open time. Recomputed only if the resolution changes.
    fx: f32,
    cx: f32,
    cy: f32,
    started_at: Instant,
}

impl WebcamBackend {
    /// Open the n-th webcam advertised by the OS (0-based). When `index`
    /// is past the end of the list, the call falls back to the first
    /// webcam — a multi-camera pincab can pin a specific device via
    /// `[Plugin.HeadTracking] DeviceIndex=N`.
    pub fn open(index: usize) -> Result<Self, Error> {
        let cams = webcam::list()?;
        if cams.is_empty() {
            return Err(Error::NoDevice);
        }
        let pick: &CameraInfo = cams.get(index).unwrap_or(&cams[0]);
        let camera = Camera::open(pick.id)?;
        let detector = face::Detector::new()?;
        let w = camera.width();
        let h = camera.height();
        let fx = FOCAL_RATIO * w as f32;
        let cx = w as f32 / 2.0;
        let cy = h as f32 / 2.0;
        info!(
            id = pick.id,
            name = %pick.name,
            width = w,
            height = h,
            fx,
            "webcam: device opened"
        );
        Ok(Self {
            camera,
            detector,
            last_faces: Vec::new(),
            fx,
            cx,
            cy,
            started_at: Instant::now(),
        })
    }

    fn frame_to_pose(&mut self, frame: &webcam::RgbFrame) -> Option<Pose> {
        self.last_faces = self.detector.detect(&frame.data, frame.width, frame.height);
        let face = pick_largest_face(&self.last_faces)?;

        // Z from inter-pupillary pixel distance.
        let dx = face.left_eye_x - face.right_eye_x;
        let dy = face.left_eye_y - face.right_eye_y;
        let iod_px = (dx * dx + dy * dy).sqrt();
        if iod_px < 8.0 {
            // Eyes too close together to triangulate reliably (face very far,
            // landmarks likely noise) — drop this frame.
            return None;
        }
        let z_mm = IOD_MM * self.fx / iod_px;

        // (X, Y) from the eye midpoint via pinhole.
        let u = (face.left_eye_x + face.right_eye_x) * 0.5;
        let v = (face.left_eye_y + face.right_eye_y) * 0.5;
        let x_mm = (u - self.cx) * z_mm / self.fx;
        let y_mm = (v - self.cy) * z_mm / self.fx; // assume fx == fy

        Some(Pose {
            position_mm: [x_mm, y_mm, z_mm],
            timestamp_us: self.started_at.elapsed().as_micros() as u64,
            confidence: face.confidence.clamp(0.0, 1.0),
        })
    }
}

impl HeadTracker for WebcamBackend {
    fn poll(&mut self) -> Option<Pose> {
        let frame = self.camera.poll_rgb()?;
        self.frame_to_pose(&frame)
    }

    fn name(&self) -> &'static str {
        "webcam"
    }

    fn shutdown(&mut self) {
        // SDL3 camera handle closes on drop; nothing to do.
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("no webcam available")]
    NoDevice,
    #[error("webcam: {0}")]
    Webcam(#[from] webcam::Error),
    #[error("face detector init: {0}")]
    Face(#[from] face::Error),
}
