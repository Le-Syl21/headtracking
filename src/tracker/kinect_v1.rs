//! Kinect v1 head tracker backend (libfreenect via the `freenect` crate).
//!
//! Same algorithm as the v2 backend (face detection on the colour stream
//! → median depth inside the bbox → IR deprojection), adapted for v1's
//! 640×480 sensor and u16 depth frames. libfreenect doesn't expose
//! factory intrinsics, so we use the published Microsoft nominal values;
//! per-cab calibration can refine them later via `tools/ht-calibrate`.
//!
//! Untested on hardware (Sylvain only has a v2 sensor). Compile-time
//! parity is what we check in CI; hardware runs come later.

use std::time::Instant;

use tracing::{info, warn};

use freenect::{CX, CY, Context, DEPTH_HEIGHT, DEPTH_WIDTH, DepthFrame, Device, FX, FY};

use super::face_depth::{Intrinsics, head_from_face_depth, pick_largest_face};
use super::{HeadTracker, Pose};

/// Kinect v1 RGB and depth share the same 640×480 grid (different sensors,
/// same factory framing); the linear rescale in `head_from_face_depth`
/// becomes identity.
const RGB_W: u32 = DEPTH_WIDTH;
const RGB_H: u32 = DEPTH_HEIGHT;

pub struct KinectV1Backend {
    // Drop order (declaration order): device first, then context.
    device: Device,
    _ctx: Context,
    intrinsics: Intrinsics,
    detector: face::Detector,
    last_faces: Vec<face::FaceDetection>,
    last_depth: Vec<f32>,
    started_at: Instant,
}

impl KinectV1Backend {
    pub fn open() -> Result<Self, Error> {
        let ctx = Context::new()?;
        let count = ctx.enumerate();
        if count <= 0 {
            return Err(Error::Freenect(freenect::Error::NoDevice));
        }
        let mut device = ctx.open(0)?;
        device.start()?;
        let detector = face::Detector::new()?;
        info!(
            n_devices = count,
            fx = FX,
            fy = FY,
            cx = CX,
            cy = CY,
            "kinect-v1: device opened (640x480 depth in mm)"
        );
        Ok(Self {
            device,
            _ctx: ctx,
            intrinsics: Intrinsics {
                fx: FX,
                fy: FY,
                cx: CX,
                cy: CY,
            },
            detector,
            last_faces: Vec::new(),
            last_depth: Vec::new(),
            started_at: Instant::now(),
        })
    }

    fn refresh_face_from_rgb(&mut self) {
        if let Some(rgb) = self.device.poll_rgb() {
            // libfreenect's RGB stream is already RGB888 at 640×480.
            self.last_faces = self.detector.detect(&rgb.data, rgb.width, rgb.height);
        }
    }

    fn frame_to_pose(&mut self, frame: &DepthFrame) -> Option<Pose> {
        debug_assert_eq!(frame.width, DEPTH_WIDTH);
        debug_assert_eq!(frame.height, DEPTH_HEIGHT);

        // Widen u16 mm into f32 once per frame; reuse the buffer to avoid
        // per-frame allocation.
        self.last_depth.clear();
        self.last_depth.reserve(frame.data.len());
        self.last_depth
            .extend(frame.data.iter().map(|&z| f32::from(z)));

        let face = pick_largest_face(&self.last_faces)?;
        let xyz = head_from_face_depth(
            face,
            RGB_W,
            RGB_H,
            &self.last_depth,
            frame.width,
            frame.height,
            &self.intrinsics,
        )?;
        Some(Pose {
            position_mm: xyz,
            timestamp_us: self.started_at.elapsed().as_micros() as u64,
            confidence: face.confidence.clamp(0.0, 1.0),
        })
    }
}

impl HeadTracker for KinectV1Backend {
    fn poll(&mut self) -> Option<Pose> {
        self.refresh_face_from_rgb();
        let frame = self.device.poll_depth()?;
        self.frame_to_pose(&frame)
    }

    fn name(&self) -> &'static str {
        "kinect-v1"
    }

    fn shutdown(&mut self) {
        if let Err(e) = self.device.stop() {
            warn!(?e, "kinect-v1: stop failed");
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("libfreenect: {0}")]
    Freenect(#[from] freenect::Error),
    #[error("face detector init: {0}")]
    Face(#[from] face::Error),
}
