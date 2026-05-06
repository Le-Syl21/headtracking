//! Kinect v2 head tracker backend (libfreenect2 via the `freenect2` crate).
//!
//! Algorithm:
//!
//! 1. Poll the colour stream (1920×1080 BGRX) and run YuNet face detection
//!    on each new frame. The bbox of the largest detected face is cached.
//! 2. Poll the depth stream (512×424 f32 mm). Map the cached face bbox
//!    from RGB pixel coords into depth pixel coords by linear rescale,
//!    sample valid depth pixels inside the bbox, take the median (robust
//!    to outliers).
//! 3. Deproject the bbox centre through the IR camera intrinsics returned
//!    by libfreenect2 to produce a (x, y, z) pose in millimetres.
//!
//! No face detected this frame ⇒ no pose. The earlier "closest valid
//! pixel" heuristic is gone — it tripped on hands, lockbar edges, and
//! stray noise pixels in the play frustum.

use std::time::Instant;

use tracing::{info, warn};

use freenect2::{Context, DepthFrame, Device};

use super::face_depth::{Intrinsics, bgrx_to_rgb888, head_from_face_depth, pick_largest_face};
use super::{HeadTracker, Pose};

/// Kinect v2 RGB sensor resolution. The colour stream we receive from
/// libfreenect2 is always 1920×1080 BGRX.
const RGB_W: u32 = 1920;
const RGB_H: u32 = 1080;

pub struct KinectV2Backend {
    // Drop order matters: `device` must run its destructor before `_ctx` so
    // libfreenect2's Freenect2Device shutdown still has a live Freenect2.
    // Rust drops struct fields in declaration order, so list `device` first.
    device: Device,
    _ctx: Context,
    intrinsics: Intrinsics,
    detector: face::Detector,
    last_faces: Vec<face::FaceDetection>,
    started_at: Instant,
}

impl KinectV2Backend {
    /// Open the first Kinect v2 found on USB and start the depth + colour
    /// streams. Returns an error if no device is connected, libfreenect2
    /// fails to start, or the YuNet ONNX model can't be loaded into tract.
    pub fn open() -> Result<Self, Error> {
        let ctx = Context::new()?;
        let count = ctx.enumerate();
        if count <= 0 {
            return Err(Error::Freenect2(freenect2::Error::NoDevice));
        }
        let device = ctx.open_default()?;
        device.start()?;
        let ir = device.ir_params();
        let intrinsics = Intrinsics {
            fx: ir.fx,
            fy: ir.fy,
            cx: ir.cx,
            cy: ir.cy,
        };
        let detector = face::Detector::new()?;
        info!(
            n_devices = count,
            fx = intrinsics.fx,
            fy = intrinsics.fy,
            cx = intrinsics.cx,
            cy = intrinsics.cy,
            "kinect-v2: device opened"
        );
        Ok(Self {
            device,
            _ctx: ctx,
            intrinsics,
            detector,
            last_faces: Vec::new(),
            started_at: Instant::now(),
        })
    }

    fn refresh_face_from_rgb(&mut self) {
        // poll_rgb is non-blocking; only re-run YuNet if a fresh frame
        // arrived. If not, keep the previous detections.
        if let Some(rgb) = self.device.poll_rgb() {
            let rgb888 = bgrx_to_rgb888(&rgb.data);
            self.last_faces = self.detector.detect(&rgb888, rgb.width, rgb.height);
        }
    }

    fn frame_to_pose(&self, frame: &DepthFrame) -> Option<Pose> {
        let face = pick_largest_face(&self.last_faces)?;
        let xyz = head_from_face_depth(
            face,
            RGB_W,
            RGB_H,
            &frame.data,
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

impl HeadTracker for KinectV2Backend {
    fn poll(&mut self) -> Option<Pose> {
        self.refresh_face_from_rgb();
        let frame = self.device.poll_depth()?;
        self.frame_to_pose(&frame)
    }

    fn name(&self) -> &'static str {
        "kinect-v2"
    }

    fn shutdown(&mut self) {
        if let Err(e) = self.device.stop() {
            warn!(?e, "kinect-v2: stop failed");
        }
    }
}

/// Errors returned by [`KinectV2Backend::open`]. Consolidates the libfreenect2
/// failure modes with the YuNet model load path so the session thread has
/// a single error type to surface in logs.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("libfreenect2: {0}")]
    Freenect2(#[from] freenect2::Error),
    #[error("face detector init: {0}")]
    Face(#[from] face::Error),
}
