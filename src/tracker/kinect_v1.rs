//! Kinect v1 head tracker backend (libfreenect via the `freenect` crate).
//!
//! Demo-validated pipeline. The v1 shares ONE USB isochronous endpoint
//! between the colour and IR cameras, so exactly one of them streams at a
//! time:
//!
//! * **IR-first** (default): `set_video_stream(Ir)` — actively illuminated,
//!   full rate in a dark game room. The 8-bit IR frame feeds BlazePose
//!   directly; IR and depth share the sensor and the 640×480 grid, so the
//!   head point samples the native `u16` depth exactly.
//! * **Colour**: BlazePose on the RGB888 stream; the head point rescales
//!   into the depth grid (same 640×480 nominal framing).
//!
//! Depth sampling widens `u16` per-sample inside the 17×17 window instead
//! of paying a full-frame `u16→f32` copy per poll, and deprojection uses
//! the depth/IR intrinsics the samples live in.

use std::time::Instant;

use tracing::{info, warn};

use freenect::{CX, CY, Context, Device, FX, FY, VideoStream};

use super::pipeline::{
    DEPTH_MIN_SAMPLES, HeadPixel, Intrinsics, TrackingStream, gray8_to_rgb888,
    head_pixel_from_pose_depth,
};
use super::{HeadTracker, Pose};

pub struct KinectV1Backend {
    // Drop order (declaration order): device first, then context.
    device: Device,
    _ctx: Context,
    blaze: blazepose::BlazePose,
    stream: TrackingStream,
    depth_intr: Intrinsics,
    last_pose: Option<blazepose::Pose>,
    pose_src: (u32, u32),
    started_at: Instant,
    /// Declared last: released only after the device handle above closes.
    _hwlock: crate::hwlock::HwLock,
}

impl KinectV1Backend {
    pub fn open(ir: bool) -> Result<Self, Error> {
        // Cross-process exclusivity (demo, cron capture, second VPX): fail
        // fast with a readable message instead of a USB-level fight.
        let hwlock = crate::hwlock::HwLock::acquire("kinect-v1").map_err(Error::Busy)?;
        let ctx = Context::new()?;
        let count = ctx.enumerate();
        if count <= 0 {
            return Err(Error::Freenect(freenect::Error::NoDevice));
        }
        let mut device = ctx.open(0)?;
        device.start_streams(true, true)?;
        let stream = if ir {
            device.set_video_stream(VideoStream::Ir)?;
            TrackingStream::Ir
        } else {
            TrackingStream::Rgb
        };
        let blaze = blazepose::BlazePose::new()?;
        info!(
            n_devices = count,
            ?stream,
            fx = FX,
            fy = FY,
            "kinect-v1: device opened (640x480 depth in mm)"
        );
        Ok(Self {
            device,
            _ctx: ctx,
            blaze,
            stream,
            depth_intr: Intrinsics {
                fx: FX,
                fy: FY,
                cx: CX,
                cy: CY,
            },
            last_pose: None,
            pose_src: (0, 0),
            started_at: Instant::now(),
            _hwlock: hwlock,
        })
    }

    fn refresh_pose(&mut self) {
        match self.stream {
            TrackingStream::Ir => {
                if let Some(ir) = self.device.poll_ir_frame() {
                    // v1 IR is native 8-bit — no levelling needed, just the
                    // 3-channel expansion BlazePose expects.
                    let rgb888 = gray8_to_rgb888(&ir.data);
                    match self.blaze.poll(&rgb888, ir.width, ir.height) {
                        Ok(pose) => {
                            if pose.is_some() {
                                self.pose_src = (ir.width, ir.height);
                            }
                            self.last_pose = pose;
                        }
                        Err(e) => warn!("kinect-v1: blazepose failed on IR: {e}"),
                    }
                }
            }
            TrackingStream::Rgb => {
                if let Some(rgb) = self.device.poll_rgb() {
                    // libfreenect's colour stream is already RGB888.
                    match self.blaze.poll(&rgb.data, rgb.width, rgb.height) {
                        Ok(pose) => {
                            if pose.is_some() {
                                self.pose_src = (rgb.width, rgb.height);
                            }
                            self.last_pose = pose;
                        }
                        Err(e) => warn!("kinect-v1: blazepose failed on colour: {e}"),
                    }
                }
            }
        }
    }
}

impl HeadTracker for KinectV1Backend {
    fn poll(&mut self) -> Option<Pose> {
        self.refresh_pose();
        let depth = self.device.poll_depth()?;
        let pose = self.last_pose.as_ref()?;
        let head: HeadPixel = head_pixel_from_pose_depth(
            pose,
            self.pose_src,
            &depth.data,
            (depth.width, depth.height),
            &self.depth_intr,
            DEPTH_MIN_SAMPLES,
        )?;
        Some(Pose {
            position_mm: [head.x_mm, head.y_mm, head.depth_mm],
            timestamp_us: self.started_at.elapsed().as_micros() as u64,
            confidence: self
                .last_pose
                .as_ref()
                .map_or(0.0, |p| p.presence.clamp(0.0, 1.0)),
        })
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
    #[error("blazepose init: {0}")]
    Blaze(#[from] blazepose::Error),
    #[error("{0}")]
    Busy(String),
}
