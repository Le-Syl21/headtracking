//! Kinect v2 head tracker backend (libfreenect2 via the `freenect2` crate).
//!
//! The demo-validated pipeline:
//!
//! * **IR-first** (default): only the depth pipeline streams
//!   (`start_streams(false, true)`) — IR rides the same listener. The IR
//!   frame is auto-levelled and fed to BlazePose; the head point then
//!   samples the depth grid **directly** (IR and depth share the sensor and
//!   the 512×424 grid — no registration involved). Actively illuminated IR
//!   holds 30 fps in a dark game room where auto-exposed colour drops to 15.
//! * **Colour fallback**: BlazePose runs on the 1920×1080 BGRX stream and
//!   depth is sampled through libfreenect2's factory registration
//!   (`bigdepth`, depth expressed in colour pixels), deprojected with the
//!   **colour** intrinsics.
//!
//! Either way the v2 frame is mirrored vs the v1 — X is negated so
//! left/right POV travel matches across backends.

use std::time::Instant;

use tracing::{info, warn};

use freenect2::{BIGDEPTH_LEN, Context, Device, Registration};

use super::pipeline::{
    BIGDEPTH_H, BIGDEPTH_W, DEPTH_MIN_SAMPLES, HeadPixel, Intrinsics, TrackingStream,
    autolevel_gray8_raw, bgrx_to_rgb888, gray8_to_rgb888, head_pixel_from_bigdepth,
    head_pixel_from_pose_depth,
};
use super::{HeadTracker, Pose};

pub struct KinectV2Backend {
    // Drop order matters: `device` must run its destructor before `_ctx` so
    // libfreenect2's Freenect2Device shutdown still has a live Freenect2.
    // Rust drops struct fields in declaration order, so list `device` first.
    device: Device,
    registration: Option<Registration>,
    _ctx: Context,
    blaze: blazepose::BlazePose,
    stream: TrackingStream,
    ir_intr: Intrinsics,
    color_intr: Intrinsics,
    /// Latest BlazePose fix + the dimensions of the frame it was found in.
    last_pose: Option<blazepose::Pose>,
    pose_src: (u32, u32),
    /// Scratch for the registration output (colour mode only).
    bigdepth: Vec<f32>,
    /// Zeroed BGRX scratch: `Registration::apply` demands a colour buffer,
    /// but `bigdepth` is computed from depth alone — zeros are fine.
    rgb_zero: Vec<u8>,
    started_at: Instant,
    /// Declared last: released only after the device handle above closes.
    _hwlock: crate::hwlock::HwLock,
}

impl KinectV2Backend {
    /// Open the first Kinect v2 found on USB and start the streams the
    /// selected tracking mode needs.
    pub fn open(stream: TrackingStream) -> Result<Self, Error> {
        // Cross-process exclusivity (demo, cron capture, second VPX): fail
        // fast with a readable message instead of a USB-level fight.
        let hwlock = crate::hwlock::HwLock::acquire("kinect-v2").map_err(Error::Busy)?;
        let ctx = Context::new()?;
        let count = ctx.enumerate();
        if count <= 0 {
            return Err(Error::Freenect2(freenect2::Error::NoDevice));
        }
        let device = ctx.open_default()?;
        // Say which depth pipeline opened, before anything else can go wrong.
        // On the CPU one the v2 drops USB depth packets and delivers ~5 fps
        // instead of 30, and every number downstream inherits it — a report
        // that starts with "CPU" needs no further diagnosis. Compiling the GPU
        // path in is not the same as it running: a machine with no registered
        // OpenCL ICD falls back here, silently, at runtime.
        let pipeline = device.depth_pipeline();
        if pipeline == "CPU" {
            warn!(
                "kinect-v2: depth pipeline is CPU — expect dropped USB packets and \
                 ~5 fps of depth instead of 30. No usable OpenCL device (GPU driver ICD?)."
            );
        } else {
            info!(pipeline, "kinect-v2: depth pipeline");
        }
        // Colour always flows at open, whatever the target: the anchor
        // calibration phase needs RGB frames. `begin_tracking` then trades
        // colour for the IR/depth-only pipeline (halved USB load, no 8 MB
        // BGRX conversions per frame).
        device.start()?;
        let ir = device.ir_params();
        let ir_intr = Intrinsics {
            fx: ir.fx,
            fy: ir.fy,
            cx: ir.cx,
            cy: ir.cy,
        };
        let color = device.color_params();
        let color_intr = Intrinsics {
            fx: color.fx,
            fy: color.fy,
            cx: color.cx,
            cy: color.cy,
        };
        let registration = match stream {
            TrackingStream::Rgb => Some(device.registration()),
            TrackingStream::Ir => None,
        };
        let blaze = blazepose::BlazePose::new()?;
        info!(
            n_devices = count,
            ?stream,
            ir_fx = ir_intr.fx,
            color_fx = color_intr.fx,
            "kinect-v2: device opened"
        );
        Ok(Self {
            device,
            registration,
            _ctx: ctx,
            blaze,
            stream,
            ir_intr,
            color_intr,
            last_pose: None,
            pose_src: (0, 0),
            bigdepth: vec![0.0; BIGDEPTH_LEN],
            rgb_zero: vec![0u8; BIGDEPTH_W * BIGDEPTH_H * 4],
            started_at: Instant::now(),
            _hwlock: hwlock,
        })
    }

    /// Feed the newest video frame (IR or colour per mode) to BlazePose.
    fn refresh_pose(&mut self) {
        match self.stream {
            TrackingStream::Ir => {
                if let Some(ir) = self.device.poll_ir() {
                    // v2 IR is a wide-range f32 intensity; round to u16 and
                    // auto-level, or the untouched high byte is nearly black.
                    let raw: Vec<u16> = ir.data.iter().map(|&v| v as u16).collect();
                    let gray = autolevel_gray8_raw(&raw, false);
                    let rgb888 = gray8_to_rgb888(&gray);
                    match self.blaze.poll(&rgb888, ir.width, ir.height) {
                        Ok(pose) => {
                            if pose.is_some() {
                                self.pose_src = (ir.width, ir.height);
                            }
                            self.last_pose = pose;
                        }
                        Err(e) => warn!("kinect-v2: blazepose failed on IR: {e}"),
                    }
                }
            }
            TrackingStream::Rgb => {
                if let Some(rgb) = self.device.poll_rgb() {
                    let rgb888 = bgrx_to_rgb888(&rgb.data);
                    match self.blaze.poll(&rgb888, rgb.width, rgb.height) {
                        Ok(pose) => {
                            if pose.is_some() {
                                self.pose_src = (rgb.width, rgb.height);
                            }
                            self.last_pose = pose;
                        }
                        Err(e) => warn!("kinect-v2: blazepose failed on colour: {e}"),
                    }
                }
            }
        }
    }

    fn head_from_depth(&mut self, depth: &freenect2::DepthFrame) -> Option<HeadPixel> {
        let pose = self.last_pose.as_ref()?;
        let head = match self.stream {
            // Pose already lives in the depth camera's own grid (IR and
            // depth are the same sensor, pixel aligned) — exact sampling.
            TrackingStream::Ir => head_pixel_from_pose_depth(
                pose,
                self.pose_src,
                &depth.data,
                (depth.width, depth.height),
                &self.ir_intr,
                DEPTH_MIN_SAMPLES,
            ),
            TrackingStream::Rgb => {
                let reg_ok = self.registration.as_mut().is_some_and(|reg| {
                    reg.bigdepth(&self.rgb_zero, &depth.data, &mut self.bigdepth)
                });
                if reg_ok {
                    head_pixel_from_bigdepth(
                        pose,
                        &self.bigdepth,
                        &self.color_intr,
                        DEPTH_MIN_SAMPLES,
                    )
                } else {
                    // Registration unavailable: linear rescale into the raw
                    // depth grid — cross-sensor parallax uncorrected, still
                    // better than nothing.
                    head_pixel_from_pose_depth(
                        pose,
                        self.pose_src,
                        &depth.data,
                        (depth.width, depth.height),
                        &self.ir_intr,
                        DEPTH_MIN_SAMPLES,
                    )
                }
            }
        };
        // v2 frames are mirrored → negate X so left/right POV travel
        // matches the v1 (bigdepth inherits the colour framing, the IR grid
        // shares the depth sensor — the correction applies to all paths).
        head.map(|mut h| {
            h.x_mm = -h.x_mm;
            h
        })
    }
}

impl HeadTracker for KinectV2Backend {
    fn poll(&mut self) -> Option<Pose> {
        self.refresh_pose();
        let depth = self.device.poll_depth()?;
        let head = self.head_from_depth(&depth)?;
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
        "kinect-v2"
    }

    fn device_label(&self) -> String {
        match self.stream {
            TrackingStream::Ir => "Kinect v2 (IR stream)".to_string(),
            TrackingStream::Rgb => "Kinect v2 (color stream)".to_string(),
        }
    }

    fn poll_calibration_rgb(&mut self) -> Option<(u32, u32, Vec<u8>)> {
        let rgb = self.device.poll_rgb()?;
        Some((rgb.width, rgb.height, bgrx_to_rgb888(&rgb.data)))
    }

    fn begin_tracking(&mut self) {
        if self.stream == TrackingStream::Ir {
            let res = self
                .device
                .stop()
                .and_then(|()| self.device.start_streams(false, true));
            match res {
                Ok(()) => info!("kinect-v2: calibration done, restarted on IR/depth only"),
                Err(e) => warn!(?e, "kinect-v2: IR restart failed; streams may be stale"),
            }
        }
    }

    fn color_intrinsics(&self) -> Option<[f32; 4]> {
        let c = &self.color_intr;
        Some([c.fx, c.fy, c.cx, c.cy])
    }

    fn shutdown(&mut self) {
        if let Err(e) = self.device.stop() {
            warn!(?e, "kinect-v2: stop failed");
        }
    }
}

/// Errors returned by [`KinectV2Backend::open`].
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("libfreenect2: {0}")]
    Freenect2(#[from] freenect2::Error),
    #[error("blazepose init: {0}")]
    Blaze(#[from] blazepose::Error),
    #[error("{0}")]
    Busy(String),
}
