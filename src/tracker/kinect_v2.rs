//! Kinect v2 head tracker backend (libfreenect2 via the `freenect2` crate).
//!
//! The demo-validated pipeline, and the only one: only the depth pipeline
//! streams (`start_streams(false, true)`) — IR rides the same listener. The
//! IR frame is auto-levelled and fed to BlazePose; the head point then samples
//! the depth grid **directly**, because IR and depth are the same sensor on
//! the same 512×424 grid and no registration is involved. Actively illuminated
//! IR holds 30 fps in a dark game room where auto-exposed colour drops to 15.
//!
//! There used to be a colour path behind a VPX setting, running BlazePose on
//! the 1920×1080 BGRX stream and sampling depth through libfreenect2's factory
//! registration. It is gone. A sensor that carries its own illuminator should
//! use it; offering the worse stream as a choice only invited someone to pick
//! it, and it kept a whole second geometry — colour intrinsics, a registration
//! object, an 8.3 MB frame buffer, a windowed depth projection — alive to
//! serve that choice. Webcams have no IR and use colour because they have
//! nothing else, which is a capability, not a preference.
//!
//! The v2 frame is mirrored vs the v1 — X is negated so left/right POV travel
//! matches across backends.
//!
//! **Anchor calibration runs on the same IR frame**, so the cabinet is located
//! in the IR image and the geometry lands in the depth camera's own
//! coordinates — the same lens again. Colour never starts at all: the session
//! opens on IR/depth and stays there. Before this, calibration asked for
//! colour frames while `open()` called `Device::start()` — which is
//! `start_streams(false, true)` and does not start colour — so the anchor
//! phase on a v2 waited out its timeout on a stream that was never running,
//! every time.

use std::time::Instant;

use tracing::{info, warn};

use freenect2::{Context, DepthFrame, Device, IrFrame};

use super::pipeline::{
    DEPTH_MIN_SAMPLES, HeadPixel, Intrinsics, autolevel_gray8_raw, gray8_to_rgb888,
    head_pixel_from_pose_depth,
};
use super::{HeadTracker, Pose};

pub struct KinectV2Backend {
    // Drop order matters: `device` must run its destructor before `_ctx` so
    // libfreenect2's Freenect2Device shutdown still has a live Freenect2.
    // Rust drops struct fields in declaration order, so list `device` first.
    device: Device,
    _ctx: Context,
    blaze: blazepose::BlazePose,
    ir_intr: Intrinsics,
    /// Latest BlazePose fix + the dimensions of the frame it was found in.
    last_pose: Option<blazepose::Pose>,
    pose_src: (u32, u32),
    /// Reused frame buffers. The v2 colour frame is 8.3 MB and depth/IR
    /// 868 KB each: allocating them per poll cost more than the copy.
    depth_buf: DepthFrame,
    ir_buf: IrFrame,
    started_at: Instant,
    /// Declared last: released only after the device handle above closes.
    _hwlock: crate::hwlock::HwLock,
}

impl KinectV2Backend {
    /// Open the first Kinect v2 found on USB and start IR + depth.
    pub fn open() -> Result<Self, Error> {
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
        // IR + depth for the whole session, colour never. Half the USB load,
        // no 8 MB BGRX conversion per frame, no stop/restart between
        // calibration and tracking, and the cabinet geometry lands in the
        // depth camera's own frame -- IR and depth share the lens on this
        // sensor -- so the per-head depth lookup needs no colour-to-depth
        // registration. `Device::start()` is exactly `start_streams(false,
        // true)`.
        device.start()?;
        let ir = device.ir_params();
        let ir_intr = Intrinsics {
            fx: ir.fx,
            fy: ir.fy,
            cx: ir.cx,
            cy: ir.cy,
        };
        let blaze = blazepose::BlazePose::new()?;
        info!(
            n_devices = count,
            ir_fx = ir_intr.fx,
            "kinect-v2: device opened (IR + depth)"
        );
        Ok(Self {
            device,
            _ctx: ctx,
            blaze,
            ir_intr,
            last_pose: None,
            pose_src: (0, 0),
            depth_buf: DepthFrame::default(),
            ir_buf: IrFrame::default(),
            started_at: Instant::now(),
            _hwlock: hwlock,
        })
    }

    /// Feed the newest IR frame to BlazePose.
    fn refresh_pose(&mut self) {
        if self.device.poll_ir_into(&mut self.ir_buf) {
            // v2 IR is a wide-range f32 intensity; round to u16 and
            // auto-level, or the untouched high byte is nearly black.
            let raw: Vec<u16> = self.ir_buf.data.iter().map(|&v| v as u16).collect();
            let gray = autolevel_gray8_raw(&raw, false);
            let rgb888 = gray8_to_rgb888(&gray);
            let (w, h) = (self.ir_buf.width, self.ir_buf.height);
            match self
                .blaze
                .poll(&rgb888, w, h, blazepose::PixelLayout::Rgb888)
            {
                Ok(pose) => {
                    if pose.is_some() {
                        self.pose_src = (w, h);
                    }
                    self.last_pose = pose;
                }
                Err(e) => warn!("kinect-v2: blazepose failed on IR: {e}"),
            }
        }
    }

    fn head_from_depth(&mut self) -> Option<HeadPixel> {
        let pose = self.last_pose.as_ref()?;
        let depth = &self.depth_buf;
        // The pose already lives in the depth camera's own grid — IR and
        // depth are the same sensor, pixel aligned — so this is an exact
        // sampling with no registration in the way.
        let head = head_pixel_from_pose_depth(
            pose,
            self.pose_src,
            &depth.data,
            (depth.width, depth.height),
            &self.ir_intr,
            DEPTH_MIN_SAMPLES,
        );
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
        if !self.device.poll_depth_into(&mut self.depth_buf) {
            return None;
        }
        let head = self.head_from_depth()?;
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
        "Kinect v2 (IR stream)".to_string()
    }

    fn poll_calibration_frame(&mut self) -> Option<(u32, u32, Vec<u8>)> {
        // Same auto-levelling that produced the `_irview` frames the anchor
        // model was trained on, so it sees the distribution it learned rather
        // than the near-black native IR.
        if !self.device.poll_ir_into(&mut self.ir_buf) {
            return None;
        }
        let raw: Vec<u16> = self.ir_buf.data.iter().map(|&v| v as u16).collect();
        let gray = autolevel_gray8_raw(&raw, false);
        Some((
            self.ir_buf.width,
            self.ir_buf.height,
            gray8_to_rgb888(&gray),
        ))
    }

    fn calibration_intrinsics(&self) -> Option<[f32; 4]> {
        let c = &self.ir_intr;
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
