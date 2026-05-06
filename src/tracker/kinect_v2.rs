//! Kinect v2 head tracker backend (libfreenect2 via the `freenect2` crate).
//!
//! Algorithm (MVP placeholder):
//!
//! 1. Pull the latest depth frame (512×424 f32 mm).
//! 2. Locate the closest valid pixel inside the play range [500, 2500] mm.
//!    Inside a pincab frustum the player's face is the closest object.
//! 3. Take a 50×50 px window around that pixel and average the valid depths.
//! 4. Deproject the window centroid to (x, y, z) in the device frame using
//!    the IR camera intrinsics returned by libfreenect2.
//!
//! This is intentionally simple — we'll swap in a proper connected-component
//! head detector once we can actually point the device at a player and
//! compare ground truth.

use std::time::Instant;

use tracing::{info, warn};

use freenect2::{Context, DepthFrame, Device, IrCameraParams};

use super::{HeadTracker, Pose};

const DEPTH_MIN_MM: f32 = 500.0;
const DEPTH_MAX_MM: f32 = 2_500.0;
const WINDOW_HALF: i32 = 25;
const MIN_VALID_PIXELS: u32 = 100;

pub struct KinectV2Backend {
    // Drop order matters: `device` must run its destructor before `_ctx` so
    // libfreenect2's Freenect2Device shutdown still has a live Freenect2.
    // Rust drops struct fields in declaration order, so list `device` first.
    device: Device,
    _ctx: Context,
    intrinsics: IrCameraParams,
    started_at: Instant,
}

impl KinectV2Backend {
    /// Open the first Kinect v2 found on USB and start the depth stream.
    pub fn open() -> Result<Self, freenect2::Error> {
        let ctx = Context::new()?;
        let count = ctx.enumerate();
        if count <= 0 {
            return Err(freenect2::Error::NoDevice);
        }
        let device = ctx.open_default()?;
        device.start()?;
        let intrinsics = device.ir_params();
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
            started_at: Instant::now(),
        })
    }

    fn frame_to_pose(&self, frame: &DepthFrame) -> Option<Pose> {
        let w = frame.width as i32;
        let h = frame.height as i32;
        if w <= 0 || h <= 0 {
            return None;
        }

        // Pass 1: find the closest valid pixel in the play range.
        let valid_range = DEPTH_MIN_MM..=DEPTH_MAX_MM;
        let mut min_z = f32::INFINITY;
        let mut min_idx: i32 = -1;
        for (i, &z) in frame.data.iter().enumerate() {
            if !valid_range.contains(&z) {
                continue;
            }
            if z < min_z {
                min_z = z;
                min_idx = i as i32;
            }
        }
        if min_idx < 0 {
            return None;
        }
        let cu = min_idx % w;
        let cv = min_idx / w;

        // Pass 2: average the valid depths in a 2N+1 square around (cu, cv),
        // filtered to the slab [min_z, min_z + 150 mm] so we don't bleed onto
        // farther background.
        let z_max = min_z + 150.0;
        let mut sum_x = 0.0_f64;
        let mut sum_y = 0.0_f64;
        let mut sum_z = 0.0_f64;
        let mut count: u32 = 0;
        let u0 = (cu - WINDOW_HALF).max(0);
        let u1 = (cu + WINDOW_HALF).min(w - 1);
        let v0 = (cv - WINDOW_HALF).max(0);
        let v1 = (cv + WINDOW_HALF).min(h - 1);
        for v in v0..=v1 {
            let row = (v * w) as usize;
            for u in u0..=u1 {
                let z = frame.data[row + u as usize];
                if z < DEPTH_MIN_MM || z > z_max {
                    continue;
                }
                let zf = f64::from(z);
                let x =
                    f64::from(u as f32 - self.intrinsics.cx) * zf / f64::from(self.intrinsics.fx);
                let y =
                    f64::from(v as f32 - self.intrinsics.cy) * zf / f64::from(self.intrinsics.fy);
                sum_x += x;
                sum_y += y;
                sum_z += zf;
                count += 1;
            }
        }

        if count < MIN_VALID_PIXELS {
            return None;
        }
        let n = f64::from(count);
        let timestamp_us = self.started_at.elapsed().as_micros() as u64;
        // Confidence: ratio of valid pixels in the window vs. its total area.
        let area = ((u1 - u0 + 1) * (v1 - v0 + 1)) as f32;
        let confidence = (count as f32 / area).clamp(0.0, 1.0);
        Some(Pose {
            position_mm: [(sum_x / n) as f32, (sum_y / n) as f32, (sum_z / n) as f32],
            timestamp_us,
            confidence,
        })
    }
}

impl HeadTracker for KinectV2Backend {
    fn poll(&mut self) -> Option<Pose> {
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

// Unit tests for the blob algorithm will land once it's extracted as a free
// function on (`&IrCameraParams`, `&DepthFrame`) — at that point we no longer
// need a fake `Device` to test the geometry.
