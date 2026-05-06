//! Kinect v1 head tracker backend (libfreenect via the `freenect` crate).
//!
//! Same blob-centroid algorithm as the v2 backend, adapted for v1's u16
//! depth frames. libfreenect doesn't expose factory intrinsics, so we use
//! the published Microsoft nominal values; per-cab calibration can refine
//! them later via `tools/ht-calibrate`.
//!
//! Untested on hardware (Sylvain only has a v2 sensor). Compile-time
//! parity is what we check in CI; hardware runs come later.

use std::time::Instant;

use tracing::{info, warn};

use freenect::{CX, CY, Context, DEPTH_HEIGHT, DEPTH_WIDTH, DepthFrame, Device, FX, FY};

use super::{HeadTracker, Pose};

const DEPTH_MIN_MM: u16 = 500;
const DEPTH_MAX_MM: u16 = 2_500;
const WINDOW_HALF: i32 = 25;
const MIN_VALID_PIXELS: u32 = 100;

pub struct KinectV1Backend {
    // Drop order (declaration order): device first, then context.
    device: Device,
    _ctx: Context,
    started_at: Instant,
}

impl KinectV1Backend {
    pub fn open() -> Result<Self, freenect::Error> {
        let ctx = Context::new()?;
        let count = ctx.enumerate();
        if count <= 0 {
            return Err(freenect::Error::NoDevice);
        }
        let mut device = ctx.open(0)?;
        device.start()?;
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
            started_at: Instant::now(),
        })
    }

    fn frame_to_pose(&self, frame: &DepthFrame) -> Option<Pose> {
        debug_assert_eq!(frame.width, DEPTH_WIDTH);
        debug_assert_eq!(frame.height, DEPTH_HEIGHT);
        let w = DEPTH_WIDTH as i32;
        let h = DEPTH_HEIGHT as i32;

        // Pass 1: closest valid pixel in the play range.
        let valid_range = DEPTH_MIN_MM..=DEPTH_MAX_MM;
        let mut min_z: u16 = u16::MAX;
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

        // Pass 2: average valid pixels in a 50x50 window inside a 150 mm
        // depth slab around the closest sample.
        let z_max = min_z.saturating_add(150);
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
                let x = f64::from(u as f32 - CX) * zf / f64::from(FX);
                let y = f64::from(v as f32 - CY) * zf / f64::from(FY);
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
        let area = ((u1 - u0 + 1) * (v1 - v0 + 1)) as f32;
        let confidence = (count as f32 / area).clamp(0.0, 1.0);
        Some(Pose {
            position_mm: [(sum_x / n) as f32, (sum_y / n) as f32, (sum_z / n) as f32],
            timestamp_us,
            confidence,
        })
    }
}

impl HeadTracker for KinectV1Backend {
    fn poll(&mut self) -> Option<Pose> {
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
