//! Map a head [`Pose`] to a delta on the VPX view-setup `(viewX, viewY, viewZ)`.
//!
//! The MVP applies the change as a *delta* relative to a baseline captured on
//! the first valid pose of a game. This way the absolute Kinect position
//! doesn't matter — only head movement does — and we don't disturb the
//! table's authored POV when no parallax is happening.
//!
//! Axis convention (Kinect mounted on the backbox, lens looking at the player):
//!
//! ```text
//!   Kinect frame                        VPX view (Camera mode)
//!   +X  : rightward (player's right)    +X  : rightward
//!   +Y  : downward                       +Z  : upward
//!   +Z  : depth into the scene           -Y  : forward (away from player)
//! ```
//!
//! These mappings are first-pass — we'll calibrate signs and per-axis scale
//! once we can compare on a pincab.

use crate::camera::units::mm_to_vpu;
use crate::tracker::Pose;

/// View-space deltas in VPU, in `(dx, dy, dz)` order — direct mapping to
/// `VPXViewSetupDef::{viewX, viewY, viewZ}`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ViewDelta {
    pub dx: f32,
    pub dy: f32,
    pub dz: f32,
}

/// Compute the view delta a renderer should apply this frame, given the
/// current pose and a captured baseline.
pub fn pose_delta_to_view_delta(current: &Pose, baseline: &Pose) -> ViewDelta {
    let dx_mm = current.position_mm[0] - baseline.position_mm[0];
    let dy_mm = current.position_mm[1] - baseline.position_mm[1];
    let dz_mm = current.position_mm[2] - baseline.position_mm[2];
    ViewDelta {
        dx: mm_to_vpu(dx_mm),
        // Kinect Y is downward; VPX Z is upward → flip.
        dz: -mm_to_vpu(dy_mm),
        // Kinect Z grows away from the sensor; in VPX Camera mode +Y is
        // forward (away from the player) → user moving closer = view moves
        // forward, which means dy negative when dz_mm is negative.
        dy: -mm_to_vpu(dz_mm),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose(x: f32, y: f32, z: f32) -> Pose {
        Pose {
            position_mm: [x, y, z],
            timestamp_us: 0,
            confidence: 1.0,
        }
    }

    #[test]
    fn baseline_yields_zero_delta() {
        let p = pose(100.0, 200.0, 700.0);
        let d = pose_delta_to_view_delta(&p, &p);
        assert_eq!(d, ViewDelta::default());
    }

    #[test]
    fn rightward_motion_increases_view_x() {
        let base = pose(0.0, 0.0, 700.0);
        let cur = pose(50.0, 0.0, 700.0); // 50 mm to the right
        let d = pose_delta_to_view_delta(&cur, &base);
        assert!(d.dx > 0.0, "dx should be positive: {d:?}");
        assert_eq!(d.dy, 0.0);
        assert_eq!(d.dz, 0.0);
    }

    #[test]
    fn upward_motion_increases_view_z() {
        let base = pose(0.0, 0.0, 700.0);
        let cur = pose(0.0, -30.0, 700.0); // 30 mm up (Kinect Y is down)
        let d = pose_delta_to_view_delta(&cur, &base);
        assert!(d.dz > 0.0, "dz should be positive when head goes up: {d:?}");
    }

    #[test]
    fn approaching_screen_decreases_view_y() {
        let base = pose(0.0, 0.0, 700.0);
        let cur = pose(0.0, 0.0, 600.0); // 100 mm closer to the camera
        let d = pose_delta_to_view_delta(&cur, &base);
        assert!(
            d.dy > 0.0,
            "dy should be positive when head approaches: {d:?}"
        );
    }
}
