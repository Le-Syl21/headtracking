//! Map a head [`Pose`] to a delta on the VPX view-setup `(viewX, viewY, viewZ)`.
//!
//! The plugin applies the change as a *delta* relative to a baseline captured
//! on the first valid pose of a game. This way the absolute camera position
//! doesn't matter — only head movement does — and we don't disturb the
//! table's authored POV when no parallax is happening.
//!
//! Axis convention (camera mounted on the backbox, lens looking at the
//! player):
//!
//! ```text
//!   Camera frame                        VPX view (table space)
//!   +X  : rightward (player's right)    +X  : rightward
//!   +Y  : downward                       +Z  : upward
//!   +Z  : depth into the scene           -Y  : forward (away from player)
//! ```
//!
//! **Incline rotation.** A cab player does not face the screen like a desktop
//! head-tracking setup: they look DOWN at a playfield inclined by a few
//! degrees from horizontal. Physical vertical/depth head motion therefore
//! decomposes into the playfield's up/forward axes through a rotation by
//! `90° − incline` (field-validated in the demo's parallax bench). The
//! incline comes from the HOST (`VPXViewSetupDef::screenInclination`), never
//! from a plugin setting.
//!
//! **Window mode.** `VLM_WINDOW` positions the view with the same table-frame
//! `viewX/Y/Z` writes; VPX's internal player↔view conversion
//! (`ViewSetup::SetViewPosFromPlayerPosition`) applies
//! `RotateX(atan2(windowTopZOfs − windowBottomZOfs, table_length) − incline)`
//! and a constant `windowBottomZOfs` shift. For *deltas* the constant shift
//! cancels, and the table-length term isn't exposed to plugins (upstream API
//! gap, tracked in the port plan) — the remaining dominant rotation is the
//! same incline rotation used here. So one mapping serves both modes today.

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

/// How head-space deltas turn into view-space deltas.
#[derive(Debug, Clone, Copy)]
pub struct MappingParams {
    /// Playfield inclination vs horizontal, degrees — read from the host's
    /// `VPXViewSetupDef::screenInclination`.
    pub incline_deg: f32,
    /// Per-axis sign flips (camera-frame X/Y/Z) for exotic mountings.
    pub invert: [bool; 3],
}

impl Default for MappingParams {
    fn default() -> Self {
        Self {
            incline_deg: 6.5,
            invert: [false; 3],
        }
    }
}

/// Compute the view delta a renderer should apply this frame, given the
/// current pose, a captured baseline, and the mapping parameters.
pub fn pose_delta_to_view_delta(current: &Pose, baseline: &Pose, p: &MappingParams) -> ViewDelta {
    let sign = |i: usize| if p.invert[i] { -1.0f32 } else { 1.0 };
    let dx_mm = (current.position_mm[0] - baseline.position_mm[0]) * sign(0);
    let dy_mm = (current.position_mm[1] - baseline.position_mm[1]) * sign(1);
    let dz_mm = (current.position_mm[2] - baseline.position_mm[2]) * sign(2);

    // The camera faces the standing player (~vertical), but the screen is
    // the near-flat playfield. Tilt the head's vertical/depth motion by
    // (90° − incline) so the parallax feels right on the laid-flat screen.
    // X (left/right) is unaffected.
    let theta = (90.0 - p.incline_deg).to_radians();
    let (ct, st) = (theta.cos(), theta.sin());
    let dy_t = dy_mm * ct + dz_mm * st;
    let dz_t = -dy_mm * st + dz_mm * ct;

    ViewDelta {
        dx: mm_to_vpu(dx_mm),
        // Camera Y is downward; VPX Z is upward → flip.
        dz: -mm_to_vpu(dy_t),
        // Camera Z grows away from the sensor; in VPX +Y is forward (away
        // from the player) → moving closer pulls the view toward the player.
        dy: -mm_to_vpu(dz_t),
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

    /// Incline 90° collapses the rotation to identity on (y, z) — handy to
    /// test the raw axis mapping alone.
    const FLAT: MappingParams = MappingParams {
        incline_deg: 90.0,
        invert: [false; 3],
    };

    #[test]
    fn baseline_yields_zero_delta() {
        let p = pose(100.0, 200.0, 700.0);
        let d = pose_delta_to_view_delta(&p, &p, &FLAT);
        assert_eq!(d, ViewDelta::default());
    }

    #[test]
    fn rightward_motion_increases_view_x() {
        let base = pose(0.0, 0.0, 700.0);
        let cur = pose(50.0, 0.0, 700.0); // 50 mm to the right
        let d = pose_delta_to_view_delta(&cur, &base, &FLAT);
        assert!(d.dx > 0.0, "dx should be positive: {d:?}");
        assert!(d.dy.abs() < 1e-4);
        assert!(d.dz.abs() < 1e-4);
    }

    #[test]
    fn upward_motion_increases_view_z() {
        let base = pose(0.0, 0.0, 700.0);
        let cur = pose(0.0, -30.0, 700.0); // 30 mm up (camera Y is down)
        let d = pose_delta_to_view_delta(&cur, &base, &FLAT);
        assert!(d.dz > 0.0, "dz should be positive when head goes up: {d:?}");
    }

    #[test]
    fn approaching_screen_pulls_view_forward() {
        let base = pose(0.0, 0.0, 700.0);
        let cur = pose(0.0, 0.0, 600.0); // 100 mm closer to the camera
        let d = pose_delta_to_view_delta(&cur, &base, &FLAT);
        assert!(
            d.dy > 0.0,
            "dy should be positive when head approaches: {d:?}"
        );
    }

    #[test]
    fn incline_rotation_mixes_vertical_into_depth() {
        // At a realistic 6.5° incline, θ = 83.5°: pure vertical head motion
        // must land mostly on the depth axis (dy), with a small dz share —
        // the "player looks down at the playfield" decomposition.
        let p = MappingParams {
            incline_deg: 6.5,
            invert: [false; 3],
        };
        let base = pose(0.0, 0.0, 700.0);
        let cur = pose(0.0, -100.0, 700.0); // 100 mm straight up
        let d = pose_delta_to_view_delta(&cur, &base, &p);
        assert!(
            d.dy.abs() > d.dz.abs(),
            "vertical motion should mostly become forward motion on an \
             inclined playfield: {d:?}"
        );
        assert!(
            d.dz > 0.0,
            "and the residual vertical share stays up: {d:?}"
        );
    }

    #[test]
    fn invert_flags_flip_each_axis() {
        let base = pose(0.0, 0.0, 700.0);
        let cur = pose(50.0, 0.0, 700.0);
        let inv = MappingParams {
            incline_deg: 90.0,
            invert: [true, false, false],
        };
        let d = pose_delta_to_view_delta(&cur, &base, &inv);
        assert!(d.dx < 0.0, "inverted X must flip the sign: {d:?}");
    }
}
