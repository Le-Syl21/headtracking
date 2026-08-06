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

/// VPX view layout mode (mirror of `ViewLayoutMode` in `ViewSetup.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewMode {
    Legacy,
    Camera,
    /// The cab / head-tracking mode: the screen is a fixed window into the
    /// cabinet, `viewX/Y/Z` is the PLAYER's eye relative to the screen's
    /// bottom centre, and VPX derives an oblique projection — the fish-tank
    /// effect happens INSIDE the table instead of the table sliding around.
    Window,
}

impl ViewMode {
    #[must_use]
    pub fn from_i32(v: i32) -> Self {
        match v {
            2 => Self::Window,
            1 => Self::Camera,
            _ => Self::Legacy,
        }
    }
}

/// How head-space deltas turn into view-space deltas.
///
/// Deliberately inclination-free: VPX owns the screen geometry (it builds
/// the Window-mode oblique projection itself), so the plugin's whole job
/// is relabeling camera axes into the frame VPX expects. Inclined-screen
/// cabs need VPX's internal player→view converter exposed to plugins
/// (upstream patch candidate, tracked in the port plan) — no trigonometry
/// belongs here.
#[derive(Debug, Clone, Copy)]
pub struct MappingParams {
    /// Per-axis sign flips (camera-frame X/Y/Z) for exotic mountings.
    pub invert: [bool; 3],
    /// Active view layout mode — read from `VPXViewSetupDef::viewMode`.
    pub mode: ViewMode,
}

impl Default for MappingParams {
    fn default() -> Self {
        Self {
            invert: [false; 3],
            mode: ViewMode::Camera,
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

    match p.mode {
        // In Window mode `viewX/Y/Z` is the player's EYE in the screen
        // frame: X lateral, Y away from the screen (toward the standing
        // player), Z up — a pure relabeling of the camera axes. VPX builds
        // the oblique projection from there.
        ViewMode::Window => ViewDelta {
            dx: mm_to_vpu(dx_mm),
            // The eye sits on the NEGATIVE-Y side of the screen plane
            // (field-verified: ScreenPlayerY is negative in VPX configs) —
            // stepping back from the cab pushes viewY further negative.
            dy: mm_to_vpu(-dz_mm),
            dz: mm_to_vpu(-dy_mm), // camera Y is down; player Z is up
        },
        // Camera/Legacy: `viewX/Y/Z` is a flying camera in table space —
        // the degraded path (the plugin nudges the user toward Window).
        ViewMode::Legacy | ViewMode::Camera => ViewDelta {
            dx: mm_to_vpu(dx_mm),
            // Camera Y is downward; VPX Z is upward → flip.
            dz: -mm_to_vpu(dy_mm),
            // Camera Z grows away from the sensor; +Y is forward (away
            // from the player) → moving closer pulls the view back.
            dy: -mm_to_vpu(dz_mm),
        },
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

    const FLAT: MappingParams = MappingParams {
        invert: [false; 3],
        mode: ViewMode::Camera,
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
    fn window_mode_maps_eye_axes() {
        // The eye lives on the negative-Y side of the screen: stepping BACK
        // from the cab pushes viewY further negative (-dy), and standing
        // taller must raise it (+dz) — the whole point of the Window frame.
        let p = MappingParams {
            invert: [false; 3],
            mode: ViewMode::Window,
        };
        let base = pose(0.0, 0.0, 700.0);
        let back = pose(0.0, 0.0, 800.0); // 100 mm away from the camera/cab
        let d = pose_delta_to_view_delta(&back, &base, &p);
        assert!(
            d.dy < 0.0,
            "stepping back must push viewY further negative: {d:?}"
        );
        let up = pose(0.0, -50.0, 700.0); // camera Y is down
        let d = pose_delta_to_view_delta(&up, &base, &p);
        assert!(d.dz > 0.0, "standing taller must raise the eye: {d:?}");
        assert!(d.dy.abs() < 1e-4, "pure height change: {d:?}");
    }

    #[test]
    fn invert_flags_flip_each_axis() {
        let base = pose(0.0, 0.0, 700.0);
        let cur = pose(50.0, 0.0, 700.0);
        let inv = MappingParams {
            invert: [true, false, false],
            mode: ViewMode::Camera,
        };
        let d = pose_delta_to_view_delta(&cur, &base, &inv);
        assert!(d.dx < 0.0, "inverted X must flip the sign: {d:?}");
    }
}
