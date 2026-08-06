//! Cabinet lockbar reference data shared across the workspace.
//!
//! The depth-based `detect_lockbar` scanner that used to live here was
//! retired (the anchor model + hand-fixed lines replaced it — see
//! `crates/anchor`); what remains is the metric constant and the RGB quad
//! type the demo overlays still consume.

pub const LOCKBAR_WIDTH_MM: f32 = 610.0;

#[derive(Debug, Clone, Copy)]
pub struct LockbarQuadRgb {
    pub frame_width: u32,
    pub frame_height: u32,
    /// `[top_left, top_right, bottom_right, bottom_left]`. Top has the
    /// smaller row index (image Y points down).
    pub corners: [(u32, u32); 4],
    /// Slope of the top edge in degrees. Image Y points down, so a
    /// positive value means the right endpoint sits lower in the image
    /// than the left — i.e. the camera is rolled CCW around its
    /// optical axis (or the cab itself is tilted).
    pub slope_deg: f32,
    /// Mean vertical separation between top and bottom edges in
    /// pixels. Reflects the apparent thickness of the lockbar's 70 mm
    /// depth at the current camera distance.
    pub thickness_px: u32,
    /// Inlier counts for the two fitted lines. Higher = more confident.
    pub n_inliers_top: u32,
    pub n_inliers_bottom: u32,
    /// Left/right playfield **sidebars** (the rails of the U), when the U
    /// detector could fit them, as straight segments `[near, far]` in image
    /// pixels: `near` sits at the lockbar end, `far` at the open end of the
    /// U. Together with the lockbar they bound the playfield opening — the
    /// fixed real-world reference for the parallax. `None` for detectors that
    /// only yield the front bar (e.g. the old RGB-OBB path) or when a rail
    /// could not be fit.
    pub left_rail: Option<[(u32, u32); 2]>,
    pub right_rail: Option<[(u32, u32); 2]>,
}

impl LockbarQuadRgb {
    /// Width in pixels at the top edge.
    pub fn top_width_px(&self) -> u32 {
        self.corners[1].0.saturating_sub(self.corners[0].0) + 1
    }
    /// Width in pixels at the bottom edge.
    pub fn bottom_width_px(&self) -> u32 {
        self.corners[2].0.saturating_sub(self.corners[3].0) + 1
    }
    /// Mean of top and bottom widths — single number when the consumer
    /// just wants a "size" reference.
    pub fn mean_width_px(&self) -> u32 {
        (self.top_width_px() + self.bottom_width_px()) / 2
    }
    /// Vertical mid-row of the quad (mean of top and bottom edges).
    pub fn mean_row(&self) -> u32 {
        (self.corners[0].1 + self.corners[1].1 + self.corners[2].1 + self.corners[3].1) / 4
    }
}
