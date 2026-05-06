//! Detect the cabinet lockbar from a depth frame.
//!
//! Geometric assumption: the camera sits on top of the backbox and looks
//! down/forward, so the lockbar — a horizontal metallic bar spanning the
//! full cabinet width — is always somewhere in the bottom rows of the
//! depth image, at roughly constant distance from the sensor. We scan
//! the bottom slice of the frame for the longest contiguous row of
//! near-constant valid depth, then return its row index, horizontal
//! extent, and mean depth.
//!
//! Knowing the lockbar's geometric width (configured per cab in
//! `calibration.toml`) the caller turns this observation into:
//! * a horizon reference (`row` → camera pitch),
//! * a focal-length anchor (`width_px` + known `width_mm` → `fx`),
//! * a metric distance (`mean_depth_mm`).

#[derive(Debug, Clone, Copy)]
pub struct LockbarObservation {
    pub frame_width: u32,
    pub frame_height: u32,
    /// Pixel row where the lockbar appears.
    pub row: u32,
    /// Left/right pixel columns of the detected bar (inclusive).
    pub left_col: u32,
    pub right_col: u32,
    /// Mean depth across the bar in mm.
    pub mean_depth_mm: f32,
    /// Standard deviation of depth across the bar (lower = flatter).
    pub depth_stddev_mm: f32,
    /// Number of valid pixels averaged.
    pub valid_pixels: u32,
}

impl LockbarObservation {
    pub fn width_px(&self) -> u32 {
        self.right_col - self.left_col + 1
    }
}

/// Tuneable parameters for [`detect_lockbar`].
#[derive(Debug, Clone, Copy)]
pub struct LockbarParams {
    /// Start the row search at `bottom_fraction * height`. `0.7` means we
    /// only look in the bottom 30% of the frame.
    pub bottom_fraction: f32,
    /// Plausible depth range for the lockbar — typically `0.3..1.5 m`.
    pub depth_min_mm: f32,
    pub depth_max_mm: f32,
    /// Minimum candidate run length, as a fraction of frame width.
    pub min_width_fraction: f32,
    /// Reject candidate rows whose depth std-dev exceeds this.
    pub max_stddev_mm: f32,
}

impl Default for LockbarParams {
    fn default() -> Self {
        Self {
            bottom_fraction: 0.65,
            depth_min_mm: 300.0,
            depth_max_mm: 1_500.0,
            min_width_fraction: 0.45,
            max_stddev_mm: 60.0,
        }
    }
}

/// Walk the bottom rows of the depth frame and return the longest run of
/// near-constant valid depth. `data` is row-major depth in mm; `0.0` (or
/// values outside the play range) are treated as invalid pixels.
pub fn detect_lockbar(
    data: &[f32],
    width: u32,
    height: u32,
    params: &LockbarParams,
) -> Option<LockbarObservation> {
    if width == 0 || height == 0 || data.len() != (width as usize) * (height as usize) {
        return None;
    }
    let w = width as usize;
    let h = height as usize;
    let start_row = ((params.bottom_fraction * height as f32) as usize).min(h.saturating_sub(1));
    let min_run_px = ((params.min_width_fraction * width as f32) as usize).max(1);

    let mut best: Option<LockbarObservation> = None;

    for row_idx in start_row..h {
        let row = &data[row_idx * w..(row_idx + 1) * w];
        if let Some(run) = longest_valid_run(row, params, min_run_px) {
            let stddev = run.stddev();
            if stddev > params.max_stddev_mm as f64 {
                continue;
            }
            let candidate = LockbarObservation {
                frame_width: width,
                frame_height: height,
                row: row_idx as u32,
                left_col: run.left as u32,
                right_col: run.right as u32,
                mean_depth_mm: run.mean() as f32,
                depth_stddev_mm: stddev as f32,
                valid_pixels: run.count as u32,
            };
            best = Some(match best {
                Some(prev) if prev.width_px() >= candidate.width_px() => prev,
                _ => candidate,
            });
        }
    }

    best
}

/// Internal: descriptor of the longest valid depth run on a row.
#[derive(Debug, Clone, Copy)]
struct Run {
    left: usize,
    right: usize,
    sum: f64,
    sum_sq: f64,
    count: usize,
}

impl Run {
    fn mean(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            self.sum / self.count as f64
        }
    }
    fn stddev(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            let mean = self.mean();
            ((self.sum_sq / self.count as f64) - mean * mean)
                .max(0.0)
                .sqrt()
        }
    }
    fn len(&self) -> usize {
        self.right - self.left + 1
    }
}

fn longest_valid_run(row: &[f32], params: &LockbarParams, min_len: usize) -> Option<Run> {
    let mut best: Option<Run> = None;
    let mut current: Option<Run> = None;

    for (i, &z) in row.iter().enumerate() {
        let valid = z >= params.depth_min_mm && z <= params.depth_max_mm;
        if valid {
            let cur = current.get_or_insert(Run {
                left: i,
                right: i,
                sum: 0.0,
                sum_sq: 0.0,
                count: 0,
            });
            cur.right = i;
            cur.sum += f64::from(z);
            cur.sum_sq += f64::from(z) * f64::from(z);
            cur.count += 1;
        } else if let Some(closed) = current.take() {
            best = best_of(best, closed, min_len);
        }
    }
    if let Some(closed) = current.take() {
        best = best_of(best, closed, min_len);
    }
    best
}

fn best_of(prev: Option<Run>, candidate: Run, min_len: usize) -> Option<Run> {
    if candidate.len() < min_len {
        return prev;
    }
    Some(match prev {
        Some(p) if p.len() >= candidate.len() => p,
        _ => candidate,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a synthetic frame with a horizontal band of constant depth.
    fn frame_with_band(
        width: u32,
        height: u32,
        band_top: u32,
        band_height: u32,
        band_left: u32,
        band_right: u32,
        depth_mm: f32,
    ) -> Vec<f32> {
        let mut out = vec![0.0_f32; (width * height) as usize];
        for v in band_top..(band_top + band_height).min(height) {
            for u in band_left..=band_right.min(width - 1) {
                out[(v * width + u) as usize] = depth_mm;
            }
        }
        out
    }

    #[test]
    fn detects_a_full_width_band_at_the_bottom() {
        let data = frame_with_band(100, 100, 80, 5, 0, 99, 600.0);
        let obs = detect_lockbar(&data, 100, 100, &LockbarParams::default()).expect("found");
        assert!((obs.row as i32 - 80).abs() <= 5, "row was {}", obs.row);
        assert_eq!(obs.left_col, 0);
        assert_eq!(obs.right_col, 99);
        assert!((obs.mean_depth_mm - 600.0).abs() < 1e-2);
        assert!(obs.depth_stddev_mm < 1.0);
    }

    #[test]
    fn returns_none_when_band_is_too_short() {
        // 100px-wide frame, 30% min run → 30px. Band is 20px → reject.
        let data = frame_with_band(100, 100, 80, 5, 40, 59, 600.0);
        let obs = detect_lockbar(&data, 100, 100, &LockbarParams::default());
        assert!(obs.is_none(), "should reject short band");
    }

    #[test]
    fn skips_bands_in_the_top_half() {
        // Band is at row 20 — outside the bottom search zone (default 35%
        // of the bottom). Expect None.
        let data = frame_with_band(100, 100, 20, 5, 0, 99, 600.0);
        let obs = detect_lockbar(&data, 100, 100, &LockbarParams::default());
        assert!(obs.is_none(), "should ignore top-half bands");
    }

    #[test]
    fn picks_the_widest_candidate_when_two_match() {
        // A short band at the very bottom and a wide one a bit above —
        // both inside the search zone. Expect the wide one.
        let mut data = frame_with_band(200, 100, 70, 3, 0, 199, 600.0);
        // Add a narrow band lower
        for v in 95..98 {
            for u in 50..70 {
                data[(v * 200 + u) as usize] = 800.0;
            }
        }
        let obs = detect_lockbar(&data, 200, 100, &LockbarParams::default()).expect("found");
        assert_eq!(obs.left_col, 0);
        assert_eq!(obs.right_col, 199);
        assert!((obs.mean_depth_mm - 600.0).abs() < 1e-2);
    }

    #[test]
    fn rejects_bands_with_high_depth_variance() {
        // A "band" whose depth oscillates wildly — should be rejected by
        // the stddev cap.
        let width = 100u32;
        let height = 100u32;
        let mut data = vec![0.0_f32; (width * height) as usize];
        for v in 80..85 {
            for u in 0..width {
                let alt = if (u + v) % 2 == 0 { 400.0 } else { 1_200.0 };
                data[(v * width + u) as usize] = alt;
            }
        }
        let obs = detect_lockbar(&data, width, height, &LockbarParams::default());
        assert!(obs.is_none(), "high stddev band should be rejected");
    }
}
