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
//! Knowing the lockbar's geometric width (currently hardcoded as
//! [`LOCKBAR_WIDTH_MM`], eventually pulled from VPX's table config) the
//! caller turns this observation into:
//! * a horizon reference (`row` → camera pitch),
//! * a focal-length anchor (`width_px` + known `width_mm` → `fx`,
//!   via the pinhole inverse `Z = fx · W / w_px`),
//! * a metric distance (`mean_depth_mm`),
//! * a lateral-offset signal (per-end depth → camera yaw) — useful for
//!   cabs whose topper / siderail forces the camera off the centerline
//!   (girophare, etc.).

/// Physical outer-edge-to-outer-edge width of a pincab lockbar in
/// millimetres. Williams/Bally widebody = 24-5/16" ≈ 618 mm; standard
/// body = 22-5/16" ≈ 567 mm. Sylvain's widebody measures 610 mm. VPX's
/// table file already carries the cabinet dimensions and we'll plumb
/// them through later — hardcoded for now so the lockbar detector has
/// a real-world length scale today.
pub const LOCKBAR_WIDTH_MM: f32 = 610.0;

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

// ============================================================ RGB detection
//
// The RGB path is the universal one: it works on any camera that
// produces a colour frame (Kinect v1/v2 RGB stream + webcam), regardless
// of whether depth is available. It looks for the strongest horizontal
// luminance edge in the bottom slice of the frame, which corresponds to
// the lockbar's top boundary — the transition between the lockbar
// surface (black-oxide / brushed stainless / chrome / painted wood) and
// the playfield (or cabinet interior) behind it.
//
// Per-column gradient + median + least-squares line fit was preferred
// over Canny+HoughLinesP to keep dependencies tight (no OpenCV /
// imageproc). The algorithm is O(W * (H * bottom_fraction)) — ~2 ms on
// a 1920×1080 frame and microseconds on 640×480, which fits comfortably
// in a frame budget at 30–60 fps.

/// Detected lockbar in the RGB image. Endpoints are the leftmost and
/// rightmost inlier columns from the line fit; the line may be tilted,
/// so `left_row` and `right_row` can differ.
#[derive(Debug, Clone, Copy)]
pub struct LockbarObservationRgb {
    pub frame_width: u32,
    pub frame_height: u32,
    pub left_col: u32,
    pub left_row: u32,
    pub right_col: u32,
    pub right_row: u32,
    /// Slope of the detected edge in degrees. Image Y axis points down,
    /// so a positive value means the right endpoint sits lower in the
    /// image than the left — i.e. the camera is rolled CCW around its
    /// optical axis (or the cab itself is tilted). Useful as a roll
    /// signal for the camera calibration.
    pub slope_deg: f32,
    /// Number of columns that contributed to the line fit after the
    /// outlier filter. Confidence proxy — `0..frame_width`.
    pub n_inliers: u32,
}

impl LockbarObservationRgb {
    /// Horizontal extent of the detected edge in pixels.
    pub fn width_px(&self) -> u32 {
        self.right_col - self.left_col + 1
    }

    /// Mean image row of the two endpoints — handy when the consumer
    /// just wants a single Y reference and doesn't care about tilt.
    pub fn mean_row(&self) -> u32 {
        (self.left_row + self.right_row) / 2
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LockbarRgbParams {
    /// Start the row search at `bottom_fraction * height`. `0.65` keeps
    /// the bottom 35% of the frame in scope, which matches the typical
    /// camera-on-backbox geometry.
    pub bottom_fraction: f32,
    /// Minimum absolute luminance gradient (0..255 scale) for a column
    /// to vote. Columns where the player's body produces only weak
    /// gradients (uniform skin / fabric) fall below this.
    pub min_edge_strength: f32,
    /// Required fraction of columns voting consistently for the line
    /// to call it a real lockbar. `0.40` = 40% of the image width.
    pub min_width_fraction: f32,
    /// Inlier filter: drop columns whose detected row is farther than
    /// this many pixels from the median row of all voters.
    pub max_row_deviation: u32,
}

impl Default for LockbarRgbParams {
    fn default() -> Self {
        Self {
            bottom_fraction: 0.65,
            min_edge_strength: 20.0,
            min_width_fraction: 0.40,
            max_row_deviation: 12,
        }
    }
}

/// Detect the lockbar in an RGB888 frame (row-major, 3 bytes/pixel,
/// channel order R,G,B). Returns `None` when the candidate edge spans
/// too little of the frame width or no consistent line emerges. See the
/// module-level comment for algorithm rationale.
pub fn detect_lockbar_rgb(
    rgb888: &[u8],
    width: u32,
    height: u32,
    params: &LockbarRgbParams,
) -> Option<LockbarObservationRgb> {
    let w = width as usize;
    let h = height as usize;
    if w == 0 || h < 4 || rgb888.len() < w * h * 3 {
        return None;
    }
    let row_start = ((params.bottom_fraction * height as f32) as usize).min(h.saturating_sub(2));
    if row_start + 1 >= h {
        return None;
    }
    let min_cols = ((params.min_width_fraction * width as f32) as usize).max(2);

    // Rec.709 integer luma — avoids per-pixel f32 conversion.
    // 54/183/19 ≈ 0.2126/0.7152/0.0722 scaled by 256, then >>8.
    let luma = |c: usize, r: usize| -> u16 {
        let i = (r * w + c) * 3;
        let g = u32::from(rgb888[i + 1]) * 183;
        let r_ = u32::from(rgb888[i]) * 54;
        let b = u32::from(rgb888[i + 2]) * 19;
        ((r_ + g + b) >> 8) as u16
    };

    // Per-column: row of strongest vertical gradient in the ROI.
    let mut per_col_row: Vec<Option<u32>> = vec![None; w];
    for c in 0..w {
        let mut best_row: u32 = 0;
        let mut best_grad: i32 = 0;
        for r in row_start..(h - 1) {
            let above = luma(c, r) as i32;
            let below = luma(c, r + 1) as i32;
            let g = (below - above).abs();
            if g > best_grad {
                best_grad = g;
                best_row = r as u32;
            }
        }
        if (best_grad as f32) >= params.min_edge_strength {
            per_col_row[c] = Some(best_row);
        }
    }

    // Collect raw votes.
    let votes: Vec<(usize, u32)> = per_col_row
        .iter()
        .enumerate()
        .filter_map(|(c, &r)| r.map(|r| (c, r)))
        .collect();
    if votes.len() < min_cols {
        return None;
    }

    // Median row → inlier filter.
    let mut rows_sorted: Vec<u32> = votes.iter().map(|(_, r)| *r).collect();
    rows_sorted.sort_unstable();
    let median_row = rows_sorted[rows_sorted.len() / 2];
    let dev = params.max_row_deviation;

    let inliers: Vec<(usize, u32)> = votes
        .into_iter()
        .filter(|(_, r)| r.abs_diff(median_row) <= dev)
        .collect();
    if inliers.len() < min_cols {
        return None;
    }

    // Least-squares line fit row = slope * col + intercept.
    let n = inliers.len() as f64;
    let mut sx = 0.0f64;
    let mut sy = 0.0f64;
    let mut sxx = 0.0f64;
    let mut sxy = 0.0f64;
    for &(c, r) in &inliers {
        let cx = c as f64;
        let ry = f64::from(r);
        sx += cx;
        sy += ry;
        sxx += cx * cx;
        sxy += cx * ry;
    }
    let denom = n * sxx - sx * sx;
    let (slope, intercept) = if denom.abs() > 1e-6 {
        let a = (n * sxy - sx * sy) / denom;
        let b = (sy - a * sx) / n;
        (a, b)
    } else {
        (0.0, sy / n)
    };

    let left_col = inliers.iter().map(|(c, _)| *c).min().unwrap();
    let right_col = inliers.iter().map(|(c, _)| *c).max().unwrap();
    let left_row = (slope * left_col as f64 + intercept).round().clamp(0.0, (h - 1) as f64) as u32;
    let right_row = (slope * right_col as f64 + intercept)
        .round()
        .clamp(0.0, (h - 1) as f64) as u32;
    let slope_deg = slope.atan().to_degrees() as f32;

    Some(LockbarObservationRgb {
        frame_width: width,
        frame_height: height,
        left_col: left_col as u32,
        left_row,
        right_col: right_col as u32,
        right_row,
        slope_deg,
        n_inliers: inliers.len() as u32,
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

    // ============================== RGB detection tests

    /// Build a synthetic RGB frame: white above `band_top`, black from
    /// `band_top` onward. The transition row is the lockbar's top edge.
    fn rgb_with_horizontal_edge(width: u32, height: u32, band_top: u32) -> Vec<u8> {
        let mut out = vec![0u8; (width as usize) * (height as usize) * 3];
        for v in 0..height {
            let lum: u8 = if v < band_top { 240 } else { 20 };
            for u in 0..width {
                let i = ((v * width + u) as usize) * 3;
                out[i] = lum;
                out[i + 1] = lum;
                out[i + 2] = lum;
            }
        }
        out
    }

    #[test]
    fn rgb_detects_horizontal_edge_in_bottom_zone() {
        // 100×100 frame, edge at row 80 → bottom 35% covers 65..100.
        let data = rgb_with_horizontal_edge(100, 100, 80);
        let obs =
            detect_lockbar_rgb(&data, 100, 100, &LockbarRgbParams::default()).expect("detected");
        // The gradient peak between row 79 (white) and row 80 (black)
        // → strongest at row 79 (above = white, below = black).
        assert!(
            obs.mean_row() >= 78 && obs.mean_row() <= 80,
            "mean row was {}",
            obs.mean_row()
        );
        assert_eq!(obs.left_col, 0);
        assert_eq!(obs.right_col, 99);
        assert!(obs.slope_deg.abs() < 0.5, "should be flat, got {}°", obs.slope_deg);
        assert!(obs.n_inliers >= 90, "n_inliers = {}", obs.n_inliers);
    }

    #[test]
    fn rgb_ignores_edge_in_top_zone() {
        // Edge at row 20 — outside the bottom 35% search zone.
        let data = rgb_with_horizontal_edge(100, 100, 20);
        let obs = detect_lockbar_rgb(&data, 100, 100, &LockbarRgbParams::default());
        assert!(obs.is_none(), "top-zone edge should not be detected");
    }

    #[test]
    fn rgb_recovers_slope_from_tilted_edge() {
        // Tilted edge: row = 80 + (col - 50) * 0.1 → slope ≈ atan(0.1) ≈ 5.7°
        let width = 200u32;
        let height = 100u32;
        let mut out = vec![0u8; (width as usize) * (height as usize) * 3];
        for u in 0..width {
            let edge_row = 80.0 + ((u as f32) - 100.0) * 0.1;
            for v in 0..height {
                let lum: u8 = if (v as f32) < edge_row { 240 } else { 20 };
                let i = ((v * width + u) as usize) * 3;
                out[i] = lum;
                out[i + 1] = lum;
                out[i + 2] = lum;
            }
        }
        let obs =
            detect_lockbar_rgb(&out, width, height, &LockbarRgbParams::default()).expect("detected");
        let expected_deg = 0.1f32.atan().to_degrees();
        assert!(
            (obs.slope_deg - expected_deg).abs() < 1.0,
            "slope {}° not within ±1° of expected {}°",
            obs.slope_deg,
            expected_deg
        );
    }

    #[test]
    fn rgb_rejects_uniform_frame() {
        // All-gray frame: no edge anywhere.
        let data = vec![128u8; 100 * 100 * 3];
        let obs = detect_lockbar_rgb(&data, 100, 100, &LockbarRgbParams::default());
        assert!(obs.is_none(), "uniform frame should produce no detection");
    }
}
