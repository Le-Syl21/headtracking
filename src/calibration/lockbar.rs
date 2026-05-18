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
// of whether depth is available. The lockbar surface (700 mm wide × ~70
// mm deep) projects to a narrow horizontal BAND in the image with two
// roughly parallel edges:
//
//   * TOP edge — boundary between the lockbar and the playfield/cab
//     interior behind it (from the camera's POV looking down at the
//     player)
//   * BOTTOM edge — boundary between the lockbar and the floor / player
//     front
//
// We find the strongest horizontal luminance edge in the bottom slice
// (line A), mask out a band around it, find a SECOND horizontal edge
// elsewhere (line B), and validate the pair: separation must match the
// expected band thickness (~20–80 px depending on backend), slopes must
// agree, and the band must NOT span the full frame width (that
// signature belongs to room features like horizons, not a finite cab).
//
// Per-column gradient + median + least-squares line fit avoids any
// OpenCV / imageproc dependency. ~2 ms on a 1920×1080 frame and tens of
// microseconds on 640×480 — fits in a 30–60 fps budget.

/// A fitted line in image space, returned by the internal helper.
/// `row(col)` = slope * col + intercept.
#[derive(Debug, Clone, Copy)]
struct FittedLine {
    slope: f64,
    intercept: f64,
    left_col: u32,
    right_col: u32,
    #[allow(dead_code)]
    n_inliers: u32,
}

impl FittedLine {
    fn row_at(&self, col: u32) -> u32 {
        (self.slope * f64::from(col) + self.intercept)
            .round()
            .max(0.0) as u32
    }
    fn mean_row(&self) -> u32 {
        (self.row_at(self.left_col) + self.row_at(self.right_col)) / 2
    }
    fn slope_deg(&self) -> f32 {
        self.slope.atan().to_degrees() as f32
    }
}

/// Detected lockbar in the RGB image, expressed as a 4-corner quad
/// (clockwise from top-left). With perspective the quad is generally a
/// trapezoid — top and bottom edges aren't required to be parallel in
/// pixel space.
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

#[derive(Debug, Clone, Copy)]
pub struct LockbarRgbParams {
    /// Start the row search at `bottom_fraction * height`. The 70 mm
    /// lockbar at a typical 0.7–1.0 m camera distance projects into
    /// the last 10–20% of the frame, so the default 0.75 keeps the
    /// bottom 25% in scope with margin.
    pub bottom_fraction: f32,
    /// Minimum absolute luminance gradient (0..255 scale) for a column
    /// to vote. Columns occluded by the player produce weak gradients
    /// over uniform skin / fabric and fall below this.
    pub min_edge_strength: f32,
    /// Required fraction of columns voting consistently for the line
    /// to be considered a real edge. `0.30` = 30% of the image width.
    pub min_width_fraction: f32,
    /// Upper cap on edge width — a "lockbar" spanning >85% of the
    /// frame is almost certainly a room horizon / wall trim, not the
    /// finite cab front. Rejecting wide edges kills the most common
    /// false positive.
    pub max_width_fraction: f32,
    /// Inlier filter: drop columns whose detected row is farther than
    /// this many pixels from the median row of all voters.
    pub max_row_deviation: u32,
    /// Minimum pixel separation between top and bottom edges of the
    /// lockbar band. Below this we treat the two edges as the same
    /// detection (noise / single edge), reject the pair.
    pub min_separation: u32,
    /// Maximum pixel separation. The 70 mm bar at 0.5–1.5 m projects
    /// into ~20–100 px depending on backend focal length; anything
    /// thicker is a different surface entirely.
    pub max_separation: u32,
    /// Maximum allowed slope difference between top and bottom edges
    /// (in degrees). The two lockbar edges must be parallel within
    /// measurement noise; a big mismatch means we paired the wrong
    /// two horizontals.
    pub max_slope_diff_deg: f32,
    /// Minimum pixel ratio of `width / thickness`. Physical lockbars
    /// span ~40 cm × 10 cm at the skinniest to ~80 cm × 5 cm at the
    /// boxiest (Sylvain's widebody = 61 cm × 7 cm → ratio ≈ 8.7).
    /// Perspective preserves length ratios on a fronto-parallel
    /// surface, so the pixel ratio should land near the physical
    /// ratio regardless of camera distance.
    pub min_aspect_ratio: f32,
    pub max_aspect_ratio: f32,
    /// The detected pair's top edge must sit at least this far down
    /// the frame (as a fraction of frame height). Cabs are placed
    /// roughly head-height with the camera on the backbox tilted
    /// downward, so the lockbar always falls into the lower portion
    /// of the frame — shelves, door frames, window sills that pass
    /// aspect-ratio are rejected here.
    pub min_top_edge_row_fraction: f32,
}

impl Default for LockbarRgbParams {
    fn default() -> Self {
        Self {
            bottom_fraction: 0.75,
            min_edge_strength: 20.0,
            // 0.05 = 5% of frame width. The Kinect v2's wide FOV
            // means a 61 cm lockbar at ~76 cm camera-to-bar distance
            // projects to ~10% of 1920 columns; after the joint-
            // column intersection (columns must have peaks at
            // BOTH lockbar edges) that drops further. Aspect-ratio
            // + min-top-row gates keep false positives out.
            min_width_fraction: 0.05,
            max_width_fraction: 0.85,
            max_row_deviation: 12,
            min_separation: 8,
            max_separation: 120,
            // Wide-FOV / fisheye webcams curve horizontal lines —
            // top and bottom of the lockbar diverge slightly in
            // image space. 5° is enough for the cheap action-cam
            // we tested without letting wildly mismatched pairs in.
            max_slope_diff_deg: 5.0,
            // 40 cm × 10 cm = 4.0 (skinniest plausible cab),
            // 80 cm × 5 cm = 16.0 (widest plausible cab). 18 leaves
            // some slack for edge measurement noise inflating
            // the apparent thickness.
            min_aspect_ratio: 4.0,
            max_aspect_ratio: 18.0,
            // 0.80 = lockbar's top edge must be in the bottom 20% of
            // the frame. Trades a bit of permissiveness for false
            // negatives if the camera angle puts the lockbar higher,
            // for very strong rejection of shelves / horizons /
            // baseboards that look like a lockbar by aspect alone.
            min_top_edge_row_fraction: 0.80,
        }
    }
}

/// Detect the lockbar in an RGB888 frame (row-major, 3 bytes/pixel,
/// channel order R,G,B). Returns `None` unless we find a PAIR of
/// roughly parallel horizontal edges in the bottom slice that's
/// geometrically consistent with a lockbar band — see
/// [`LockbarRgbParams`] for the gating constraints.
pub fn detect_lockbar_rgb(
    rgb888: &[u8],
    width: u32,
    height: u32,
    params: &LockbarRgbParams,
) -> Option<LockbarQuadRgb> {
    let w = width as usize;
    let h = height as usize;
    if w == 0 || h < 4 || rgb888.len() < w * h * 3 {
        tracing::debug!(target: "lockbar", "reject: bad frame {w}x{h} / {} bytes", rgb888.len());
        return None;
    }
    let row_start = ((params.bottom_fraction * height as f32) as usize).min(h.saturating_sub(2));
    if row_start + 1 >= h {
        return None;
    }
    let min_cols = ((params.min_width_fraction * width as f32) as usize).max(2);
    let max_cols = ((params.max_width_fraction * width as f32) as usize).max(2);

    // Rec.709 integer luma — avoids per-pixel f32 conversion.
    // 54/183/19 ≈ 0.2126/0.7152/0.0722 scaled by 256, then >>8.
    let luma = |c: usize, r: usize| -> u16 {
        let i = (r * w + c) * 3;
        let g = u32::from(rgb888[i + 1]) * 183;
        let r_ = u32::from(rgb888[i]) * 54;
        let b = u32::from(rgb888[i + 2]) * 19;
        ((r_ + g + b) >> 8) as u16
    };

    // For each column, find the row(s) of every local-maximum
    // gradient above `min_edge_strength`. Keep the TOP-K strongest
    // peaks per column. K=4 is generous because the disambiguation
    // is done downstream by the JOINT-column intersection (lockbar
    // columns must have peaks near BOTH the top and bottom edge
    // simultaneously) — the K=2 setting we tried earlier left
    // textured columns without enough peaks for the intersection
    // to succeed.
    const TOP_K_PEAKS: usize = 4;
    let scan_col = |c: usize, mask_pred: &dyn Fn(u32) -> bool| -> Vec<u32> {
        // Walk the column once, find all local-max rows with
        // gradient >= threshold, keep the top K by strength.
        let mut peaks: Vec<(i32, u32)> = Vec::new(); // (grad, row)
        let mut prev_g: i32 = 0;
        let mut rising = false;
        for r in row_start..(h - 1) {
            if mask_pred(r as u32) {
                rising = false;
                prev_g = 0;
                continue;
            }
            let above = luma(c, r) as i32;
            let below = luma(c, r + 1) as i32;
            let g = (below - above).abs();
            if rising && g < prev_g && prev_g >= params.min_edge_strength as i32 {
                peaks.push((prev_g, (r - 1) as u32));
                rising = false;
            }
            if g > prev_g {
                rising = true;
            }
            prev_g = g;
        }
        if rising && prev_g >= params.min_edge_strength as i32 {
            peaks.push((prev_g, (h - 2) as u32));
        }
        peaks.sort_unstable_by_key(|&(g, _)| std::cmp::Reverse(g));
        peaks.truncate(TOP_K_PEAKS);
        peaks.into_iter().map(|(_, r)| r).collect()
    };

    // Pre-scan once: all top-K peaks per column.
    let no_mask = |_r: u32| false;
    let per_col_full: Vec<Vec<u32>> = (0..w).map(|c| scan_col(c, &no_mask)).collect();
    let n_cols_voting = per_col_full.iter().filter(|p| !p.is_empty()).count();
    if n_cols_voting < min_cols {
        tracing::debug!(
            target: "lockbar",
            "reject: only {n_cols_voting} columns produced any peak (need ≥{min_cols})"
        );
        return None;
    }

    // Enumerate ALL valid horizontal line candidates (peaks in the
    // row distribution that have ≥ min_cols unique-column voters
    // and width ≤ max_cols). Then iterate every pair, score by how
    // close the aspect ratio lands to the physical lockbar's 8.7,
    // and return the best pair that passes every geometry gate.
    let dev = params.max_row_deviation;
    let candidates = fit_all_lines(&per_col_full, min_cols, max_cols, dev);
    if candidates.is_empty() {
        tracing::debug!(
            target: "lockbar",
            "reject: no candidate horizontal lines (n_cols_voting={n_cols_voting}, \
             min_cols={min_cols}, max_cols={max_cols})"
        );
        return None;
    }
    tracing::debug!(
        target: "lockbar",
        "{} line candidates, scanning {} pairs",
        candidates.len(),
        candidates.len() * (candidates.len() - 1) / 2
    );

    let min_top_row = (params.min_top_edge_row_fraction * height as f32) as u32;
    let target_aspect: f32 = (LOCKBAR_WIDTH_MM / 70.0).max(4.0); // ≈ 8.7

    // For each candidate pair, compute the COLUMN INTERSECTION:
    // columns that have a peak near top_row AND a peak near
    // bot_row. This isolates true lockbar columns from columns
    // that only have one of the two edges visible (shelves /
    // floor seams). The lockbar's actual horizontal extent is
    // the longest contiguous run inside this intersection.
    let mut best: Option<(f32, FittedLine, FittedLine, u32, u32, u32, u32)> = None;

    for i in 0..candidates.len() {
        for j in (i + 1)..candidates.len() {
            let (a, b) = (candidates[i], candidates[j]);
            let slope_diff = (a.slope_deg() - b.slope_deg()).abs();
            if slope_diff > params.max_slope_diff_deg {
                tracing::trace!(
                    target: "lockbar",
                    "pair ({},{})→ slope diff {:.2}° > {:.2}°",
                    a.mean_row(),
                    b.mean_row(),
                    slope_diff,
                    params.max_slope_diff_deg
                );
                continue;
            }
            let (top, bottom) = if a.mean_row() < b.mean_row() {
                (a, b)
            } else {
                (b, a)
            };
            if top.mean_row() < min_top_row {
                tracing::trace!(
                    target: "lockbar",
                    "pair ({},{})→ top {} < min_top {}",
                    top.mean_row(),
                    bottom.mean_row(),
                    top.mean_row(),
                    min_top_row
                );
                continue;
            }

            // Find every column with a peak near BOTH top.row_at(col)
            // and bottom.row_at(col). Walk columns and check.
            let mut joint_cols: Vec<u32> = Vec::new();
            for (c, peaks) in per_col_full.iter().enumerate() {
                let target_top = top.row_at(c as u32);
                let target_bot = bottom.row_at(c as u32);
                let has_top = peaks.iter().any(|r| r.abs_diff(target_top) <= dev);
                let has_bot = peaks.iter().any(|r| r.abs_diff(target_bot) <= dev);
                if has_top && has_bot {
                    joint_cols.push(c as u32);
                }
            }
            if joint_cols.len() < min_cols {
                tracing::trace!(
                    target: "lockbar",
                    "pair ({},{})→ joint cols {} < min {}",
                    top.mean_row(),
                    bottom.mean_row(),
                    joint_cols.len(),
                    min_cols
                );
                continue;
            }

            // Longest contiguous run in joint_cols.
            const MAX_COL_GAP: u32 = 24;
            let mut best_start = 0usize;
            let mut best_end = 0usize;
            let mut run_start = 0usize;
            for k in 1..joint_cols.len() {
                if joint_cols[k] - joint_cols[k - 1] > MAX_COL_GAP {
                    if k - 1 - run_start > best_end - best_start {
                        best_start = run_start;
                        best_end = k - 1;
                    }
                    run_start = k;
                }
            }
            let last = joint_cols.len() - 1;
            if last - run_start > best_end - best_start {
                best_start = run_start;
                best_end = last;
            }
            let lc = joint_cols[best_start];
            let rc = joint_cols[best_end];
            let n_joint_run = (best_end - best_start + 1) as u32;
            if (n_joint_run as usize) < min_cols {
                tracing::trace!(
                    target: "lockbar",
                    "pair ({},{})→ contig run {} < min {}",
                    top.mean_row(),
                    bottom.mean_row(),
                    n_joint_run,
                    min_cols
                );
                continue;
            }

            let mean_width = (rc - lc + 1) as f32;
            if mean_width > max_cols as f32 {
                tracing::trace!(
                    target: "lockbar",
                    "pair ({},{})→ width {} > max {}",
                    top.mean_row(),
                    bottom.mean_row(),
                    mean_width,
                    max_cols
                );
                continue;
            }
            let sep_left = bottom.row_at(lc).saturating_sub(top.row_at(lc));
            let sep_right = bottom.row_at(rc).saturating_sub(top.row_at(rc));
            let thickness = (sep_left + sep_right) / 2;
            if thickness < params.min_separation || thickness > params.max_separation {
                tracing::trace!(
                    target: "lockbar",
                    "pair ({},{})→ thickness {} not in [{}, {}]",
                    top.mean_row(),
                    bottom.mean_row(),
                    thickness,
                    params.min_separation,
                    params.max_separation
                );
                continue;
            }
            let aspect = if thickness > 0 {
                mean_width / thickness as f32
            } else {
                continue;
            };
            if aspect < params.min_aspect_ratio || aspect > params.max_aspect_ratio {
                tracing::trace!(
                    target: "lockbar",
                    "pair ({},{})→ aspect {:.1} not in [{:.1}, {:.1}] (w={} t={})",
                    top.mean_row(),
                    bottom.mean_row(),
                    aspect,
                    params.min_aspect_ratio,
                    params.max_aspect_ratio,
                    mean_width as u32,
                    thickness
                );
                continue;
            }
            let aspect_err = (aspect - target_aspect).abs() / target_aspect;
            // Strong inlier counts beat near-perfect aspect; tie-break
            // by sharpness only.
            let inlier_bonus = 1.0 / (1.0 + n_joint_run as f32 / 100.0);
            let score = aspect_err + 0.1 * inlier_bonus;
            if best.as_ref().is_none_or(|(s, _, _, _, _, _, _)| score < *s) {
                best = Some((score, top, bottom, lc, rc, thickness, n_joint_run));
            }
        }
    }

    let (score, top, bottom, lc, rc, thickness, n_joint_run) = best.or_else(|| {
        tracing::debug!(
            target: "lockbar",
            "reject: no candidate pair passed all gates after joint-column intersection"
        );
        None
    })?;
    let mean_width = (rc - lc + 1) as f32;
    let aspect = mean_width / thickness as f32;
    tracing::debug!(
        target: "lockbar",
        "accept: row≈{}, width={} px, thickness={} px, aspect={:.1} (target {:.1}, score={:.3}), \
         slope={:.2}°, joint cols={n_joint_run}",
        top.mean_row(),
        mean_width as u32,
        thickness,
        aspect,
        target_aspect,
        score,
        top.slope_deg(),
    );
    let bound_row = |r: u32| r.min((h - 1) as u32);
    let corners = [
        (lc, bound_row(top.row_at(lc))),
        (rc, bound_row(top.row_at(rc))),
        (rc, bound_row(bottom.row_at(rc))),
        (lc, bound_row(bottom.row_at(lc))),
    ];
    Some(LockbarQuadRgb {
        frame_width: width,
        frame_height: height,
        corners,
        slope_deg: top.slope_deg(),
        thickness_px: thickness,
        n_inliers_top: n_joint_run,
        n_inliers_bottom: n_joint_run,
    })
}

/// Enumerate every fittable horizontal line in `per_col` — i.e.
/// every distinct row cluster (peak in the row distribution) that
/// has at least `min_cols` unique-column voters and fits inside
/// `max_cols` width. Returns candidates ordered from highest row
/// (image bottom) to lowest (image top); the caller iterates pairs
/// and scores them by lockbar shape.
fn fit_all_lines(
    per_col: &[Vec<u32>],
    min_cols: usize,
    max_cols: usize,
    max_row_deviation: u32,
) -> Vec<FittedLine> {
    let mut all_peaks: Vec<(usize, u32)> = per_col
        .iter()
        .enumerate()
        .flat_map(|(c, peaks)| peaks.iter().map(move |&r| (c, r)))
        .collect();
    if all_peaks.len() < min_cols {
        return Vec::new();
    }
    all_peaks.sort_unstable_by_key(|&(_, r)| r);
    let dev = max_row_deviation;
    // Cluster span tighter than the inlier radius: the lockbar's
    // top and bottom edges land in the same per-column peak set
    // and must be detected as DISTINCT clusters. `span = dev`
    // (window ±dev/2) resolves edges separated by `dev` pixels,
    // while the wider inlier collection (±dev = ±max_row_deviation)
    // still pulls in noise around each edge centre.
    let span = dev;
    let n = all_peaks.len();

    let mut out: Vec<FittedLine> = Vec::new();
    // Track which row centres we've already emitted so we don't
    // produce many overlapping fits for the same cluster. Threshold
    // is `dev` (not `span = 2*dev`) so two parallel edges only one
    // band thick apart still count as distinct candidates — that's
    // the lockbar's own top/bottom edges in the limit case.
    let mut emitted_centers: Vec<u32> = Vec::new();
    let separated_enough =
        |c: u32, prev: &[u32]| -> bool { prev.iter().all(|&p| c.abs_diff(p) > dev) };

    let mut hi = n;
    let mut lo = n;
    while hi > 0 {
        hi -= 1;
        while lo > 0 && all_peaks[hi].1 - all_peaks[lo - 1].1 <= span {
            lo -= 1;
        }
        let window = &all_peaks[lo..=hi];
        let mut cols_in: std::collections::HashSet<usize> = std::collections::HashSet::new();
        for &(c, _) in window {
            cols_in.insert(c);
        }
        if cols_in.len() < min_cols {
            tracing::trace!(
                target: "lockbar",
                "fit: hi={} row={} window=[{},{}] cols={} < {}",
                hi,
                all_peaks[hi].1,
                window.first().unwrap().1,
                window.last().unwrap().1,
                cols_in.len(),
                min_cols
            );
            continue;
        }
        let center = (window.first().unwrap().1 + window.last().unwrap().1) / 2;
        tracing::trace!(
            target: "lockbar",
            "fit: hi={} row={} window=[{},{}] center={} cols={}",
            hi,
            all_peaks[hi].1,
            window.first().unwrap().1,
            window.last().unwrap().1,
            center,
            cols_in.len()
        );
        if !separated_enough(center, &emitted_centers) {
            tracing::trace!(target: "lockbar", "fit: center={center} too close to emitted");
            continue;
        }
        let mut inliers: Vec<(usize, u32)> = Vec::with_capacity(cols_in.len());
        for &col in &cols_in {
            let best = per_col[col]
                .iter()
                .copied()
                .filter(|r| r.abs_diff(center) <= max_row_deviation)
                .min_by_key(|r| r.abs_diff(center));
            if let Some(r) = best {
                inliers.push((col, r));
            }
        }
        if inliers.len() < min_cols {
            tracing::trace!(target: "lockbar", "fit: center={center} inliers={} < {min_cols}", inliers.len());
            continue;
        }
        // Contiguity filter: keep only the longest run of inlier
        // columns where consecutive ones are at most MAX_COL_GAP
        // apart. The lockbar is a localised object; without this,
        // co-row features elsewhere in the frame get bundled in.
        const MAX_COL_GAP: usize = 24;
        inliers.sort_unstable_by_key(|&(c, _)| c);
        let mut best_start = 0usize;
        let mut best_end = 0usize;
        let mut run_start = 0usize;
        for k in 1..inliers.len() {
            if inliers[k].0 - inliers[k - 1].0 > MAX_COL_GAP {
                if k - 1 - run_start > best_end - best_start {
                    best_start = run_start;
                    best_end = k - 1;
                }
                run_start = k;
            }
        }
        let last_idx = inliers.len() - 1;
        if last_idx - run_start > best_end - best_start {
            best_start = run_start;
            best_end = last_idx;
        }
        let run_inliers: Vec<(usize, u32)> = inliers[best_start..=best_end].to_vec();
        if run_inliers.len() < min_cols {
            tracing::trace!(
                target: "lockbar",
                "fit: center={center} longest run {} < {min_cols}",
                run_inliers.len()
            );
            continue;
        }
        let inliers = run_inliers;
        let lc = inliers.first().unwrap().0;
        let rc = inliers.last().unwrap().0;
        if rc - lc + 1 > max_cols {
            tracing::trace!(target: "lockbar", "fit: center={center} run span={} > {max_cols}", rc - lc + 1);
            continue;
        }
        let np = inliers.len() as f64;
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
        let denom = np * sxx - sx * sx;
        let (slope, intercept) = if denom.abs() > 1e-6 {
            let a = (np * sxy - sx * sy) / denom;
            let b = (sy - a * sx) / np;
            (a, b)
        } else {
            (0.0, sy / np)
        };
        tracing::trace!(
            target: "lockbar",
            "candidate line: center≈{}, span=[{},{}] ({} cols), {} inliers",
            center,
            lc,
            rc,
            rc - lc + 1,
            inliers.len()
        );
        out.push(FittedLine {
            slope,
            intercept,
            left_col: lc as u32,
            right_col: rc as u32,
            n_inliers: inliers.len() as u32,
        });
        emitted_centers.push(center);
    }
    out
}

/// Legacy single-peak path — superseded by `fit_all_lines` but
/// kept around as a reference implementation. The synthetic unit
/// tests construct one peak per column, which this path handles
/// directly; the production path translates that into a multi-peak
/// vector and uses `fit_all_lines`.
#[allow(dead_code)]
fn fit_line_densest_legacy(
    per_col: &[Option<u32>],
    min_cols: usize,
    max_cols: usize,
    max_row_deviation: u32,
) -> Option<FittedLine> {
    let votes: Vec<(usize, u32)> = per_col
        .iter()
        .enumerate()
        .filter_map(|(c, &r)| r.map(|r| (c, r)))
        .collect();
    if votes.len() < min_cols {
        return None;
    }
    let mut rows_sorted: Vec<u32> = votes.iter().map(|(_, r)| *r).collect();
    rows_sorted.sort_unstable();
    let dev = max_row_deviation;
    let span = 2 * dev;
    let mut best_count = 0usize;
    let mut best_center = rows_sorted[rows_sorted.len() / 2];
    let mut lo = 0usize;
    for hi in 0..rows_sorted.len() {
        while rows_sorted[hi] - rows_sorted[lo] > span {
            lo += 1;
        }
        let count = hi - lo + 1;
        if count > best_count {
            best_count = count;
            best_center = (rows_sorted[lo] + rows_sorted[hi]) / 2;
        }
    }
    let inliers: Vec<(usize, u32)> = votes
        .into_iter()
        .filter(|(_, r)| r.abs_diff(best_center) <= max_row_deviation)
        .collect();
    if inliers.len() < min_cols {
        return None;
    }

    let left_col = inliers.iter().map(|(c, _)| *c).min().unwrap();
    let right_col = inliers.iter().map(|(c, _)| *c).max().unwrap();
    if right_col - left_col + 1 > max_cols {
        return None;
    }

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

    Some(FittedLine {
        slope,
        intercept,
        left_col: left_col as u32,
        right_col: right_col as u32,
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

    /// Build an RGB888 frame with a horizontal dark BAND between rows
    /// `band_top` and `band_bot` (inclusive). Everything else is white.
    /// `left_col`/`right_col` (inclusive) determine the band's
    /// horizontal extent; outside those columns the band rows are
    /// also white. This simulates a finite-width lockbar surface.
    fn rgb_with_band(
        width: u32,
        height: u32,
        band_top: u32,
        band_bot: u32,
        left_col: u32,
        right_col: u32,
    ) -> Vec<u8> {
        let mut out = vec![240u8; (width as usize) * (height as usize) * 3];
        for v in band_top..=band_bot.min(height - 1) {
            for u in left_col..=right_col.min(width - 1) {
                let i = ((v * width + u) as usize) * 3;
                out[i] = 20;
                out[i + 1] = 20;
                out[i + 2] = 20;
            }
        }
        out
    }

    #[test]
    fn rgb_detects_band_in_bottom_zone() {
        // 200×200 frame, dark band at rows 170..185 spanning cols
        // 30..170 (~70% width, ~15 px thick). bottom_fraction default
        // 0.75 → search starts at row 150 → band fully in scope; and
        // min_top_edge_row_fraction 0.80 → top must be ≥ 160 → the
        // top peak at row 169 satisfies it with margin.
        let data = rgb_with_band(200, 200, 170, 185, 30, 170);
        let q =
            detect_lockbar_rgb(&data, 200, 200, &LockbarRgbParams::default()).expect("detected");
        assert!(
            q.thickness_px >= 10 && q.thickness_px <= 20,
            "thickness {} px not near 15",
            q.thickness_px
        );
        // Quad spans the band roughly 30..170 horizontally — allow a
        // few pixels of slack at the edges (the gradient at columns
        // 30/170 vs 29/171 picks up subtly).
        assert!(
            q.corners[0].0 <= 35 && q.corners[1].0 >= 165,
            "horizontal extent off: corners {:?}",
            q.corners
        );
        assert!(
            q.slope_deg.abs() < 0.5,
            "should be flat, got {}°",
            q.slope_deg
        );
        assert!(q.n_inliers_top >= 100 && q.n_inliers_bottom >= 100);
    }

    #[test]
    fn rgb_rejects_single_edge() {
        // Half-white / half-dark frame — only ONE horizontal edge in
        // the ROI. The pair-finder shouldn't manufacture a second one.
        let mut data = vec![240u8; 200 * 200 * 3];
        for v in 170..200 {
            for u in 0..200 {
                let i = ((v * 200 + u) as usize) * 3;
                data[i] = 20;
                data[i + 1] = 20;
                data[i + 2] = 20;
            }
        }
        let q = detect_lockbar_rgb(&data, 200, 200, &LockbarRgbParams::default());
        assert!(q.is_none(), "single edge must not produce a quad");
    }

    #[test]
    fn rgb_rejects_full_width_horizon() {
        // Dark band spans the full width — looks like a room horizon
        // line, NOT a finite cabinet front. max_width_fraction 0.85
        // by default should reject it.
        let data = rgb_with_band(200, 200, 160, 175, 0, 199);
        let q = detect_lockbar_rgb(&data, 200, 200, &LockbarRgbParams::default());
        assert!(q.is_none(), "full-width band must be rejected");
    }

    #[test]
    fn rgb_ignores_band_in_top_zone() {
        // Band high in the frame — outside the bottom 25% ROI.
        let data = rgb_with_band(200, 200, 20, 35, 30, 170);
        let q = detect_lockbar_rgb(&data, 200, 200, &LockbarRgbParams::default());
        assert!(q.is_none(), "top-zone band should not be detected");
    }

    #[test]
    fn rgb_rejects_uniform_frame() {
        let data = vec![128u8; 200 * 200 * 3];
        let q = detect_lockbar_rgb(&data, 200, 200, &LockbarRgbParams::default());
        assert!(q.is_none(), "uniform frame should produce no detection");
    }

    #[test]
    fn rgb_rejects_band_with_implausible_aspect_ratio() {
        // Band 50 px wide × 30 px thick → ratio 1.67, way below
        // min_aspect_ratio 4.0. That looks like a square panel, not
        // a long-and-flat lockbar.
        let data = rgb_with_band(200, 200, 150, 180, 75, 124);
        let q = detect_lockbar_rgb(&data, 200, 200, &LockbarRgbParams::default());
        assert!(
            q.is_none(),
            "near-square band should be rejected by aspect ratio"
        );
    }
}
