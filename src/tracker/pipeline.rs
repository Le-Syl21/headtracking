//! Shared head-tracking pipeline pieces, ported from the field-validated
//! demo (`tools/headtracking-demo`): the glabella head point on a BlazePose
//! pose, depth sampling for each sensor geometry, and the IR preprocessing
//! that feeds BlazePose. The demo remains the lab bench; this module is the
//! production copy consumed by the plugin backends.

/// Camera intrinsics of the frame the head pixel is deprojected in.
#[derive(Debug, Clone, Copy)]
pub struct Intrinsics {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
}

/// A head fix: pixel in the source frame + metric camera-space position.
#[derive(Debug, Clone, Copy)]
pub struct HeadPixel {
    pub u: u32,
    pub v: u32,
    pub depth_mm: f32,
    pub x_mm: f32,
    pub y_mm: f32,
}

// NOTE: there is deliberately NO plausibility window on head distance any
// more (an earlier 0.5–2.5 m gate is gone). It was a crutch for the old
// head finders, which could land the sampling point off the head entirely
// (the classic failure: sampling "the sidebar" and reading the door 2 m
// behind it). BlazePose's glabella is reliably mid-face, so the ±8 px
// median window stays on the face at any playable distance, and residual
// single-frame spikes are the median gate + One-Euro's job downstream.
// Only sensor validity is filtered: 0 = no reading (v1), ±inf = unmapped
// (v2 bigdepth).

/// Webcams report no intrinsics; assume a nominal focal from the frame
/// width (~55° horizontal FOV, typical for a webcam).
pub const WEBCAM_FX_PER_WIDTH: f32 = 0.9;

/// Stable adult shoulder span used for webcam Z triangulation (mm).
pub const SHOULDER_W_MM: f32 = 400.0;

/// POV "eye" position in source-frame pixels — the **glabella / forehead**
/// (between the eyebrows), a better viewpoint than the nose. `eye_mid` is
/// the mean of the 6 eye landmarks (indices 1..=6); we push up from the eye
/// line, away from the nose, toward the brow.
#[must_use]
pub fn head_center_xy(pose: &blazepose::Pose) -> (f32, f32) {
    let nose = &pose.landmarks[0];
    let (mut ex, mut ey) = (0.0f32, 0.0f32);
    for l in &pose.landmarks[1..=6] {
        ex += l.x;
        ey += l.y;
    }
    let (ex, ey) = (ex / 6.0, ey / 6.0);
    (ex + (ex - nose.x) * 0.4, ey + (ey - nose.y) * 0.4)
}

/// Colour-space width/height of libfreenect2's `bigdepth` map, and the one-
/// row top border it carries (`filter_height_half = 1`), so colour row `y`
/// lives at bigdepth row `y + 1`.
pub const BIGDEPTH_W: usize = 1920;
pub const BIGDEPTH_H: usize = 1080;
pub const BIGDEPTH_ROW_OFFSET: usize = 1;

/// Half-width of the square colour-space window sampled around the head:
/// 17×17 colour pixels, wide enough for a stable median at cabinet distance
/// and small enough that projecting depth into it is a rounding error.
pub const HEAD_WINDOW_HALF: i32 = 8;

/// Head pixel from a BlazePose landmark sampled in **colour space**: the
/// colour-space depth **window** around the head, as filled by
/// `Registration::depth_window`.
///
/// This is the accurate path for the Kinect v2: the landmark is already in
/// colour pixels and the window is depth expressed in those same pixels, so
/// no cross-sensor mapping is needed at all. Deprojection therefore uses the
/// **colour** intrinsics — passing the IR ones here would reintroduce the
/// very error the registration removes.
///
/// Unmapped pixels come back `+inf` from libfreenect2 (not `0`), so the
/// validity gate checks `is_finite()` on top of the `> 0` no-reading test.
///
/// `window` is row-major `(2*half+1)²`, centred on the pose's own head point,
/// so the caller must have asked for the window at `head_center_xy(pose)`
/// rounded down — which is what `depth_window` is given.
#[must_use]
pub fn head_pixel_from_window(
    pose: &blazepose::Pose,
    window: &[f32],
    half: i32,
    color: &Intrinsics,
    min_samples: usize,
) -> Option<HeadPixel> {
    let side = (2 * half + 1).max(0) as usize;
    if window.len() != side * side || color.fx <= 0.0 {
        return None;
    }
    let (hx, hy) = head_center_xy(pose);
    let mut samples: Vec<f32> = window
        .iter()
        .copied()
        .filter(|z| z.is_finite() && *z > 0.0)
        .collect();
    if samples.len() < min_samples.max(1) {
        return None;
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let depth_mm = samples[samples.len() / 2];
    let zf = f64::from(depth_mm);
    Some(HeadPixel {
        u: hx.max(0.0) as u32,
        v: hy.max(0.0) as u32,
        depth_mm,
        x_mm: (f64::from(hx - color.cx) * zf / f64::from(color.fx)) as f32,
        y_mm: (f64::from(hy - color.cy) * zf / f64::from(color.fy)) as f32,
    })
}

/// Head pixel from a BlazePose pose + a raw depth grid. Generic over the
/// depth sample type so the v1's native `u16` grid is sampled in place —
/// the 17×17 window widens per-sample instead of paying a full-frame
/// `u16→f32` copy (1.2 MB at 30 Hz) up front. `f32: From<T>` covers both
/// `u16` (v1) and `f32` (v2) losslessly.
///
/// `rgb` is the frame the pose was detected in; the head point rescales
/// linearly into the depth grid (identity when the pose was detected on
/// the IR stream — IR and depth share the sensor and the grid).
#[must_use]
pub fn head_pixel_from_pose_depth<T: Copy>(
    pose: &blazepose::Pose,
    rgb: (u32, u32),
    depth_data: &[T],
    depth_dims: (u32, u32),
    intr: &Intrinsics,
    min_samples: usize,
) -> Option<HeadPixel>
where
    f32: From<T>,
{
    let (rgb_w, rgb_h) = rgb;
    let (depth_w, depth_h) = depth_dims;
    if rgb_w == 0 || rgb_h == 0 || depth_w == 0 || depth_h == 0 {
        return None;
    }
    let (hx, hy) = head_center_xy(pose);
    let depth_cx = hx * depth_w as f32 / rgb_w as f32;
    let depth_cy = hy * depth_h as f32 / rgb_h as f32;
    let (cx, cy) = (depth_cx as i32, depth_cy as i32);
    let half = 8i32;
    let mut samples: Vec<f32> = Vec::new();
    for dv in -half..=half {
        let v = cy + dv;
        if v < 0 || v >= depth_h as i32 {
            continue;
        }
        let row = v as usize * depth_w as usize;
        for du in -half..=half {
            let u = cx + du;
            if u < 0 || u >= depth_w as i32 {
                continue;
            }
            let z = f32::from(depth_data[row + u as usize]);
            if z.is_finite() && z > 0.0 {
                samples.push(z);
            }
        }
    }
    if samples.len() < min_samples.max(1) {
        return None;
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let depth_mm = samples[samples.len() / 2];
    let zf = f64::from(depth_mm);
    Some(HeadPixel {
        u: depth_cx.max(0.0) as u32,
        v: depth_cy.max(0.0) as u32,
        depth_mm,
        x_mm: (f64::from(depth_cx - intr.cx) * zf / f64::from(intr.fx)) as f32,
        y_mm: (f64::from(depth_cy - intr.cy) * zf / f64::from(intr.fy)) as f32,
    })
}

/// Head pixel from a BlazePose pose on a **webcam** frame (no depth):
/// triangulate distance from the shoulder width — a stable ~0.40 m span, so
/// `Z = fx · W / w_px` — then deproject the glabella. `None` if the
/// shoulders aren't confidently seen. `focal_px = 0` selects the nominal
/// [`WEBCAM_FX_PER_WIDTH`] focal.
#[must_use]
pub fn head_pixel_from_pose_webcam(
    pose: &blazepose::Pose,
    rgb_w: u32,
    rgb_h: u32,
    focal_px: f32,
) -> Option<HeadPixel> {
    use blazepose::idx::{LEFT_SHOULDER, RIGHT_SHOULDER};
    if rgb_w == 0 || rgb_h == 0 {
        return None;
    }
    let (ls, rs) = (
        &pose.landmarks[LEFT_SHOULDER],
        &pose.landmarks[RIGHT_SHOULDER],
    );
    let (hx, hy) = head_center_xy(pose);
    if ls.visibility < 0.5 || rs.visibility < 0.5 {
        return None;
    }
    let w_px = ((ls.x - rs.x).powi(2) + (ls.y - rs.y).powi(2)).sqrt();
    if w_px < 1.0 {
        return None;
    }
    let fx = if focal_px > 0.0 {
        focal_px
    } else {
        rgb_w as f32 * WEBCAM_FX_PER_WIDTH
    };
    let cx = rgb_w as f32 * 0.5;
    let cy = rgb_h as f32 * 0.5;
    let depth_mm = fx * SHOULDER_W_MM / w_px;
    let zf = f64::from(depth_mm);
    Some(HeadPixel {
        u: hx.max(0.0) as u32,
        v: hy.max(0.0) as u32,
        depth_mm,
        x_mm: (f64::from(hx - cx) * zf / f64::from(fx)) as f32,
        y_mm: (f64::from(hy - cy) * zf / f64::from(fx)) as f32,
    })
}

/// Stretch a raw 16-bit IR frame to full 8-bit contrast (min/max
/// auto-level). `zero_is_hole` keeps depth-style zero pixels black instead
/// of letting them drag the level range.
#[must_use]
pub fn autolevel_gray8_raw(samples: &[u16], zero_is_hole: bool) -> Vec<u8> {
    let (mut lo, mut hi) = (u16::MAX, 0u16);
    for &v in samples {
        if zero_is_hole && v == 0 {
            continue;
        }
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if hi < lo {
        lo = 0;
        hi = 0;
    }
    let span = f32::from(hi.saturating_sub(lo)).max(1.0);
    samples
        .iter()
        .map(|&v| {
            if zero_is_hole && v == 0 {
                0
            } else {
                ((f32::from(v.saturating_sub(lo)) / span) * 255.0).clamp(0.0, 255.0) as u8
            }
        })
        .collect()
}

/// Expand a gray8 plane to RGB888 (BlazePose wants 3 channels).
#[must_use]
pub fn gray8_to_rgb888(gray: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(gray.len() * 3);
    for &v in gray {
        out.extend_from_slice(&[v, v, v]);
    }
    out
}

/// Default number of valid depth samples required inside the head window
/// before trusting the median (the demo's non-bypass value).
pub const DEPTH_MIN_SAMPLES: usize = 16;

/// libfreenect2's colour stream is BGRX; BlazePose wants RGB888.
#[must_use]
pub fn bgrx_to_rgb888(bgrx: &[u8]) -> Vec<u8> {
    let (src, _) = bgrx.as_chunks::<4>();
    // Sized up front and written through fixed-width chunks rather than
    // pushed a byte at a time: at 1080p that is 6.2 M pushes a frame, and the
    // bounds-checked push loop refuses to vectorise.
    let mut out = vec![0u8; src.len() * 3];
    let (dst, _) = out.as_chunks_mut::<3>();
    for (d, s) in dst.iter_mut().zip(src) {
        *d = [s[2], s[1], s[0]]; // R, G, B from B, G, R, X
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose_with(points: &[(usize, f32, f32, f32)]) -> blazepose::Pose {
        let mut pose = blazepose::Pose {
            landmarks: std::array::from_fn(|_| blazepose::Landmark {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                visibility: 1.0,
                presence: 1.0,
            }),
            presence: 1.0,
        };
        for &(i, x, y, vis) in points {
            pose.landmarks[i].x = x;
            pose.landmarks[i].y = y;
            pose.landmarks[i].visibility = vis;
        }
        pose
    }

    #[test]
    fn glabella_sits_above_the_eye_line() {
        // Nose below the eyes: the glabella must land above the eye mean,
        // pushed away from the nose.
        let mut pts = vec![(0usize, 100.0f32, 110.0f32, 1.0f32)];
        for i in 1..=6 {
            pts.push((i, 100.0, 100.0, 1.0));
        }
        let (gx, gy) = head_center_xy(&pose_with(&pts));
        assert!((gx - 100.0).abs() < f32::EPSILON);
        assert!(gy < 100.0, "glabella {gy} must be above the eye line 100");
    }

    #[test]
    fn depth_median_rejects_out_of_range() {
        let mut pts = vec![(0usize, 32.0f32, 32.0f32, 1.0f32)];
        for i in 1..=6 {
            pts.push((i, 32.0, 32.0, 1.0));
        }
        let pose = pose_with(&pts);
        let mut depth = vec![0u16; 64 * 64];
        for v in depth.iter_mut() {
            *v = 1200;
        }
        let intr = Intrinsics {
            fx: 100.0,
            fy: 100.0,
            cx: 32.0,
            cy: 32.0,
        };
        let h = head_pixel_from_pose_depth(&pose, (64, 64), &depth, (64, 64), &intr, 8)
            .expect("median over a uniform plane");
        assert!((h.depth_mm - 1200.0).abs() < f32::EPSILON);
        // All-zero grid (holes) → no fix.
        let holes = vec![0u16; 64 * 64];
        assert!(head_pixel_from_pose_depth(&pose, (64, 64), &holes, (64, 64), &intr, 8).is_none());
    }

    #[test]
    fn webcam_distance_scales_inverse_to_shoulder_width() {
        use blazepose::idx::{LEFT_SHOULDER, RIGHT_SHOULDER};
        let near = pose_with(&[
            (0, 320.0, 200.0, 1.0),
            (LEFT_SHOULDER, 120.0, 400.0, 1.0),
            (RIGHT_SHOULDER, 520.0, 400.0, 1.0),
        ]);
        let far = pose_with(&[
            (0, 320.0, 200.0, 1.0),
            (LEFT_SHOULDER, 220.0, 400.0, 1.0),
            (RIGHT_SHOULDER, 420.0, 400.0, 1.0),
        ]);
        let hn = head_pixel_from_pose_webcam(&near, 640, 480, 0.0).unwrap();
        let hf = head_pixel_from_pose_webcam(&far, 640, 480, 0.0).unwrap();
        assert!(
            (hf.depth_mm / hn.depth_mm - 2.0).abs() < 1e-3,
            "half the pixel span must read twice the distance"
        );
    }
}
