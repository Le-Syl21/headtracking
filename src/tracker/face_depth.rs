//! Shared face-anchored depth sampling for the Kinect v1 / v2 backends.
//!
//! The Kinect path runs YuNet on the colour frame to find the face bbox,
//! then samples depth pixels inside that bbox (mapped from RGB grid to
//! depth grid by linear rescale), takes the median to absorb sensor noise,
//! and deprojects through the IR intrinsics. This replaces the earlier
//! "closest pixel" heuristic which was tripped by hands, lockbar edges,
//! and stray noise pixels in the play frustum.

/// Min depth we consider valid head distance (mm). Below this is usually
/// noise or the player's hand on the lockbar.
pub const DEPTH_MIN_MM: f32 = 500.0;
/// Max depth we consider plausible (mm). Past this is the back wall.
pub const DEPTH_MAX_MM: f32 = 2_500.0;

/// Pick the largest detected face by bounding-box area. On a pincab the
/// largest face is the one closest to the sensor — almost always the
/// player. Returns `None` if `faces` is empty.
pub fn pick_largest_face(faces: &[face::FaceDetection]) -> Option<&face::FaceDetection> {
    faces.iter().max_by(|a, b| {
        let area_a = a.width * a.height;
        let area_b = b.width * b.height;
        area_a
            .partial_cmp(&area_b)
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

/// Camera intrinsics for the depth (IR) sensor.
#[derive(Clone, Copy)]
pub struct Intrinsics {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
}

/// Sample depth at the face bbox and deproject through the IR intrinsics.
///
/// The (rgb_w, rgb_h) → (depth_w, depth_h) mapping is a naive linear
/// rescale. Kinect v1 has the same 640×480 grid for both sensors so the
/// mapping is identity; Kinect v2 has 1920×1080 RGB and 512×424 depth
/// with a small physical parallax. The window we sample is wide enough
/// (face_size × 0.4 in each direction) that a few-pixel parallax error
/// stays inside the bbox, so the median is unaffected.
///
/// Returns `[x_mm, y_mm, z_mm]` in the IR camera frame, or `None` if
/// fewer than 16 valid depth samples land inside the bbox window (face
/// off-frame, occlusion, or face detected but no depth coverage).
pub fn head_from_face_depth(
    face: &face::FaceDetection,
    rgb_w: u32,
    rgb_h: u32,
    depth: &[f32],
    depth_w: u32,
    depth_h: u32,
    intr: &Intrinsics,
) -> Option<[f32; 3]> {
    if rgb_w == 0 || rgb_h == 0 || depth_w == 0 || depth_h == 0 {
        return None;
    }
    let scale_x = depth_w as f32 / rgb_w as f32;
    let scale_y = depth_h as f32 / rgb_h as f32;
    let face_cx = face.x + face.width * 0.5;
    let face_cy = face.y + face.height * 0.5;
    let depth_cx = face_cx * scale_x;
    let depth_cy = face_cy * scale_y;
    let half_w = ((face.width * 0.4 * scale_x) as i32).clamp(4, 24);
    let half_h = ((face.height * 0.4 * scale_y) as i32).clamp(4, 24);
    let cx = depth_cx as i32;
    let cy = depth_cy as i32;
    let mut samples: Vec<f32> = Vec::with_capacity(((2 * half_w + 1) * (2 * half_h + 1)) as usize);
    for dv in -half_h..=half_h {
        let v = cy + dv;
        if v < 0 || v >= depth_h as i32 {
            continue;
        }
        let row = (v as usize) * depth_w as usize;
        for du in -half_w..=half_w {
            let u = cx + du;
            if u < 0 || u >= depth_w as i32 {
                continue;
            }
            let z = depth[row + u as usize];
            if (DEPTH_MIN_MM..=DEPTH_MAX_MM).contains(&z) {
                samples.push(z);
            }
        }
    }
    if samples.len() < 16 {
        return None;
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let z_mm = samples[samples.len() / 2];

    let zf = f64::from(z_mm);
    let x_mm = (f64::from(depth_cx - intr.cx) * zf / f64::from(intr.fx)) as f32;
    let y_mm = (f64::from(depth_cy - intr.cy) * zf / f64::from(intr.fy)) as f32;

    Some([x_mm, y_mm, z_mm])
}

/// Pack a Kinect v2 BGRX colour frame (4 bytes per pixel, X channel
/// padding) into the RGB888 layout that YuNet expects. The X channel is
/// dropped and the B/R channels are swapped in place.
pub fn bgrx_to_rgb888(bgrx: &[u8]) -> Vec<u8> {
    let pixels = bgrx.len() / 4;
    let mut out = Vec::with_capacity(pixels * 3);
    for chunk in bgrx.chunks_exact(4) {
        out.push(chunk[2]); // R
        out.push(chunk[1]); // G
        out.push(chunk[0]); // B
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fd(x: f32, y: f32, w: f32, h: f32) -> face::FaceDetection {
        face::FaceDetection {
            x,
            y,
            width: w,
            height: h,
            confidence: 0.99,
            right_eye_x: x + w * 0.3,
            right_eye_y: y + h * 0.4,
            left_eye_x: x + w * 0.7,
            left_eye_y: y + h * 0.4,
            nose_x: x + w * 0.5,
            nose_y: y + h * 0.55,
            mouth_right_x: x + w * 0.35,
            mouth_right_y: y + h * 0.75,
            mouth_left_x: x + w * 0.65,
            mouth_left_y: y + h * 0.75,
        }
    }

    #[test]
    fn picks_largest_by_area() {
        let small = fd(0.0, 0.0, 10.0, 10.0);
        let big = fd(0.0, 0.0, 100.0, 100.0);
        let faces = vec![small, big];
        let p = pick_largest_face(&faces).unwrap();
        assert_eq!(p.width, 100.0);
    }

    #[test]
    fn empty_face_list_returns_none() {
        let faces: Vec<face::FaceDetection> = Vec::new();
        assert!(pick_largest_face(&faces).is_none());
    }

    #[test]
    fn head_from_face_depth_centred_face() {
        // 100×100 depth grid; "RGB" same size; face spans the centre,
        // depth is constant 1000 mm everywhere.
        let depth: Vec<f32> = vec![1000.0; 100 * 100];
        let intr = Intrinsics {
            fx: 200.0,
            fy: 200.0,
            cx: 50.0,
            cy: 50.0,
        };
        let face = fd(40.0, 40.0, 20.0, 20.0); // centre at (50, 50)
        let p = head_from_face_depth(&face, 100, 100, &depth, 100, 100, &intr).unwrap();
        // Centred → x_mm and y_mm should be ~0; z_mm = 1000.
        assert!(p[0].abs() < 0.5);
        assert!(p[1].abs() < 0.5);
        assert!((p[2] - 1000.0).abs() < 0.1);
    }

    #[test]
    fn head_from_face_depth_filters_garbage_depths() {
        // All depths zero (out of valid range) — should return None.
        let depth: Vec<f32> = vec![0.0; 100 * 100];
        let intr = Intrinsics {
            fx: 200.0,
            fy: 200.0,
            cx: 50.0,
            cy: 50.0,
        };
        let face = fd(40.0, 40.0, 20.0, 20.0);
        assert!(head_from_face_depth(&face, 100, 100, &depth, 100, 100, &intr).is_none());
    }

    #[test]
    fn bgrx_pack_swaps_channels() {
        // One pixel: B=10, G=20, R=30, X=99 → packed should be R=30, G=20, B=10.
        let bgrx = vec![10, 20, 30, 99];
        let rgb = bgrx_to_rgb888(&bgrx);
        assert_eq!(rgb, vec![30, 20, 10]);
    }
}
