//! **Automatic cabinet calibration** — the core idea that lets head-tracking
//! work from a plain webcam with *zero install and no manual calibration*.
//!
//! The lockbar + the two sidebars detected by the U-seg form a **known planar
//! rectangle of the playfield** seen in perspective. That's enough to recover
//! the camera's focal length **and** its pose relative to the playfield, from
//! the image alone:
//!
//! 1. The two **sidebars** are parallel in 3D → they meet at a **vanishing
//!    point** (the playfield's depth axis). The **lockbar front edge** and the
//!    implicit **back edge** (between the sidebars' far ends) are parallel too →
//!    a second vanishing point (the lateral axis).
//! 2. The playfield is a rectangle, so those two vanishing points are
//!    **orthogonal**: with the principal point at the image centre,
//!    `f² = −(vp₁−p₀)·(vp₂−p₀)` gives the **focal length** — no physical
//!    measurement needed.
//! 3. The two vanishing directions give the playfield's axes in camera space →
//!    the **rotation** `R`. The lockbar's known physical width sets the
//!    **absolute scale** → the **translation** `t`.
//!
//! A head pixel (e.g. the BlazePose nose) then deprojects with `f` and maps
//! into the playfield frame via `(R, t)` — full 3D, from one webcam.
//!
//! Pure `f32`, no external maths crate, so it drops into the plugin as-is.

use super::lockbar::LockbarQuadRgb;

/// Recovered camera calibration relative to the playfield plane.
#[derive(Debug, Clone, Copy)]
pub struct CabCalibration {
    /// Focal length in pixels (shared for x/y — square pixels assumed).
    pub fx: f32,
    /// Principal point (image centre).
    pub cx: f32,
    pub cy: f32,
    /// Playfield axes expressed in **camera** coordinates, as column vectors:
    /// `[lateral (X), normal (Y, out of playfield), depth (Z, into playfield)]`.
    pub r: [[f32; 3]; 3],
    /// Playfield origin (front-edge centre) in **camera** coordinates, mm.
    pub t: [f32; 3],
}

impl CabCalibration {
    /// Map an image pixel at focal-plane depth (via the pinhole) into the
    /// **playfield frame** (mm), given its distance `z_cam` along the camera
    /// optical axis. `z_cam` comes from the head model (shoulder-width or
    /// depth). Returns `[x_lateral, y_normal, z_depth]`.
    pub fn pixel_to_playfield(&self, u: f32, v: f32, z_cam: f32) -> [f32; 3] {
        // Deproject to camera coordinates.
        let cam = [
            (u - self.cx) * z_cam / self.fx,
            (v - self.cy) * z_cam / self.fx,
            z_cam,
        ];
        // Into the playfield frame: pf = Rᵀ · (cam − t).
        let d = [cam[0] - self.t[0], cam[1] - self.t[1], cam[2] - self.t[2]];
        [
            self.r[0][0] * d[0] + self.r[1][0] * d[1] + self.r[2][0] * d[2],
            self.r[0][1] * d[0] + self.r[1][1] * d[1] + self.r[2][1] * d[2],
            self.r[0][2] * d[0] + self.r[1][2] * d[1] + self.r[2][2] * d[2],
        ]
    }
}

/// 2D line as homogeneous coefficients `a·x + b·y + c = 0`, through `p`,`q`.
fn line(p: (f32, f32), q: (f32, f32)) -> [f32; 3] {
    [p.1 - q.1, q.0 - p.0, p.0 * q.1 - q.0 * p.1]
}

/// Intersection of two homogeneous lines. `None` if (near-)parallel — the
/// vanishing point is at infinity, which this first cut can't calibrate from.
fn intersect(l1: [f32; 3], l2: [f32; 3]) -> Option<(f32, f32)> {
    let w = l1[0] * l2[1] - l1[1] * l2[0];
    if w.abs() < 1e-6 {
        return None;
    }
    let x = (l1[1] * l2[2] - l1[2] * l2[1]) / w;
    let y = (l1[2] * l2[0] - l1[0] * l2[2]) / w;
    Some((x, y))
}

fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm3(a: [f32; 3]) -> [f32; 3] {
    let n = dot3(a, a).sqrt().max(1e-9);
    [a[0] / n, a[1] / n, a[2] / n]
}

/// Calibrate from a lockbar quad + its two sidebars. Needs both rails (they
/// give the depth vanishing point). Returns `None` on degenerate geometry
/// (rails parallel in image, or no real focal solution).
pub fn calibrate_from_lockbar(
    quad: &LockbarQuadRgb,
    lockbar_width_mm: f32,
) -> Option<CabCalibration> {
    let lr = quad.left_rail?;
    let rr = quad.right_rail?;
    let f2 = |p: (u32, u32)| (p.0 as f32, p.1 as f32);
    let [tl, tr, _br, _bl] = quad.corners.map(f2);
    let (lr_near, lr_far) = (f2(lr[0]), f2(lr[1]));
    let (rr_near, rr_far) = (f2(rr[0]), f2(rr[1]));

    // Vanishing points.
    let vp_depth = intersect(line(lr_near, lr_far), line(rr_near, rr_far))?;
    let vp_lat = intersect(line(tl, tr), line(lr_far, rr_far))?;

    let cx = quad.frame_width as f32 * 0.5;
    let cy = quad.frame_height as f32 * 0.5;
    let dd = (vp_depth.0 - cx, vp_depth.1 - cy);
    let dl = (vp_lat.0 - cx, vp_lat.1 - cy);

    // Orthogonal vanishing points → focal: f² = −(dd·dl) in the image plane.
    let f2v = -(dd.0 * dl.0 + dd.1 * dl.1);
    if f2v <= 1.0 {
        return None; // no real focal (bad geometry / near-parallel)
    }
    let fx = f2v.sqrt();

    // Playfield axes in camera coords: rays toward each vanishing point.
    let depth_axis = norm3([dd.0, dd.1, fx]);
    let lat_axis = norm3([dl.0, dl.1, fx]);
    // Normal = depth × lateral; flip so it points back toward the camera (−Z).
    let mut normal = norm3(cross3(depth_axis, lat_axis));
    if normal[2] > 0.0 {
        normal = [-normal[0], -normal[1], -normal[2]];
    }

    // Absolute scale from the lockbar's known physical width.
    let front_px = ((tr.0 - tl.0).powi(2) + (tr.1 - tl.1).powi(2))
        .sqrt()
        .max(1.0);
    let z_front = fx * lockbar_width_mm / front_px;
    // Playfield origin = front-edge centre, deprojected at that distance.
    let mid = ((tl.0 + tr.0) * 0.5, (tl.1 + tr.1) * 0.5);
    let t = [
        (mid.0 - cx) * z_front / fx,
        (mid.1 - cy) * z_front / fx,
        z_front,
    ];

    Some(CabCalibration {
        fx,
        cx,
        cy,
        r: [lat_axis, normal, depth_axis],
        t,
    })
}

/// Standard lockbar front-to-back depth (mm) — the metric that turns the
/// lockbar into a *known rectangle* (width × this), so calibration needs no
/// extra measurement even for a perfectly centred camera.
pub const LOCKBAR_DEPTH_MM: f32 = 70.0;

/// Solve `A x = b` for an N×N system by Gaussian elimination with partial
/// pivoting. Returns `None` if singular.
#[allow(clippy::needless_range_loop)] // indices span two distinct rows
fn solve_linear<const N: usize>(mut a: [[f32; N]; N], mut b: [f32; N]) -> Option<[f32; N]> {
    for col in 0..N {
        // Pivot.
        let mut piv = col;
        for r in (col + 1)..N {
            if a[r][col].abs() > a[piv][col].abs() {
                piv = r;
            }
        }
        if a[piv][col].abs() < 1e-9 {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        // Eliminate.
        for r in 0..N {
            if r == col {
                continue;
            }
            let f = a[r][col] / a[col][col];
            for c in col..N {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = [0.0; N];
    for i in 0..N {
        x[i] = b[i] / a[i][i];
    }
    Some(x)
}

/// 4-point homography (metric plane → image), normalised so `h22 = 1`.
fn homography_4pt(src: [(f32, f32); 4], dst: [(f32, f32); 4]) -> Option<[[f32; 3]; 3]> {
    let mut a = [[0.0f32; 8]; 8];
    let mut b = [0.0f32; 8];
    for i in 0..4 {
        let (x, y) = src[i];
        let (u, v) = dst[i];
        a[2 * i] = [x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y];
        b[2 * i] = u;
        a[2 * i + 1] = [0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y];
        b[2 * i + 1] = v;
    }
    let h = solve_linear::<8>(a, b)?;
    Some([[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1.0]])
}

/// Calibrate from the lockbar treated as a metric rectangle (`width` ×
/// [`LOCKBAR_DEPTH_MM`]) via a single-plane homography (Zhang). Works for a
/// centred camera (no vanishing-point degeneracy). Returns `fx` + the camera
/// pose relative to the playfield.
pub fn calibrate_homography(
    quad: &LockbarQuadRgb,
    lockbar_width_mm: f32,
) -> Option<CabCalibration> {
    let f2 = |p: (u32, u32)| (p.0 as f32, p.1 as f32);
    let cx = quad.frame_width as f32 * 0.5;
    let cy = quad.frame_height as f32 * 0.5;
    // Image corners, centred on the principal point.
    let c = quad.corners.map(|p| {
        let (u, v) = f2(p);
        (u - cx, v - cy)
    });
    // Metric rectangle: x = lateral (±W/2), y = depth (0 front, +T back).
    let (hw, td) = (lockbar_width_mm * 0.5, LOCKBAR_DEPTH_MM);
    let src = [(-hw, 0.0), (hw, 0.0), (hw, td), (-hw, td)];
    let h = homography_4pt(src, c)?;
    // Columns h1, h2 = images of the metric X, Y axes.
    let h1 = [h[0][0], h[1][0], h[2][0]];
    let h2 = [h[0][1], h[1][1], h[2][1]];
    let h3 = [h[0][2], h[1][2], h[2][2]];
    // Zhang, centred principal point, ω = diag(w, w, 1), w = 1/f².
    //   h1·ω·h2 = 0            → w = −(h1z·h2z)/(h1x·h2x + h1y·h2y)
    //   h1·ω·h1 = h2·ω·h2      → w = (h2z² − h1z²)/(h1x²+h1y² − h2x²−h2y²)
    let denom_a = h1[0] * h2[0] + h1[1] * h2[1];
    let denom_b = h1[0] * h1[0] + h1[1] * h1[1] - (h2[0] * h2[0] + h2[1] * h2[1]);
    let w = if denom_a.abs() > denom_b.abs() {
        -(h1[2] * h2[2]) / denom_a
    } else {
        (h2[2] * h2[2] - h1[2] * h1[2]) / denom_b
    };
    if !(w.is_finite() && w > 0.0) {
        return None;
    }
    let fx = (1.0 / w).sqrt();

    // Pose: K⁻¹ (centred) = diag(1/f, 1/f, 1).
    let kinv = |v: [f32; 3]| [v[0] / fx, v[1] / fx, v[2]];
    let r1u = kinv(h1);
    let lambda = 1.0 / dot3(r1u, r1u).sqrt().max(1e-9);
    let r1 = [r1u[0] * lambda, r1u[1] * lambda, r1u[2] * lambda];
    let r2u = kinv(h2);
    let r2 = [r2u[0] * lambda, r2u[1] * lambda, r2u[2] * lambda];
    let mut r3 = cross3(r1, r2);
    // playfield normal should face the camera (−Z).
    if r3[2] > 0.0 {
        r3 = [-r3[0], -r3[1], -r3[2]];
    }
    let t3 = kinv(h3);
    let mut t = [t3[0] * lambda, t3[1] * lambda, t3[2] * lambda];
    if t[2] < 0.0 {
        t = [-t[0], -t[1], -t[2]]; // keep the playfield in front
    }
    Some(CabCalibration {
        fx,
        cx,
        cy,
        r: [r1, r3, r2],
        t,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Project a 3D point (camera coords) to a pixel with focal `f`, centre
    /// `(cx,cy)`.
    fn project(p: [f32; 3], f: f32, cx: f32, cy: f32) -> (u32, u32) {
        (
            (cx + p[0] * f / p[2]).round().max(0.0) as u32,
            (cy + p[1] * f / p[2]).round().max(0.0) as u32,
        )
    }

    #[test]
    fn homography_recovers_focal_centered_camera() {
        // The degenerate case for the VP method: camera perfectly centred (no
        // yaw), just pitched down at the lockbar. The homography from the
        // lockbar's known W×70mm rectangle must still recover the focal.
        let (f, cx, cy) = (1000.0f32, 640.0f32, 360.0f32);
        let pitch = 25.0f32.to_radians();
        let (cp, sp) = (pitch.cos(), pitch.sin());
        let w = 520.0f32;
        let td = LOCKBAR_DEPTH_MM;
        let cam_at = [0.0f32, 100.0, 550.0];
        // metric (x lateral, y depth) → 3D camera coords.
        let place = |x: f32, y: f32| {
            let pf = [x, 0.0, y];
            let r = [pf[0], cp * pf[1] - sp * pf[2], sp * pf[1] + cp * pf[2]];
            [r[0] + cam_at[0], r[1] + cam_at[1], r[2] + cam_at[2]]
        };
        let px = |p: [f32; 3]| project(p, f, cx, cy);
        let quad = LockbarQuadRgb {
            frame_width: 1280,
            frame_height: 720,
            corners: [
                px(place(-w / 2.0, 0.0)),
                px(place(w / 2.0, 0.0)),
                px(place(w / 2.0, td)),
                px(place(-w / 2.0, td)),
            ],
            slope_deg: 0.0,
            thickness_px: 8,
            n_inliers_top: 100,
            n_inliers_bottom: 100,
            left_rail: None,
            right_rail: None,
        };
        let cal = calibrate_homography(&quad, w).expect("calibrate");
        assert!(
            (cal.fx - f).abs() / f < 0.06,
            "centred-camera focal recovered {} vs {f}",
            cal.fx
        );
    }

    #[test]
    fn recovers_focal_from_synthetic_playfield() {
        // Ground-truth camera: 1280×720, f = 1000 px, looking down at the
        // playfield tilted 25° about X.
        let (f, cx, cy) = (1000.0f32, 640.0f32, 360.0f32);
        let a = 25.0f32.to_radians();
        let (ca, sa) = (a.cos(), a.sin());
        // Playfield→camera rotation (tilt about camera X): playfield lies in
        // front, tilted so its depth axis dives away and down.
        let rot = |p: [f32; 3]| [p[0], ca * p[1] - sa * p[2], sa * p[1] + ca * p[2]];
        let w = 520.0f32; // lockbar width mm
        let depth = 700.0f32; // playfield opening depth mm
        let cam_at = [0.0, 0.0, 900.0]; // camera 0.9 m back
        // Playfield corners (front centre at origin, +Z into playfield).
        let place = |x: f32, z: f32| {
            let pf = [x, 0.0, z];
            let r = rot(pf);
            [r[0] + cam_at[0], r[1] + cam_at[1], r[2] + cam_at[2]]
        };
        // Camera yaw so the lateral (lockbar) edges converge to a *finite*
        // vanishing point. A perfectly centred camera keeps them parallel →
        // VP at infinity → the degenerate case this VP method can't handle
        // (that needs the homography-with-known-depth upgrade).
        let yaw = 15.0f32.to_radians();
        let (cyw, syw) = (yaw.cos(), yaw.sin());
        let yawed = |p: [f32; 3]| [cyw * p[0] + syw * p[2], p[1], -syw * p[0] + cyw * p[2]];
        let tl = yawed(place(-w / 2.0, 0.0));
        let tr = yawed(place(w / 2.0, 0.0));
        let bl_far = yawed(place(-w / 2.0, depth));
        let br_far = yawed(place(w / 2.0, depth));
        let px = |p: [f32; 3]| project(p, f, cx, cy);

        let quad = LockbarQuadRgb {
            frame_width: 1280,
            frame_height: 720,
            corners: [px(tl), px(tr), px(tr), px(tl)], // thin bar: reuse front
            slope_deg: 0.0,
            thickness_px: 4,
            n_inliers_top: 100,
            n_inliers_bottom: 100,
            left_rail: Some([px(tl), px(bl_far)]),
            right_rail: Some([px(tr), px(br_far)]),
        };

        let cal = calibrate_from_lockbar(&quad, w).expect("calibrate");
        // Focal within a few percent of ground truth.
        assert!(
            (cal.fx - f).abs() / f < 0.05,
            "focal recovered {} vs {f}",
            cal.fx
        );
        // The front-edge centre should land at ~playfield origin (0,0,0).
        let mid = px([
            (tl[0] + tr[0]) * 0.5,
            (tl[1] + tr[1]) * 0.5,
            (tl[2] + tr[2]) * 0.5,
        ]);
        let o = cal.pixel_to_playfield(mid.0 as f32, mid.1 as f32, cal.t[2]);
        assert!(o[0].abs() < 30.0 && o[2].abs() < 30.0, "origin off: {o:?}");
    }
}
