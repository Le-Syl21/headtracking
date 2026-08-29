//! What the signs of [`anchor::camera_pose`] actually mean.
//!
//! The golden tests pin the *values* against the validated Python harness;
//! this one pins the *conventions*, by building a cabinet in 3D, placing a
//! camera at a known pose, projecting through a pinhole and checking what
//! comes back. Every user-facing sentence in the demo and the plugin ("the
//! camera is 3 cm left of centre") is written from these assertions, so a
//! sign that flips has to break a test rather than quietly mislead a player.
//!
//! World frame, right-handed: `+X` to the right as the camera sees it, `+Y`
//! up out of the playfield, `+Z` from the lockbar toward the backbox. The
//! camera sits in front of the lockbar (negative Z), above the playfield
//! plane, looking toward `+Z` — the framing the anchor model was trained on
//! (lockbar near and low in the image, rails receding upward).

use anchor::{CameraIntrinsics, LineSeg, camera_pose, geometry_from_lines};

const LOCKBAR_MM: f32 = 610.0;
const HALF: f64 = LOCKBAR_MM as f64 / 2.0;
/// Lockbar depth: the player edge sits this much toward the camera.
const BAR_DEPTH: f64 = 60.0;
const RAIL_LEN: f64 = 1400.0;

const W: u32 = 1920;
const H: u32 = 1080;
const FX: f64 = 1081.0;

type V3 = (f64, f64, f64);

fn sub(a: V3, b: V3) -> V3 {
    (a.0 - b.0, a.1 - b.1, a.2 - b.2)
}
fn dot(a: V3, b: V3) -> f64 {
    a.0 * b.0 + a.1 * b.1 + a.2 * b.2
}
fn scale(a: V3, k: f64) -> V3 {
    (a.0 * k, a.1 * k, a.2 * k)
}
fn add(a: V3, b: V3) -> V3 {
    (a.0 + b.0, a.1 + b.1, a.2 + b.2)
}

/// Camera placement in world terms, plus the cabinet it looks at.
#[derive(Clone, Copy)]
struct Cam {
    /// Playfield inclination, degrees: the rails rise by this much on their
    /// way to the backbox. The lockbar is the hinge, so its two edges stay
    /// along `+X` whatever this is.
    incline_deg: f64,
    /// Position: `x` right of the cabinet centreline, `y` above the playfield
    /// plane, `z` in front of the lockbar (negative = on the player side).
    pos: V3,
    /// Turn about the playfield normal, degrees. Positive turns the optical
    /// axis toward `+X` (to the camera's right).
    yaw_deg: f64,
    /// Nose-down tilt, degrees. Positive looks down at the playfield.
    pitch_deg: f64,
    /// Rotation about the optical axis, degrees. Positive rolls the camera
    /// clockwise as seen from behind it (the horizon tips anticlockwise in
    /// the image).
    roll_deg: f64,
}

impl Cam {
    /// Project a world point to pixels. `None` if it lands behind the camera.
    fn project(self, p: V3) -> Option<(f32, f32)> {
        let (sy, cy) = self.yaw_deg.to_radians().sin_cos();
        let (sp, cp) = self.pitch_deg.to_radians().sin_cos();
        let (sr, cr) = self.roll_deg.to_radians().sin_cos();
        // Yaw about world +Y, then pitch about the camera's own right axis,
        // then roll about the optical axis.
        let f1 = (sy, 0.0, cy);
        let r1 = (cy, 0.0, -sy);
        let u1 = (0.0, 1.0, 0.0);
        let f2 = add(scale(f1, cp), scale(u1, -sp));
        let u2 = add(scale(f1, sp), scale(u1, cp));
        let r3 = add(scale(r1, cr), scale(u2, sr));
        let u3 = add(scale(r1, -sr), scale(u2, cr));
        let d = sub(p, self.pos);
        let (xc, yc, zc) = (dot(d, r3), -dot(d, u3), dot(d, f2));
        if zc <= 1.0 {
            return None;
        }
        Some((
            (FX * xc / zc + f64::from(W) * 0.5) as f32,
            (FX * yc / zc + f64::from(H) * 0.5) as f32,
        ))
    }

    /// Far end of a rail, risen by the playfield inclination.
    fn rail_far(self, x: f64) -> V3 {
        let (s, c) = self.incline_deg.to_radians().sin_cos();
        (x, RAIL_LEN * s, RAIL_LEN * c)
    }

    /// The four annotated lines, as the annotator would have drawn them.
    fn lines(self) -> (LineSeg, LineSeg, LineSeg, LineSeg) {
        let seg = |a: V3, b: V3| -> LineSeg {
            let pa = self.project(a).expect("point a in front of the camera");
            let pb = self.project(b).expect("point b in front of the camera");
            [[pa.0, pa.1], [pb.0, pb.1]]
        };
        (
            seg((-HALF, 0.0, 0.0), self.rail_far(-HALF)),
            seg((HALF, 0.0, 0.0), self.rail_far(HALF)),
            seg((-HALF, 0.0, -BAR_DEPTH), (HALF, 0.0, -BAR_DEPTH)),
            seg((-HALF, 0.0, 0.0), (HALF, 0.0, 0.0)),
        )
    }

    fn recovered(self) -> anchor::CameraPose {
        let (sl, sr, lp, ls) = self.lines();
        let geom = geometry_from_lines(sl, sr, lp, ls, W, H).expect("valid line set");
        let intr = CameraIntrinsics {
            fx: FX as f32,
            fy: FX as f32,
            cx: W as f32 * 0.5,
            cy: H as f32 * 0.5,
        };
        camera_pose(&geom, &intr, LOCKBAR_MM).expect("pose recoverable")
    }
}

/// A flat cabinet — the simplest thing that isolates one convention at a
/// time. Real playfields slope; see [`sloped`].
fn base() -> Cam {
    Cam {
        incline_deg: 0.0,
        pos: (0.0, 760.0, -1270.0),
        yaw_deg: 0.0,
        pitch_deg: 0.0,
        roll_deg: 0.0,
    }
}

/// A real cabinet: 6.5 degrees of playfield slope, the usual pincab figure.
fn sloped() -> Cam {
    Cam {
        incline_deg: 6.5,
        ..base()
    }
}

/// A centred, level camera reads as zeros — and the two distances come back
/// as placed, which is what makes the rest of the signs meaningful.
#[test]
fn a_level_centred_camera_reads_as_zero() {
    let p = base().recovered();
    assert!(p.lateral_mm.abs() < 0.5, "lateral {}", p.lateral_mm);
    assert!(p.yaw_deg.abs() < 0.05, "yaw {}", p.yaw_deg);
    assert!(p.roll_deg.abs() < 0.05, "roll {}", p.roll_deg);
    assert!(p.pitch_deg.abs() < 0.05, "pitch {}", p.pitch_deg);
    assert!((p.height_mm - 760.0).abs() < 1.0, "height {}", p.height_mm);
    assert!(
        (p.distance_mm - 1270.0).abs() < 1.0,
        "distance {}",
        p.distance_mm
    );
    assert!(
        (p.rect_angle_deg - 90.0).abs() < 0.05,
        "rect angle {}",
        p.rect_angle_deg
    );
}

/// `lateral_mm` is the camera's own offset, signed to the right **as the
/// camera sees it**: a camera 30 cm to the left of the cabinet centreline
/// reads -300.
#[test]
fn lateral_is_positive_when_the_camera_sits_right_of_centre() {
    let mut right = base();
    right.pos.0 = 300.0;
    assert!((right.recovered().lateral_mm - 300.0).abs() < 1.0);

    let mut left = base();
    left.pos.0 = -300.0;
    assert!((left.recovered().lateral_mm + 300.0).abs() < 1.0);
}

/// `height_mm` is measured from the playfield plane, and never negative.
#[test]
fn height_is_measured_from_the_playfield_plane() {
    let mut low = base();
    low.pos.1 = 400.0;
    assert!((low.recovered().height_mm - 400.0).abs() < 1.0);
}

/// `pitch_deg` is nose-down and measured against the playfield **plane**:
/// 0 looks along it, 90 straight down at it.
#[test]
fn pitch_is_positive_looking_down() {
    for applied in [10.0, 25.0] {
        let mut c = base();
        c.pitch_deg = applied;
        let got = c.recovered().pitch_deg;
        assert!((got - applied as f32).abs() < 0.05, "{applied} -> {got}");
    }
}

/// `distance_mm` is the lockbar's depth **along the optical axis**, not a
/// horizontal distance: tilting the camera down without moving it makes the
/// number grow, because the lockbar moves away along the axis.
#[test]
fn distance_follows_the_optical_axis() {
    let level = base().recovered().distance_mm;
    let mut tilted = base();
    tilted.pitch_deg = 25.0;
    assert!(
        tilted.recovered().distance_mm > level + 100.0,
        "tilting must lengthen the axis distance"
    );
}

/// `yaw_deg` is positive when the camera is aimed to **its own left** — the
/// opposite side to a positive `lateral_mm`, which is why an off-centre
/// camera aimed back at the cabinet reports the two with opposite signs.
#[test]
fn yaw_is_positive_when_the_camera_is_aimed_left() {
    let mut aimed_right = base();
    aimed_right.yaw_deg = 8.0; // optical axis toward +X, i.e. to the right
    assert!((aimed_right.recovered().yaw_deg + 8.0).abs() < 0.05);

    let mut aimed_left = base();
    aimed_left.yaw_deg = -8.0;
    assert!((aimed_left.recovered().yaw_deg - 8.0).abs() < 0.05);
}

/// `roll_deg` is positive when the lockbar's **left end appears higher** in
/// the image — the one thing about roll a player can check by eye.
#[test]
fn roll_is_positive_when_the_left_end_of_the_lockbar_looks_higher() {
    let mut c = base();
    c.roll_deg = 6.0;
    let pose = c.recovered();
    assert!((pose.roll_deg - 6.0).abs() < 0.05, "roll {}", pose.roll_deg);
    // Smaller v = higher in the image.
    let left = c.project((-HALF, 0.0, 0.0)).unwrap();
    let right = c.project((HALF, 0.0, 0.0)).unwrap();
    assert!(
        left.1 < right.1,
        "positive roll must lift the left end: left v={} right v={}",
        left.1,
        right.1
    );
}

/// The self-test compares the rail and lockbar directions in 3D, and those
/// are reconstructed through the focal — so a wrong focal shows up as an
/// out-of-square cabinet.
///
/// **But only when the camera is off-axis.** Head-on and level, the lockbar
/// edges stay parallel in the image: their vanishing point runs off to
/// infinity, the width direction becomes a pure image direction that the
/// focal never touches, and the angle reads 90 whatever focal it is handed.
/// A cabinet-centred camera therefore gets no free focal check — which is
/// exactly the configuration a tidy installation ends up in.
#[test]
fn rect_angle_detects_a_wrong_focal_when_the_camera_is_off_axis() {
    let mut c = base();
    c.pos.0 = 400.0;
    c.yaw_deg = -12.0;
    c.pitch_deg = 12.0;
    let (sl, sr, lp, ls) = c.lines();
    let geom = geometry_from_lines(sl, sr, lp, ls, W, H).expect("valid line set");
    let wrong = CameraIntrinsics {
        fx: (FX * 1.2) as f32,
        fy: (FX * 1.2) as f32,
        cx: W as f32 * 0.5,
        cy: H as f32 * 0.5,
    };
    let pose = camera_pose(&geom, &wrong, LOCKBAR_MM).expect("pose recoverable");
    assert!(
        (pose.rect_angle_deg - 90.0).abs() > 2.0,
        "a 20% focal error must show up as out-of-square, got {}",
        pose.rect_angle_deg
    );
    // The same geometry with the right focal is square, so the assertion
    // above is about the focal and not about the viewpoint.
    let right = CameraIntrinsics {
        fx: FX as f32,
        fy: FX as f32,
        cx: W as f32 * 0.5,
        cy: H as f32 * 0.5,
    };
    let ok = camera_pose(&geom, &right, LOCKBAR_MM).expect("pose recoverable");
    assert!(
        (ok.rect_angle_deg - 90.0).abs() < 0.1,
        "correct focal must read square, got {}",
        ok.rect_angle_deg
    );
}

/// The counter-case on a flat table, stated as a test so it cannot be
/// forgotten when writing the help text. The sloped-table version — which is
/// the one that matters, and the one people expect to behave differently — is
/// `the_square_check_is_blind_at_zero_yaw_even_on_a_sloped_table`.
#[test]
fn rect_angle_is_blind_to_the_focal_when_aimed_down_the_cabinet_axis() {
    let c = base();
    let (sl, sr, lp, ls) = c.lines();
    let geom = geometry_from_lines(sl, sr, lp, ls, W, H).expect("valid line set");
    let wrong = CameraIntrinsics {
        fx: (FX * 1.2) as f32,
        fy: (FX * 1.2) as f32,
        cx: W as f32 * 0.5,
        cy: H as f32 * 0.5,
    };
    let pose = camera_pose(&geom, &wrong, LOCKBAR_MM).expect("pose recoverable");
    assert!(
        (pose.rect_angle_deg - 90.0).abs() < 0.1,
        "expected the degenerate head-on case to read square, got {}",
        pose.rect_angle_deg
    );
}

/// The player-facing wording has to agree with the signs it describes. If the
/// convention ever flips, this fails next to the test that pinned it.
#[test]
fn the_description_reads_the_signs_the_way_they_are_defined() {
    let mut left_of_centre = base();
    left_of_centre.pos.0 = -300.0;
    let text = left_of_centre.recovered().describe();
    assert!(text.contains("30 cm left of centre"), "{text}");

    let mut aimed_right = base();
    aimed_right.yaw_deg = 8.0; // optical axis toward +X
    let text = aimed_right.recovered().describe();
    assert!(text.contains("to its right"), "{text}");

    let mut looking_down = base();
    looking_down.pitch_deg = 20.0;
    let text = looking_down.recovered().describe();
    assert!(
        text.contains("looking 20") && text.contains("down"),
        "{text}"
    );

    // A camera on the centreline says so rather than printing "0 cm left".
    let centred = base().recovered().describe();
    assert!(centred.contains("on the centreline"), "{centred}");
    assert!(!centred.contains("cm left"), "{centred}");
}

/// `pitch_deg` is measured against the **playfield**, not against the
/// horizontal — so a level camera looking at a sloped table does not read
/// zero. It reads the slope. This is the single most confusing number in the
/// read-out, and the help text has to say it.
#[test]
fn pitch_is_relative_to_the_playfield_not_the_horizon() {
    let level_camera_flat_table = base().recovered().pitch_deg;
    let level_camera_sloped_table = sloped().recovered().pitch_deg;
    assert!(level_camera_flat_table.abs() < 0.05);
    assert!(
        (level_camera_sloped_table - 6.5).abs() < 0.2,
        "a level camera on a 6.5 deg table should read 6.5, got {level_camera_sloped_table}"
    );
}

/// The playfield slope does **not** rescue the square check.
///
/// It is tempting to think it would: the rails climb, so surely nothing is
/// parallel any more. But the two lockbar edges both run across the cabinet,
/// and the slope is a rotation *about* that very direction — it cannot make
/// them converge. Their vanishing point stays at infinity, the width
/// direction stays a pure image direction the focal never touches, and the
/// rails' own vanishing point stays on the vertical through the principal
/// point, so the reconstructed rail direction is perpendicular to it whatever
/// focal it is built with. The check is governed by **yaw**, and by nothing
/// else.
#[test]
fn the_square_check_is_blind_at_zero_yaw_even_on_a_sloped_table() {
    let wrong = CameraIntrinsics {
        fx: (FX * 1.25) as f32,
        fy: (FX * 1.25) as f32,
        cx: W as f32 * 0.5,
        cy: H as f32 * 0.5,
    };
    // Slope, camera pitch, lateral offset and roll: none of them make the
    // check fire while the camera is aimed down the cabinet's axis.
    for c in [
        sloped(),
        Cam {
            pitch_deg: 15.0,
            ..sloped()
        },
        Cam {
            pos: (450.0, 760.0, -1270.0),
            ..sloped()
        },
        Cam {
            roll_deg: 7.0,
            ..sloped()
        },
    ] {
        let (sl, sr, lp, ls) = c.lines();
        let geom = geometry_from_lines(sl, sr, lp, ls, W, H).expect("valid line set");
        let pose = camera_pose(&geom, &wrong, LOCKBAR_MM).expect("pose recoverable");
        assert!(
            (pose.rect_angle_deg - 90.0).abs() < 0.2,
            "expected the check to stay blind at zero yaw, got {} for {:?}",
            pose.rect_angle_deg,
            (c.incline_deg, c.pitch_deg, c.pos.0, c.roll_deg)
        );
    }

    // Turn the camera a few degrees off the axis and the same wrong focal
    // shows up immediately.
    let c = Cam {
        yaw_deg: -12.0,
        ..sloped()
    };
    let (sl, sr, lp, ls) = c.lines();
    let geom = geometry_from_lines(sl, sr, lp, ls, W, H).expect("valid line set");
    let pose = camera_pose(&geom, &wrong, LOCKBAR_MM).expect("pose recoverable");
    assert!(
        (pose.rect_angle_deg - 90.0).abs() > 2.0,
        "off-axis, a 25% focal error must show, got {}",
        pose.rect_angle_deg
    );
}

/// Playfield pitch and camera pitch differ by exactly the slope, so a UI that
/// knows the table's inclination can show the reader the angle they can
/// actually check with a spirit level.
#[test]
fn playfield_pitch_minus_the_slope_is_the_pitch_against_the_horizon() {
    for nose_down in [0.0, 8.0, 15.0] {
        let c = Cam {
            pitch_deg: nose_down,
            ..sloped()
        };
        let got = c.recovered().pitch_deg;
        let expected = nose_down as f32 + 6.5;
        assert!(
            (got - expected).abs() < 0.2,
            "nose-down {nose_down} on a 6.5 deg table should read {expected}, got {got}"
        );
    }
}

// ---------------------------------------------------------------- flattening

/// Apply a row-major 3x3 to a pixel.
fn apply(m: &[f64; 9], x: f64, y: f64) -> (f64, f64) {
    let w = m[6] * x + m[7] * y + m[8];
    (
        (m[0] * x + m[1] * y + m[2]) / w,
        (m[3] * x + m[4] * y + m[5]) / w,
    )
}

fn angle_between(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dot = a.0 * b.0 + a.1 * b.1;
    let la = (a.0 * a.0 + a.1 * a.1).sqrt();
    let lb = (b.0 * b.0 + b.1 * b.1).sqrt();
    (dot / (la * lb)).clamp(-1.0, 1.0).acos().to_degrees()
}

/// The rectified view has to put the cabinet back into a rectangle: the two
/// rails parallel, the lockbar square to them, and the width-to-length ratio
/// the one the cabinet really has. That is what makes it a *check* and not a
/// picture — and it can only come out right if the focal and the detected
/// lines are right, because the homography is built from those and not from
/// the corners it is being judged on.
#[test]
fn flattening_puts_the_cabinet_back_in_a_rectangle() {
    let c = Cam {
        yaw_deg: -9.0,
        pitch_deg: 14.0,
        pos: (250.0, 760.0, -1270.0),
        ..sloped()
    };
    let (sl, sr, lp, ls) = c.lines();
    let geom = geometry_from_lines(sl, sr, lp, ls, W, H).expect("valid line set");
    let intr = CameraIntrinsics {
        fx: FX as f32,
        fy: FX as f32,
        cx: W as f32 * 0.5,
        cy: H as f32 * 0.5,
    };
    // Destination -> source; invert it to follow known points forward.
    let flat = anchor::flatten_homography(&geom, &intr, W, H).expect("homography");
    let inv = flat.dst_to_src;
    let fwd = {
        // 3x3 inverse, same adjugate as the crate's own.
        let m = inv;
        let co = [
            m[4] * m[8] - m[5] * m[7],
            m[5] * m[6] - m[3] * m[8],
            m[3] * m[7] - m[4] * m[6],
            m[2] * m[7] - m[1] * m[8],
            m[0] * m[8] - m[2] * m[6],
            m[1] * m[6] - m[0] * m[7],
            m[1] * m[5] - m[2] * m[4],
            m[2] * m[3] - m[0] * m[5],
            m[0] * m[4] - m[1] * m[3],
        ];
        let det = m[0] * co[0] + m[1] * co[1] + m[2] * co[2];
        [
            co[0] / det,
            co[3] / det,
            co[6] / det,
            co[1] / det,
            co[4] / det,
            co[7] / det,
            co[2] / det,
            co[5] / det,
            co[8] / det,
        ]
    };

    // The four cabinet corners we know metrically: the lockbar's screen edge,
    // and the far end of each rail.
    let px = |p: V3| {
        let q = c.project(p).expect("in front of the camera");
        apply(&fwd, f64::from(q.0), f64::from(q.1))
    };
    let near_l = px((-HALF, 0.0, 0.0));
    let near_r = px((HALF, 0.0, 0.0));
    let far_l = px(c.rail_far(-HALF));
    let far_r = px(c.rail_far(HALF));

    let bar = (near_r.0 - near_l.0, near_r.1 - near_l.1);
    let rail_l = (far_l.0 - near_l.0, far_l.1 - near_l.1);
    let rail_r = (far_r.0 - near_r.0, far_r.1 - near_r.1);

    assert!(
        angle_between(rail_l, rail_r) < 0.5,
        "rails must come out parallel, got {}\u{b0}",
        angle_between(rail_l, rail_r)
    );
    assert!(
        (angle_between(bar, rail_l) - 90.0).abs() < 0.5,
        "lockbar must come out square to the rails, got {}\u{b0}",
        angle_between(bar, rail_l)
    );

    // Aspect: 610 mm across against 1400 mm of rail, preserved by a
    // rectification that is a similarity on the plane.
    let bar_len = (bar.0 * bar.0 + bar.1 * bar.1).sqrt();
    let rail_len = (rail_l.0 * rail_l.0 + rail_l.1 * rail_l.1).sqrt();
    let got = bar_len / rail_len;
    let want = f64::from(LOCKBAR_MM) / RAIL_LEN;
    assert!(
        (got - want).abs() / want < 0.02,
        "aspect {got:.4} vs {want:.4}"
    );

    // The lockbar sits across the bottom and the playfield runs up the frame.
    assert!(near_l.1 > far_l.1, "cabinet must run upward in the view");
    assert!(near_l.0 < near_r.0, "left must stay on the left");
}

/// A wrong focal has to *show*, otherwise the view is decoration. Feeding the
/// rectification a focal 25 % out must skew the cabinet out of square by a
/// margin nobody could miss on screen.
#[test]
fn a_wrong_focal_visibly_skews_the_flattened_cabinet() {
    let c = Cam {
        yaw_deg: -9.0,
        pitch_deg: 14.0,
        pos: (250.0, 760.0, -1270.0),
        ..sloped()
    };
    let (sl, sr, lp, ls) = c.lines();
    let geom = geometry_from_lines(sl, sr, lp, ls, W, H).expect("valid line set");
    let wrong = CameraIntrinsics {
        fx: (FX * 1.25) as f32,
        fy: (FX * 1.25) as f32,
        cx: W as f32 * 0.5,
        cy: H as f32 * 0.5,
    };
    let pose = camera_pose(&geom, &wrong, LOCKBAR_MM).expect("pose");
    // Same defect the number reports, now as a picture: if the reconstruction
    // is out of square, the flattened cabinet is a parallelogram.
    assert!(
        (pose.rect_angle_deg - 90.0).abs() > 2.0,
        "precondition: the wrong focal must be out of square"
    );
    assert!(
        anchor::flatten_homography(&geom, &wrong, W, H).is_some(),
        "the view must still build — the point is to SEE the error"
    );
}

/// The guides have to sit on the lockbar the warp actually drew, not on where
/// the fit was assumed to put it.
#[test]
fn the_flattened_lockbar_ends_are_where_the_guides_go() {
    let c = Cam {
        yaw_deg: -9.0,
        pitch_deg: 14.0,
        ..sloped()
    };
    let (sl, sr, lp, ls) = c.lines();
    let geom = geometry_from_lines(sl, sr, lp, ls, W, H).expect("valid line set");
    let intr = CameraIntrinsics {
        fx: FX as f32,
        fy: FX as f32,
        cx: W as f32 * 0.5,
        cy: H as f32 * 0.5,
    };
    let flat = anchor::flatten_homography(&geom, &intr, W, H).expect("homography");

    // Round-trip: send each reported end back through the warp and it must
    // land on the lockbar corner it came from.
    for (end, corner) in [
        (flat.bar_left, geom.corners[3]),
        (flat.bar_right, geom.corners[2]),
    ] {
        let back = apply(&flat.dst_to_src, f64::from(end.0), f64::from(end.1));
        assert!(
            (back.0 - f64::from(corner.0)).abs() < 1.0
                && (back.1 - f64::from(corner.1)).abs() < 1.0,
            "guide {end:?} maps back to {back:?}, expected {corner:?}"
        );
    }
    assert!(flat.bar_left.0 < flat.bar_right.0, "left must stay left");
    assert!(
        (flat.bar_left.1 - flat.bar_right.1).abs() < 0.5,
        "the lockbar must land horizontal"
    );
}
