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

/// Camera placement in world terms.
#[derive(Clone, Copy)]
struct Cam {
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

    /// The four annotated lines, as the annotator would have drawn them.
    fn lines(self) -> (LineSeg, LineSeg, LineSeg, LineSeg) {
        let seg = |a: V3, b: V3| -> LineSeg {
            let pa = self.project(a).expect("point a in front of the camera");
            let pb = self.project(b).expect("point b in front of the camera");
            [[pa.0, pa.1], [pb.0, pb.1]]
        };
        (
            seg((-HALF, 0.0, 0.0), (-HALF, 0.0, RAIL_LEN)),
            seg((HALF, 0.0, 0.0), (HALF, 0.0, RAIL_LEN)),
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

fn base() -> Cam {
    Cam {
        pos: (0.0, 760.0, -1270.0),
        yaw_deg: 0.0,
        pitch_deg: 0.0,
        roll_deg: 0.0,
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

/// The counter-case, stated as a test so it cannot be forgotten when writing
/// the help text: head-on, the check is blind.
#[test]
fn rect_angle_is_blind_to_the_focal_when_the_camera_is_head_on() {
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
