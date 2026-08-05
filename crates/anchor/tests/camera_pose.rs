//! Golden tests for the camera-pose port — driven through the public
//! [`anchor::geometry_from_lines`] path, i.e. the exact pipeline the
//! hand-fixed calibration (`anchor_fixed.json`) uses in the demo.
//!
//! The expected values come from the VALIDATED Python harness
//! `tools/anchor/campose.py` run on the three hand-annotated captures
//! (line endpoints below are copied from the maintainer's
//! `anchor-lines.json`, the annotator's output). That harness was
//! cross-checked against ground truth: lockbar distance within −2.2 % of the
//! 16-bit depth measurement, and camera heights from two different sensors
//! agreeing to 5 mm. If a number here diverges, the PORT is wrong — fix the
//! port, never the expectation.

use anchor::{CameraIntrinsics, LineSeg, camera_pose, geometry_from_lines};

fn centred(fx: f32, w: u32, h: u32) -> CameraIntrinsics {
    CameraIntrinsics {
        fx,
        fy: fx,
        cx: w as f32 * 0.5,
        cy: h as f32 * 0.5,
    }
}

struct Expected {
    pitch: f32,
    yaw: f32,
    roll: f32,
    rect_angle: f32,
    distance: f32,
    lateral: f32,
    height: f32,
}

fn check(pose: anchor::CameraPose, e: &Expected) {
    const ANG_TOL: f32 = 0.3; // degrees
    let pct = |a: f32, b: f32| (a - b).abs() / b.abs() * 100.0;
    assert!(
        (pose.pitch_deg - e.pitch).abs() < ANG_TOL,
        "pitch {} vs {}",
        pose.pitch_deg,
        e.pitch
    );
    assert!(
        (pose.yaw_deg - e.yaw).abs() < ANG_TOL,
        "yaw {} vs {}",
        pose.yaw_deg,
        e.yaw
    );
    assert!(
        (pose.roll_deg - e.roll).abs() < ANG_TOL,
        "roll {} vs {}",
        pose.roll_deg,
        e.roll
    );
    assert!(
        (pose.rect_angle_deg - e.rect_angle).abs() < ANG_TOL,
        "rect angle {} vs {}",
        pose.rect_angle_deg,
        e.rect_angle
    );
    assert!(
        pct(pose.distance_mm, e.distance) < 1.0,
        "distance {} vs {}",
        pose.distance_mm,
        e.distance
    );
    assert!(
        pct(pose.lateral_mm, e.lateral) < 1.0,
        "lateral {} vs {}",
        pose.lateral_mm,
        e.lateral
    );
    assert!(
        pct(pose.height_mm, e.height) < 1.0,
        "height {} vs {}",
        pose.height_mm,
        e.height
    );
}

const LOCKBAR_MM: f32 = 610.0;

#[test]
fn golden_kinect_v1() {
    // ht_kinect-v1_20260802-153659_9f7cb2_raw.png (640x480), colour fx = 525.
    let sideleft: LineSeg = [[455.92, 0.0], [74.05, 480.0]];
    let sideright: LineSeg = [[358.39, 0.0], [346.04, 480.0]];
    let lockbar_player: LineSeg = [[0.0, 428.80], [640.0, 440.92]];
    let lockbar_screen: LineSeg = [[0.0, 445.30], [640.0, 459.51]];
    let geom = geometry_from_lines(
        sideleft,
        sideright,
        lockbar_player,
        lockbar_screen,
        640,
        480,
    )
    .expect("valid line set");
    let pose = camera_pose(&geom, &centred(525.0, 640, 480), LOCKBAR_MM).unwrap();
    check(
        pose,
        &Expected {
            pitch: 12.0309,
            yaw: 4.2008,
            roll: -2.1860,
            rect_angle: 91.5052,
            distance: 1297.06,
            lateral: 372.79,
            height: 769.13,
        },
    );
}

#[test]
fn golden_kinect_v2() {
    // ht_kinect-v2_20260802-153611_a94d54_raw.png (1920x1080), colour fx = 1081.
    // Width VP is at/near infinity here (centred camera) — exercises the
    // line-at-infinity branch.
    let geom = geometry_from_lines(
        [[1052.54, 0.0], [771.60, 1080.0]],
        [[759.21, 0.0], [1334.09, 1080.0]],
        [[0.0, 1003.93], [1920.0, 982.20]],
        [[0.0, 1037.29], [1920.0, 1016.98]],
        1920,
        1080,
    )
    .expect("valid line set");
    let pose = camera_pose(&geom, &centred(1081.0, 1920, 1080), LOCKBAR_MM).unwrap();
    check(
        pose,
        &Expected {
            pitch: 8.9307,
            yaw: 0.0245,
            roll: -1.4230,
            rect_angle: 91.3360,
            distance: 1270.12,
            lateral: -57.30,
            height: 763.82,
        },
    );
}

#[test]
fn golden_webcam() {
    // ht_webcam-1_20260802-153732_bcd8aa_raw.png (1280x720), nominal fx =
    // 0.9 * 1280 = 1152 (no factory intrinsics for webcams).
    let geom = geometry_from_lines(
        [[512.98, 0.0], [503.51, 720.0]],
        [[480.18, 0.0], [880.46, 720.0]],
        [[0.0, 660.99], [1280.0, 628.11]],
        [[0.0, 679.08], [1280.0, 643.05]],
        1280,
        720,
    )
    .expect("valid line set");
    let pose = camera_pose(&geom, &centred(1152.0, 1280, 720), LOCKBAR_MM).unwrap();
    check(
        pose,
        &Expected {
            pitch: 14.3205,
            yaw: -6.9931,
            roll: 3.4688,
            rect_angle: 86.9383,
            distance: 2064.42,
            lateral: -418.77,
            height: 1027.14,
        },
    );
}

#[test]
fn degenerate_parallel_rails_yields_none() {
    // Perfectly fronto-parallel camera: rails parallel in the image → the
    // depth VP is at infinity and the pose is unobservable. The geometry
    // itself still builds (the corners exist); it's the POSE that must bail.
    let geom = geometry_from_lines(
        [[100.0, 0.0], [100.0, 480.0]],
        [[540.0, 0.0], [540.0, 480.0]],
        [[0.0, 400.0], [640.0, 400.0]],
        [[0.0, 420.0], [640.0, 420.0]],
        640,
        480,
    )
    .expect("corners intersect fine");
    assert!(camera_pose(&geom, &centred(525.0, 640, 480), LOCKBAR_MM).is_none());
}

#[test]
fn broken_annotation_yields_none() {
    // A rail drawn parallel to the lockbar: the corner intersection does not
    // exist — the annotation is broken and `geometry_from_lines` must say so.
    assert!(
        geometry_from_lines(
            [[0.0, 100.0], [640.0, 100.0]], // "rail" parallel to the lockbar
            [[540.0, 0.0], [540.0, 480.0]],
            [[0.0, 400.0], [640.0, 400.0]],
            [[0.0, 420.0], [640.0, 420.0]],
            640,
            480,
        )
        .is_none()
    );
}
