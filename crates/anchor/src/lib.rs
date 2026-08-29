//! Cabinet **anchor** detector — locates the pinball cabinet's reference frame
//! (lockbar + side rails) so head-tracking can auto-calibrate without any manual
//! step. Runs on **RGB** through ONNX Runtime (`ort`), the same runtime already
//! shipped for the BlazePose skeleton.
//!
//! Model: YOLO11n-pose, single class `anchor`, **6 keypoints**, 1280×1280 input,
//! one output `output0` `[1, 23, 33600]` — per-anchor
//! `[cx, cy, w, h, score, (kx, ky, kconf) × 6]`, channel-major, box/keypoints in
//! model-input pixels. There is exactly one cabinet per frame, so we keep the
//! single highest-scoring detection (no NMS).
//!
//! Keypoints (see `tools/anchor/`):
//! ```text
//!   0 player_left   1 player_right   2 screen_right
//!   3 screen_left   4 bottom_left    5 bottom_right
//! ```
//! From them [`AnchorDetection::geometry`] derives the lockbar rectangle, the two
//! sidebars, the **depth vanishing point** (sidebars extended to infinity) and
//! the **lockbar pixel width** (metric reference = 610 mm between the sidebars).
//! The thin lockbar *thickness* is deliberately never used for scale — it is
//! ill-conditioned; the sidebar-to-sidebar width is the reference.

use image::imageops::FilterType;
use image::{ImageBuffer, Rgb, Rgba};
use ort::session::Session;
use ort::value::Tensor;

const MODEL_BYTES: &[u8] =
    include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/models/anchor.onnx"));

pub const MODEL_SIDE: usize = 1280;
const NUM_ANCHORS: usize = 33_600;
/// 6 keypoints, each `(x, y, confidence)`.
pub const NUM_KPTS: usize = 6;
/// 4 box + 1 score + 6 × 3 keypoint channels.
const DET_CHANNELS: usize = 4 + 1 + NUM_KPTS * 3;

const DEFAULT_SCORE_THRESHOLD: f32 = 0.25;

// Keypoint indices (must match the training order in `tools/anchor/`).
pub const PLAYER_L: usize = 0;
pub const PLAYER_R: usize = 1;
pub const SCREEN_R: usize = 2;
pub const SCREEN_L: usize = 3;
pub const BOTTOM_L: usize = 4;
pub const BOTTOM_R: usize = 5;

pub type Point = (f32, f32);

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("ONNX Runtime error: {0}")]
    Ort(#[from] ort::Error),
}

/// Letterbox transform between original-image pixels and the model square input.
#[derive(Debug, Clone, Copy)]
pub struct Letterbox {
    pub scale: f32,
    pub pad_x: f32,
    pub pad_y: f32,
}

impl Letterbox {
    /// Model coords → original-image pixels.
    #[must_use]
    pub fn unmap_xy(self, mx: f32, my: f32) -> Point {
        (
            (mx - self.pad_x) / self.scale,
            (my - self.pad_y) / self.scale,
        )
    }
}

/// One detected cabinet: the 6 keypoints in ORIGINAL image pixels.
#[derive(Debug, Clone)]
pub struct AnchorDetection {
    pub score: f32,
    pub keypoints: [Point; NUM_KPTS],
    pub kconf: [f32; NUM_KPTS],
}

impl AnchorDetection {
    /// Derive the cabinet geometry (corners, sidebars, vanishing point, width,
    /// lateral offset) from the raw keypoints, for a `frame_w × frame_h` image.
    #[must_use]
    pub fn geometry(&self, frame_w: u32, frame_h: u32) -> AnchorGeometry {
        let k = &self.keypoints;
        let corners = [k[PLAYER_L], k[PLAYER_R], k[SCREEN_R], k[SCREEN_L]];
        // Each sidebar: (corner at the lockbar's SCREEN edge, far point at the
        // image bottom). The screen edge is the anchor of choice everywhere:
        // during play the player's hands rest on the lockbar and occlude the
        // player edge first, while the screen edge stays visible. The player
        // edge keeps direction-only roles (width VP, 90° quality gate).
        let left_sidebar = (k[SCREEN_L], k[BOTTOM_L]);
        let right_sidebar = (k[SCREEN_R], k[BOTTOM_R]);
        // Sidebars extended to infinity meet at the depth vanishing point.
        let depth_vp = intersect(
            left_sidebar.0,
            left_sidebar.1,
            right_sidebar.0,
            right_sidebar.1,
        );
        // Metric reference: the lockbar width (610 mm) measured on the SCREEN
        // edge only — the same edge `camera_pose` ranges against, so every
        // consumer measures the same physical thing. (It used to average both
        // edges, which mixed two depths ~70 mm apart.)
        let lockbar_width_px = dist(k[SCREEN_L], k[SCREEN_R]);
        // Perspective-correct centre of the lockbar face: the intersection of
        // the quad's diagonals (the arithmetic corner mean is NOT the centre
        // of a perspective rectangle). Fall back to the mean on degeneracy.
        let lockbar_center = intersect(corners[0], corners[2], corners[1], corners[3]).unwrap_or((
            0.25 * (corners[0].0 + corners[1].0 + corners[2].0 + corners[3].0),
            0.25 * (corners[0].1 + corners[1].1 + corners[2].1 + corners[3].1),
        ));
        let lateral_offset_px = lockbar_center.0 - frame_w as f32 * 0.5;
        let _ = frame_h;
        AnchorGeometry {
            corners,
            left_sidebar,
            right_sidebar,
            depth_vp,
            lockbar_width_px,
            lockbar_center,
            lateral_offset_px,
        }
    }
}

/// One annotated line segment, `[[x1, y1], [x2, y2]]` in image pixels — the
/// exchange format of the hand annotator (`tools/anchor/annotator.html`).
pub type LineSeg = [[f32; 2]; 2];

/// Build the cabinet geometry from the **4 hand-annotated lines** instead of
/// the model's keypoints — the "hand-fixed calibration" path used while the
/// anchor model is still weak. Derives the same 6 anchored points as
/// `tools/anchor/lines_to_yolo.py` (4 corner intersections + 2 bottom-of-image
/// rail points) and then reuses [`AnchorDetection::geometry`] — the *same*
/// assembly code path the detector output takes, so both sources stay
/// bit-identical in behaviour.
///
/// Intersections run in f64 (annotation coordinates are exact decimals; the
/// downstream VP maths amplifies noise, mirroring the Python harness).
/// Returns `None` when a needed line pair is (near-)parallel — a rail parallel
/// to the lockbar means the annotation is broken, not the camera.
#[must_use]
pub fn geometry_from_lines(
    sideleft: LineSeg,
    sideright: LineSeg,
    lockbar_player: LineSeg,
    lockbar_screen: LineSeg,
    img_w: u32,
    img_h: u32,
) -> Option<AnchorGeometry> {
    // f64 line-line intersection (same formula as `intersect`, higher precision).
    fn meet(a: LineSeg, b: LineSeg) -> Option<Point> {
        let [[x1, y1], [x2, y2]] = a.map(|p| p.map(f64::from));
        let [[x3, y3], [x4, y4]] = b.map(|p| p.map(f64::from));
        let den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if den.abs() < 1e-9 {
            return None;
        }
        let px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den;
        let py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den;
        Some((px as f32, py as f32))
    }
    // Point on `line` at row `y` (the annotator lines may be drawn in any
    // vertical order; a horizontal line degenerates to its first endpoint,
    // matching `lines_to_yolo.at_y`).
    fn at_y(line: LineSeg, y: f64) -> Point {
        let [[x1, y1], [x2, y2]] = line.map(|p| p.map(f64::from));
        if (y2 - y1).abs() < 1e-9 {
            return (x1 as f32, y as f32);
        }
        let t = (y - y1) / (y2 - y1);
        ((x1 + t * (x2 - x1)) as f32, y as f32)
    }

    let bottom = f64::from(img_h) - 1.0;
    let keypoints = [
        meet(sideleft, lockbar_player)?,  // PLAYER_L
        meet(sideright, lockbar_player)?, // PLAYER_R
        meet(sideright, lockbar_screen)?, // SCREEN_R
        meet(sideleft, lockbar_screen)?,  // SCREEN_L
        at_y(sideleft, bottom),           // BOTTOM_L
        at_y(sideright, bottom),          // BOTTOM_R
    ];
    // Hand lines carry no model score — a synthetic full-confidence detection
    // routed through the shared geometry assembly.
    let det = AnchorDetection {
        score: 1.0,
        keypoints,
        kconf: [1.0; NUM_KPTS],
    };
    Some(det.geometry(img_w, img_h))
}

/// Cabinet reference-frame geometry, all in original image pixels.
#[derive(Debug, Clone, Copy)]
pub struct AnchorGeometry {
    /// Lockbar face corners: `[player_left, player_right, screen_right, screen_left]`.
    pub corners: [Point; 4],
    /// Left sidebar as `(near corner, far bottom point)`.
    pub left_sidebar: (Point, Point),
    /// Right sidebar as `(near corner, far bottom point)`.
    pub right_sidebar: (Point, Point),
    /// Depth vanishing point — the two sidebars extended to infinity. `None` if
    /// they are (near-)parallel in the image (fronto-parallel camera).
    pub depth_vp: Option<Point>,
    /// Lockbar width in pixels — the 610 mm reference measured on the SCREEN
    /// edge (the edge `camera_pose` ranges against; robust to hands resting
    /// on the bar).
    pub lockbar_width_px: f32,
    /// Lockbar centre in pixels (perspective-correct: diagonal intersection).
    pub lockbar_center: Point,
    /// Lateral offset of the lockbar centre from the image centre (px, +right).
    pub lateral_offset_px: f32,
}

/// Loaded anchor model.
pub struct AnchorDetector {
    model: Session,
    score_threshold: f32,
}

impl AnchorDetector {
    /// Load and prepare the embedded model.
    ///
    /// # Errors
    /// Fails if ONNX Runtime cannot build a session from the embedded model.
    pub fn new() -> Result<Self, Error> {
        // Capped pool + no spinning, same rationale as the blazepose sessions:
        // ort's defaults are a per-session pool sized to all cores that
        // busy-spins between runs — hostile to a VPX plugin sharing the CPU
        // with the game (and this detector only runs during calibration
        // warmup anyway).
        // `.unwrap_or_else(|e| e.recover())`: ort's config idiom — a failed
        // option hands the builder back and degrades to the default.
        let model = Session::builder()?
            .with_intra_threads(2)
            .unwrap_or_else(|e| e.recover())
            .with_inter_threads(1)
            .unwrap_or_else(|e| e.recover())
            .with_intra_op_spinning(false)
            .unwrap_or_else(|e| e.recover())
            .with_inter_op_spinning(false)
            .unwrap_or_else(|e| e.recover())
            .commit_from_memory(MODEL_BYTES)?;
        Ok(Self {
            model,
            score_threshold: DEFAULT_SCORE_THRESHOLD,
        })
    }

    pub fn set_score_threshold(&mut self, t: f32) {
        self.score_threshold = t.clamp(0.0, 1.0);
    }

    /// Run inference on one camera frame, in whichever layout the driver
    /// produced it. Returns the single best cabinet, or `None` if nothing
    /// scores above the threshold (or inference fails).
    pub fn detect(
        &mut self,
        frame: &[u8],
        width: u32,
        height: u32,
        layout: PixelLayout,
    ) -> Option<AnchorDetection> {
        if width == 0 || height == 0 || frame.len() != layout.byte_len(width, height) {
            return None;
        }
        let (input, lb) = letterbox_to_chw(frame, width, height, layout);
        let val = Tensor::from_array((vec![1usize, 3, MODEL_SIDE, MODEL_SIDE], input)).ok()?;
        let outputs = match self.model.run(ort::inputs![val]) {
            Ok(o) => o,
            Err(e) => {
                tracing::warn!(?e, "anchor ONNX inference failed");
                return None;
            }
        };
        let (_, data) = outputs["output0"].try_extract_tensor::<f32>().ok()?;
        if data.len() != DET_CHANNELS * NUM_ANCHORS {
            tracing::warn!(len = data.len(), "anchor output0 has unexpected size");
            return None;
        }
        // Channel-major: channel c is `data[c*NUM_ANCHORS .. (c+1)*NUM_ANCHORS]`.
        let ch = |c: usize, i: usize| data[c * NUM_ANCHORS + i];

        // Single instance per frame → keep the highest-scoring anchor.
        let mut best_i = 0usize;
        let mut best_s = 0.0f32;
        for i in 0..NUM_ANCHORS {
            let s = ch(4, i);
            if s > best_s {
                best_s = s;
                best_i = i;
            }
        }
        if best_s < self.score_threshold {
            return None;
        }

        let mut keypoints = [(0.0f32, 0.0f32); NUM_KPTS];
        let mut kconf = [0.0f32; NUM_KPTS];
        for k in 0..NUM_KPTS {
            let mx = ch(5 + 3 * k, best_i);
            let my = ch(5 + 3 * k + 1, best_i);
            kconf[k] = ch(5 + 3 * k + 2, best_i);
            keypoints[k] = lb.unmap_xy(mx, my);
        }
        Some(AnchorDetection {
            score: best_s,
            keypoints,
            kconf,
        })
    }
}

// ---------------------------------------------------------------- camera pose
//
// Port of the VALIDATED offline harness `tools/anchor/campose.py`
// (`pose_given_f`), which was cross-checked against ground truth on the three
// hand-annotated captures: rails⟂lockbar 3D angle ≈ 91° with factory focals,
// lockbar distance within −2.2 % of the depth-measured value, and camera
// heights from two different sensors agreeing to 5 mm. The formulas below are
// a 1:1 port — fix the port, never re-derive.
//
// Stated assumptions (same as the harness):
//   * pinhole camera, square pixels (only `fx` is used), principal point at
//     the image centre;
//   * the playfield is a planar rectangle; the two rails are parallel and
//     exactly `lockbar_mm` apart (the metric reference — never the thin
//     lockbar depth);
//   * Z = f·W/w_px treats the lockbar segment as fronto-parallel (small error
//     when the camera yaw is small);
//   * `height_mm` measures to the plane through the lockbar TOP (the physical
//     playfield glass sits a few cm below it).

/// Pinhole intrinsics of the frame the anchor was detected on. Square pixels
/// are assumed by the pose math (`fx` is the focal used throughout); `fy` is
/// carried for completeness/future use. `cx`/`cy` = principal point (image
/// centre when no calibration is available).
#[derive(Debug, Clone, Copy)]
pub struct CameraIntrinsics {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
}

/// Camera pose relative to the cabinet, derived from the anchor geometry.
///
/// Axes/conventions (from the validated harness): `pitch_deg` is the camera
/// tilt vs the playfield plane (0 = grazing, 90 = looking straight down);
/// `yaw_deg` is the optical axis vs the rail direction, projected into the
/// playfield plane; `roll_deg` is the horizon angle vs the image x-axis;
/// `distance_mm` is the camera-Z depth of the lockbar screen edge;
/// `lateral_mm` is the camera position vs the cab centreline (+ = camera is
/// right of centre); `height_mm` is the distance to the plane through the
/// lockbar top.
#[derive(Debug, Clone, Copy)]
pub struct CameraPose {
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub roll_deg: f32,
    pub distance_mm: f32,
    pub lateral_mm: f32,
    pub height_mm: f32,
    /// 3D angle between the rail and lockbar directions — the built-in
    /// self-test: 90° means the focal and the detected geometry agree
    /// (perfect input); the hand-annotation noise floor is ~1.5°.
    pub rect_angle_deg: f32,
}

/// One field of a [`CameraPose`] ready to put in front of a player: a short
/// header, the formatted value, and what the number means — including which
/// way its sign points.
///
/// The wording lives here, beside the code that defines the signs, because a
/// sign convention explained somewhere else is a sign convention that drifts.
/// Every claim below is pinned by `tests/pose_conventions.rs`, which places a
/// camera at a known pose in a synthetic cabinet and checks what comes back.
pub struct PoseField {
    /// Column header. Short enough for a table.
    pub label: &'static str,
    /// The value, units included.
    pub value: String,
    /// What it means, and which way the sign points.
    pub help: &'static str,
}

impl CameraPose {
    /// The pose as labelled fields, in reading order: where the camera is,
    /// then where it looks, then the self-test.
    #[must_use]
    pub fn report(&self) -> Vec<PoseField> {
        vec![
            PoseField {
                label: "distance",
                value: format!("{:.2} m", self.distance_mm / 1000.0),
                help: "How far the lockbar is along the direction the camera \
                       points. Tilting the camera down lengthens it without \
                       moving the camera, so it is not a distance across the \
                       floor.",
            },
            PoseField {
                label: "height",
                value: format!("{:.0} cm", self.height_mm / 10.0),
                help: "How high the camera sits above the plane of the \
                       playfield.",
            },
            PoseField {
                label: "lateral",
                value: format!("{:+.1} cm", self.lateral_mm / 10.0),
                help: "Sideways offset of the camera from the middle of the \
                       cabinet, as the camera sees it. Negative is to the \
                       left: -3.0 cm means the camera sits 3 cm left of \
                       centre.",
            },
            PoseField {
                label: "pitch",
                value: format!("{:.0}\u{00b0}", self.pitch_deg),
                help: "How far the camera looks down at the playfield, \
                       measured against the playfield itself and NOT against \
                       the horizon. A table sloped 6\u{00b0} therefore reads \
                       6\u{00b0} even with a perfectly level camera: subtract the \
                       table's incline to get the camera's own tilt.",
            },
            PoseField {
                label: "yaw",
                value: format!("{:+.0}\u{00b0}", self.yaw_deg),
                help: "How far the camera is aimed off the cabinet's axis. \
                       Positive is aimed to its own left. A camera parked to \
                       one side and aimed back at the cabinet shows lateral \
                       and yaw with opposite signs — that is normal.",
            },
            PoseField {
                label: "roll",
                value: format!("{:+.0}\u{00b0}", self.roll_deg),
                help: "How far the camera is rotated about its own axis. \
                       Positive means the left end of the lockbar looks \
                       higher in the picture; 0 means it runs level.",
            },
            PoseField {
                label: "square",
                value: format!("{:.0}\u{00b0}", self.rect_angle_deg),
                help: "Self-test: rebuilt in 3D, the rails and the lockbar \
                       should meet at 90\u{00b0}. Far from 90 means the focal \
                       length or the detected lines are wrong. It is blind \
                       while 'yaw' is near zero — aimed straight down the \
                       cabinet, the check reads 90\u{00b0} whatever focal it is \
                       given. Nothing rescues it there: not the table's \
                       slope, not the camera's tilt, not an offset to one \
                       side. Only turning the camera off the axis makes it \
                       able to answer at all.",
            },
        ]
    }

    /// The same reading as one plain sentence, for a place with no room for a
    /// table — a log line, or the host's startup notification.
    #[must_use]
    pub fn describe(&self) -> String {
        let offset = if self.lateral_mm.abs() < 10.0 {
            "on the centreline".to_string()
        } else if self.lateral_mm < 0.0 {
            format!("{:.0} cm left of centre", -self.lateral_mm / 10.0)
        } else {
            format!("{:.0} cm right of centre", self.lateral_mm / 10.0)
        };
        let aim = if self.yaw_deg.abs() < 1.0 {
            "straight down the cabinet".to_string()
        } else if self.yaw_deg > 0.0 {
            format!("{:.0}\u{00b0} to its left", self.yaw_deg)
        } else {
            format!("{:.0}\u{00b0} to its right", -self.yaw_deg)
        };
        format!(
            "camera {offset}, {:.0} cm above the playfield, {:.2} m from the lockbar, \
             aimed {aim}, looking {:.0}\u{00b0} down onto the playfield",
            self.height_mm / 10.0,
            self.distance_mm / 1000.0,
            self.pitch_deg,
        )
    }
}

// f64 helpers for the pose math — the pixel inputs are f32 but the VP
// intersections amplify noise, so the intermediate algebra stays in f64
// (mirrors the Python harness, which is all-f64).
type V3 = (f64, f64, f64);

#[inline]
fn cross3(a: V3, b: V3) -> V3 {
    (
        a.1 * b.2 - a.2 * b.1,
        a.2 * b.0 - a.0 * b.2,
        a.0 * b.1 - a.1 * b.0,
    )
}

#[inline]
fn dot3(a: V3, b: V3) -> f64 {
    a.0 * b.0 + a.1 * b.1 + a.2 * b.2
}

#[inline]
fn unit3(a: V3) -> V3 {
    let n = dot3(a, a).sqrt();
    (a.0 / n, a.1 / n, a.2 / n)
}

#[inline]
fn neg3(a: V3) -> V3 {
    (-a.0, -a.1, -a.2)
}

/// Homogeneous line through two euclidean image points.
#[inline]
fn line_h(p1: Point, p2: Point) -> V3 {
    cross3(
        (f64::from(p1.0), f64::from(p1.1), 1.0),
        (f64::from(p2.0), f64::from(p2.1), 1.0),
    )
}

/// The cabinet's two in-plane directions and its normal, in camera
/// coordinates, reconstructed from the vanishing points and the focal.
///
/// Shared by [`camera_pose`] and [`flatten_homography`] so the pose and the
/// picture can never be built from different geometry. Returns
/// `(depth_vp, width_vp, rail_dir, width_dir, normal)`; `None` when the rails
/// are parallel in the image (fronto-parallel camera: the plane is
/// unobservable).
fn plane_basis(geom: &AnchorGeometry, intr: &CameraIntrinsics) -> Option<(V3, V3, V3, V3, V3)> {
    let f = f64::from(intr.fx);
    if f.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        return None;
    }
    let (cx, cy) = (f64::from(intr.cx), f64::from(intr.cy));

    // Depth VP: the two rails extended to infinity. Must be finite — parallel
    // rails mean a fronto-parallel camera and the pose is unobservable.
    let vd = cross3(
        line_h(geom.left_sidebar.0, geom.left_sidebar.1),
        line_h(geom.right_sidebar.0, geom.right_sidebar.1),
    );
    let s = vd.0.abs().max(vd.1.abs()).max(1.0);
    if vd.2.abs() < 1e-9 * s {
        return None;
    }

    // Width VP: the two lockbar edges (player edge, screen edge). May be at
    // infinity (near-parallel image lines) — handled below.
    let vw = cross3(
        line_h(geom.corners[0], geom.corners[1]),
        line_h(geom.corners[3], geom.corners[2]),
    );

    // Rail (depth) direction: toward the VP = receding toward the player.
    let r = unit3((vd.0 / vd.2 - cx, vd.1 / vd.2 - cy, f));

    // Width direction: from the width VP if finite, else parallel to the
    // image plane along the lockbar's common image direction.
    let sw = vw.0.abs().max(vw.1.abs()).max(1.0);
    let mut w3 = if vw.2.abs() < 1e-9 * sw {
        unit3((vw.0, vw.1, 0.0)) // line at infinity: pure image direction
    } else {
        unit3((vw.0 / vw.2 - cx, vw.1 / vw.2 - cy, f))
    };
    if w3.0 < 0.0 {
        w3 = neg3(w3); // +X_cab = image left → right
    }

    // Playfield normal (X_cab × D_cab, right-handed), signed to point "up"
    // out of the playfield toward the camera side (negative image-y-ish).
    let mut n = unit3(cross3(w3, r));
    if n.1 > 0.0 {
        n = neg3(n);
    }
    Some((vd, vw, r, w3, n))
}

/// Full camera pose relative to the cab from a detected anchor geometry, the
/// colour-frame intrinsics, and the real lockbar width in millimetres.
///
/// Returns `None` when the geometry is degenerate: rails (near-)parallel in
/// the image (depth VP at infinity — fronto-parallel camera), non-positive
/// focal, or a collapsed lockbar segment. The width VP *at* infinity is fine
/// and handled (fronto-parallel lockbar, the common centred-camera case).
#[must_use]
pub fn camera_pose(
    geom: &AnchorGeometry,
    intr: &CameraIntrinsics,
    lockbar_mm: f32,
) -> Option<CameraPose> {
    let f = f64::from(intr.fx);
    // NaN-safe positivity gates (NaN fails both comparisons → rejected).
    if f.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater)
        || lockbar_mm.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater)
    {
        return None;
    }
    let (cx, cy) = (f64::from(intr.cx), f64::from(intr.cy));
    let (vd, vw, r, w3, n) = plane_basis(geom, intr)?;

    // 3D angle between rails and lockbar — 90° when the focal is right.
    let rect_angle = dot3(r, w3).clamp(-1.0, 1.0).acos().to_degrees();

    let z_axis: V3 = (0.0, 0.0, 1.0);
    // Camera pitch vs the playfield plane: 0 = grazing, 90 = straight down.
    let pitch = 90.0 - dot3(z_axis, n).abs().clamp(-1.0, 1.0).acos().to_degrees();
    // Yaw: optical axis projected into the playfield plane vs the rail dir.
    let zn = dot3(z_axis, n);
    let zp = (
        z_axis.0 - n.0 * zn,
        z_axis.1 - n.1 * zn,
        z_axis.2 - n.2 * zn,
    );
    let yaw = if dot3(zp, zp).sqrt() < 1e-9 {
        0.0 // camera looking exactly along the normal — yaw undefined
    } else {
        let zp = unit3(zp);
        dot3(cross3(r, zp), n).atan2(dot3(zp, r)).to_degrees()
    };
    // Roll: angle of the horizon (line joining both VPs) vs the image x-axis.
    let h = cross3(vd, vw);
    let mut roll = (-h.0).atan2(h.1).to_degrees();
    if roll > 90.0 {
        roll -= 180.0;
    }
    if roll < -90.0 {
        roll += 180.0;
    }

    // Distance to the lockbar (screen edge, between the rails).
    let (pl, pr) = (geom.corners[3], geom.corners[2]); // screen_left, screen_right
    let w_px = f64::from(dist(pl, pr));
    if w_px < 1.0 {
        return None;
    }
    let z_lock = f * f64::from(lockbar_mm) / w_px; // fronto-parallel approximation

    // Lockbar centre ray → 3D, then decompose in the cab basis.
    let m = (f64::from(pl.0 + pr.0) / 2.0, f64::from(pl.1 + pr.1) / 2.0);
    let m3 = (z_lock * (m.0 - cx) / f, z_lock * (m.1 - cy) / f, z_lock);
    let lateral = dot3(m3, w3); // cab centreline appears at +lateral along X_cab
    let height = dot3(m3, n).abs(); // distance to the lockbar-top plane

    Some(CameraPose {
        pitch_deg: pitch as f32,
        yaw_deg: yaw as f32,
        roll_deg: roll as f32,
        distance_mm: z_lock as f32,
        lateral_mm: -lateral as f32, // camera position vs cab centreline
        height_mm: height as f32,
        rect_angle_deg: rect_angle as f32,
    })
}

/// A 3x3 matrix, row-major. Small enough that a dependency would cost more
/// than the twenty lines below.
type M3 = [f64; 9];

fn mat_mul(a: &M3, b: &M3) -> M3 {
    let mut out = [0.0; 9];
    for r in 0..3 {
        for c in 0..3 {
            out[r * 3 + c] = (0..3).map(|k| a[r * 3 + k] * b[k * 3 + c]).sum();
        }
    }
    out
}

/// Apply to a homogeneous point; `None` when the result is at infinity.
fn mat_apply(m: &M3, x: f64, y: f64) -> Option<(f64, f64)> {
    let w = m[6] * x + m[7] * y + m[8];
    if w.abs() < 1e-12 {
        return None;
    }
    Some((
        (m[0] * x + m[1] * y + m[2]) / w,
        (m[3] * x + m[4] * y + m[5]) / w,
    ))
}

fn mat_invert(m: &M3) -> Option<M3> {
    let c = [
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
    let det = m[0] * c[0] + m[1] * c[1] + m[2] * c[2];
    if det.abs() < 1e-12 {
        return None;
    }
    // `c` is already the adjugate laid out transposed.
    Some([
        c[0] / det,
        c[3] / det,
        c[6] / det,
        c[1] / det,
        c[4] / det,
        c[7] / det,
        c[2] / det,
        c[5] / det,
        c[8] / det,
    ])
}

/// Homography that shows the cabinet **face-on**: the camera image as it
/// would look from a viewpoint square to the playfield.
///
/// Built from the focal length and the two vanishing points — deliberately
/// **not** from the four detected corners. Mapping four corners onto a
/// rectangle would produce a rectangle whatever the calibration says, which
/// is worthless as a check. Going through the reconstructed plane means the
/// picture can come out wrong, and comes out wrong for the same reasons
/// [`CameraPose::rect_angle_deg`] leaves 90: a bad focal, or lines that do
/// not follow the real rails. It is that number as a picture.
///
/// Only the playfield plane is rectified. Everything standing off it — the
/// player, the backbox — smears, because it is being shown from a viewpoint
/// it was never seen from. That is expected, not a defect.
///
/// Returns the 3x3 matrix, row-major, mapping a **destination pixel to a
/// source pixel**, ready for an inverse-mapping resampler. The cabinet is
/// laid out with the lockbar across the bottom and the playfield running up,
/// filling `dst_w` x `dst_h`. `None` when the plane is unobservable (rails
/// parallel in the image) or the geometry is degenerate.
#[must_use]
pub fn flatten_homography(
    geom: &AnchorGeometry,
    intr: &CameraIntrinsics,
    dst_w: u32,
    dst_h: u32,
) -> Option<M3> {
    if dst_w < 2 || dst_h < 2 {
        return None;
    }
    let (vd, _vw, r, w3, n) = plane_basis(geom, intr)?;
    let f = f64::from(intr.fx);
    let (cx, cy) = (f64::from(intr.cx), f64::from(intr.cy));

    // Virtual camera looking along the playfield normal, from the same
    // centre. `n` points back toward the camera, so the view direction is its
    // opposite; check against the lockbar's own ray rather than trusting the
    // sign convention twice over.
    let lc = geom.lockbar_center;
    let to_lockbar = unit3((f64::from(lc.0) - cx, f64::from(lc.1) - cy, f));
    let z_v = if dot3(n, to_lockbar) > 0.0 {
        n
    } else {
        neg3(n)
    };
    let x_v = w3;
    let y_v = cross3(z_v, x_v);

    // H_rot = K * R * K^-1, with R the rotation into the virtual frame.
    let k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0];
    let k_inv = [1.0 / f, 0.0, -cx / f, 0.0, 1.0 / f, -cy / f, 0.0, 0.0, 1.0];
    let rot = [
        x_v.0, x_v.1, x_v.2, y_v.0, y_v.1, y_v.2, z_v.0, z_v.1, z_v.2,
    ];
    let h_rot = mat_mul(&k, &mat_mul(&rot, &k_inv));

    // Scale from the one length the geometry knows: the lockbar's screen
    // edge. Two thirds of the destination width leaves room for the rails to
    // sit inside the frame when the calibration is off.
    let a = mat_apply(
        &h_rot,
        f64::from(geom.corners[3].0),
        f64::from(geom.corners[3].1),
    )?;
    let b = mat_apply(
        &h_rot,
        f64::from(geom.corners[2].0),
        f64::from(geom.corners[2].1),
    )?;
    let bar_px = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
    if bar_px < 1e-6 {
        return None;
    }
    let scale = f64::from(dst_w) * 2.0 / 3.0 / bar_px;

    // Which way the playfield runs in the virtual image: the rail vanishing
    // point is at infinity there, so its mapped direction is the rail axis.
    // The cabinet must run UP the destination, whichever way it ran here.
    let vdn = (vd.0 / vd.2, vd.1 / vd.2);
    let rail_dir = {
        let d = (
            rot[0] * (vdn.0 - cx) + rot[1] * (vdn.1 - cy) + rot[2] * f,
            rot[3] * (vdn.0 - cx) + rot[4] * (vdn.1 - cy) + rot[5] * f,
        );
        let len = (d.0 * d.0 + d.1 * d.1).sqrt();
        if len < 1e-9 {
            return None;
        }
        (d.0 / len, d.1 / len)
    };
    // `r` is unused beyond the basis, but naming it keeps the tuple readable.
    let _ = r;
    let flip_y = if rail_dir.1 > 0.0 { -1.0 } else { 1.0 };

    // Lockbar centred across the frame, sitting near the bottom edge.
    let mid = ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
    let tx = f64::from(dst_w) / 2.0 - scale * mid.0;
    let ty = f64::from(dst_h) * 0.88 - scale * flip_y * mid.1;
    let fit = [scale, 0.0, tx, 0.0, scale * flip_y, ty, 0.0, 0.0, 1.0];

    mat_invert(&mat_mul(&fit, &h_rot))
}

#[inline]
fn dist(a: Point, b: Point) -> f32 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

/// Intersection of line `a1a2` with line `b1b2`. `None` if (near-)parallel.
#[must_use]
pub fn intersect(a1: Point, a2: Point, b1: Point, b2: Point) -> Option<Point> {
    let (x1, y1) = a1;
    let (x2, y2) = a2;
    let (x3, y3) = b1;
    let (x4, y4) = b2;
    let den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if den.abs() < 1e-6 {
        return None;
    }
    let px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den;
    let py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den;
    Some((px, py))
}

/// How a frame's bytes are laid out.
///
/// The model reads a 1280² letterbox, so repacking a 1080p frame into
/// tightly-packed RGB just to feed it costs more than reading the driver's
/// own layout. Callers hand over whatever their camera produced.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum PixelLayout {
    /// 3 bytes per pixel, `[R, G, B]`.
    #[default]
    Rgb888,
    /// 4 bytes per pixel, `[B, G, R, X]` — the Kinect v2's native colour frame.
    Bgrx8888,
}

impl PixelLayout {
    /// Expected buffer length for a `width`×`height` frame.
    #[must_use]
    pub const fn byte_len(self, width: u32, height: u32) -> usize {
        let bpp = match self {
            Self::Rgb888 => 3,
            Self::Bgrx8888 => 4,
        };
        (width as usize) * (height as usize) * bpp
    }
}

/// Letterbox-resize the frame to `MODEL_SIDE²`, return the CHW float32 tensor
/// (0..1, grey-114 padding, Ultralytics default) plus the transform to undo it.
fn letterbox_to_chw(
    frame: &[u8],
    width: u32,
    height: u32,
    layout: PixelLayout,
) -> (Vec<f32>, Letterbox) {
    let scale = (MODEL_SIDE as f32 / width as f32).min(MODEL_SIDE as f32 / height as f32);
    let new_w = ((width as f32) * scale).round() as u32;
    let new_h = ((height as f32) * scale).round() as u32;
    let pad_x = ((MODEL_SIDE as u32 - new_w) / 2) as f32;
    let pad_y = ((MODEL_SIDE as u32 - new_h) / 2) as f32;

    const PAD: f32 = 114.0 / 255.0;
    let plane = MODEL_SIDE * MODEL_SIDE;
    let mut chw = vec![PAD; 3 * plane];
    let (pad_x_i, pad_y_i) = (pad_x as u32, pad_y as u32);
    // The source is *borrowed*: `ImageBuffer` takes any `Deref<Target = [u8]>`
    // container, and the `to_vec()` this used to do is 6.2 MB of memcpy per
    // 1080p frame. BGRX rides through as if it were RGBA — resizing is
    // per-channel, so only the read-out order differs and X is dropped there.
    match layout {
        PixelLayout::Rgb888 => {
            let buf = ImageBuffer::<Rgb<u8>, &[u8]>::from_raw(width, height, frame)
                .expect("size validated by caller");
            let resized = image::imageops::resize(&buf, new_w, new_h, FilterType::Triangle);
            fill_chw(&mut chw, &resized, plane, pad_x_i, pad_y_i, [0, 1, 2]);
        }
        PixelLayout::Bgrx8888 => {
            let buf = ImageBuffer::<Rgba<u8>, &[u8]>::from_raw(width, height, frame)
                .expect("size validated by caller");
            let resized = image::imageops::resize(&buf, new_w, new_h, FilterType::Triangle);
            fill_chw(&mut chw, &resized, plane, pad_x_i, pad_y_i, [2, 1, 0]);
        }
    }
    (
        chw,
        Letterbox {
            scale,
            pad_x,
            pad_y,
        },
    )
}

/// Write the resized frame into the padded CHW planes, normalised to `0..1`,
/// picking channels in `order`.
fn fill_chw<P>(
    chw: &mut [f32],
    resized: &ImageBuffer<P, Vec<u8>>,
    plane: usize,
    pad_x: u32,
    pad_y: u32,
    order: [usize; 3],
) where
    P: image::Pixel<Subpixel = u8>,
{
    for y in 0..resized.height() {
        for x in 0..resized.width() {
            let p = resized.get_pixel(x, y).channels();
            let dst = ((y + pad_y) as usize) * MODEL_SIDE + (x + pad_x) as usize;
            for (c, &src) in order.iter().enumerate() {
                chw[c * plane + dst] = f32::from(p[src]) / 255.0;
            }
        }
    }
}

#[cfg(test)]
mod layout_tests {
    use super::*;

    fn pair(w: u32, h: u32) -> (Vec<u8>, Vec<u8>) {
        let mut rgb = Vec::with_capacity((w * h * 3) as usize);
        let mut bgrx = Vec::with_capacity((w * h * 4) as usize);
        for y in 0..h {
            for x in 0..w {
                let r = ((x * 7 + y * 3) % 256) as u8;
                let g = ((x * 3 + y * 11) % 256) as u8;
                let b = ((x * 13 + y * 5) % 256) as u8;
                rgb.extend_from_slice(&[r, g, b]);
                // The X byte is deliberately noise: nothing may read it.
                bgrx.extend_from_slice(&[b, g, r, ((x + y) % 256) as u8]);
            }
        }
        (rgb, bgrx)
    }

    /// Feeding the driver's own BGRX must produce exactly the tensor the
    /// repacked RGB888 produced.
    #[test]
    fn letterbox_is_layout_independent() {
        let (w, h) = (160u32, 90u32);
        let (rgb, bgrx) = pair(w, h);
        let (from_rgb, lb_rgb) = letterbox_to_chw(&rgb, w, h, PixelLayout::Rgb888);
        let (from_bgrx, lb_bgrx) = letterbox_to_chw(&bgrx, w, h, PixelLayout::Bgrx8888);
        assert!(
            from_rgb
                .iter()
                .zip(&from_bgrx)
                .all(|(a, b)| a.to_bits() == b.to_bits()),
            "letterbox differs between RGB888 and BGRX"
        );
        assert_eq!(lb_rgb.scale, lb_bgrx.scale);
        // Not vacuous: something other than the grey padding got written.
        assert!(from_rgb.iter().any(|v| (*v - 114.0 / 255.0).abs() > 1e-6));
    }

    /// A buffer whose length does not match the declared layout is refused
    /// rather than read past — the check that used to hardcode `* 3`.
    #[test]
    fn wrong_length_for_layout_is_rejected() {
        assert_eq!(PixelLayout::Rgb888.byte_len(4, 4), 48);
        assert_eq!(PixelLayout::Bgrx8888.byte_len(4, 4), 64);
    }
}
