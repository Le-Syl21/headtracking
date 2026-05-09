//! Calibration from the player's hands as a known-width fiducial.
//!
//! On a pincab the user's hands sit on the flipper buttons whenever
//! they're playing, separated by a near-constant world distance
//! ("hand span" — ~660 mm on a standard widebody, slightly inboard of
//! the actual lockbar width because hands wrap around the button posts).
//! Combined with a face detection, that gives us:
//!
//! * **Focal length** — apparent pixel distance between the two hand
//!   centroids and the known world span yield `fx` directly.
//! * **Horizon row** — the Y-line through the hands tracks where the
//!   lockbar plane projects in the image. Doubles as a sanity check
//!   for camera tilt drift.
//! * **Distance to face** — once we have `fx`, the IOD-from-bbox of
//!   the face detection translates to a metric Z. (That last step
//!   lives in the consumer; this module only deals with the
//!   hand-derived primitives.)
//!
//! The `face` workspace crate provides the [`HandDetection`] and
//! [`FaceDetection`] structs we consume here. This module is pure
//! arithmetic — no tract, no image processing, fully unit-tested
//! against numeric expectations.

use face::FaceDetection;
use face::hand::HandDetection;

/// World-side geometry of the lockbar / pincab. Loaded from the
/// per-cab calibration toml or, longer-term, from a future VPX plugin
/// API call. Defaults match Sylvain's widebody pincab.
#[derive(Debug, Clone, Copy)]
pub struct LockbarGeometry {
    /// Apparent world distance between the player's hand centroids
    /// when both rest on their flipper buttons. Defaults to 660 mm —
    /// slightly less than the 700 mm bar because hands wrap around
    /// the button posts. Override per cab.
    pub hand_span_mm: f32,
    /// Lockbar floor height in mm — used downstream to derive Y in
    /// world coords. Not consumed directly by this module.
    pub lockbar_floor_height_mm: f32,
    /// Anatomical interocular distance, used by face-bbox-based
    /// distance estimators. Adult mean ≈ 63 mm.
    pub ipd_mm: f32,
}

impl Default for LockbarGeometry {
    fn default() -> Self {
        Self {
            hand_span_mm: 660.0,
            lockbar_floor_height_mm: 850.0,
            ipd_mm: 63.0,
        }
    }
}

/// One frame's worth of hand observation.
#[derive(Debug, Clone, Copy)]
pub struct HandPair {
    /// Image-left hand (typically the player's left hand if the camera
    /// sits centred on the backbox). Use [`face::hand::sort_lr`] to
    /// produce these from a raw `Vec<HandDetection>`.
    pub left: HandDetection,
    /// Image-right hand.
    pub right: HandDetection,
}

impl HandPair {
    /// Pixel distance between the two hand centroids. Always positive.
    pub fn span_px(&self) -> f32 {
        let dx = self.right.center_x - self.left.center_x;
        let dy = self.right.center_y - self.left.center_y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Average image Y of the two hand centroids — our horizon row
    /// proxy.
    pub fn horizon_y_px(&self) -> f32 {
        0.5 * (self.left.center_y + self.right.center_y)
    }

    /// Image-tilt of the hand baseline in radians. Useful as a sanity
    /// signal: if the user is playing in a stance where this is large,
    /// the horizon estimate is no longer reliable.
    pub fn tilt_radians(&self) -> f32 {
        let dx = self.right.center_x - self.left.center_x;
        let dy = self.right.center_y - self.left.center_y;
        dy.atan2(dx)
    }
}

/// Compute the camera focal length in pixels from the known world hand
/// span and the apparent pixel span at a known camera→hands distance.
///
/// Uses the pinhole approximation `fx = (span_px * Z) / span_mm`.
/// Works for both X and Y if the camera has square pixels; we assume
/// `fx ≈ fy` for our cheap webcams.
///
/// # Inputs
/// * `span_mm` — world distance between hand centroids (lockbar geometry).
/// * `span_px` — observed pixel distance between hand centroids.
/// * `distance_mm` — camera→hands distance, e.g. measured on a Kinect or
///   estimated by an iterative loop bootstrapped from the user's height.
///
/// # Returns
/// Focal length in pixels, or `None` if any input is non-positive.
pub fn focal_from_hand_span(span_mm: f32, span_px: f32, distance_mm: f32) -> Option<f32> {
    if span_mm <= 0.0 || span_px <= 0.0 || distance_mm <= 0.0 {
        return None;
    }
    Some((span_px * distance_mm) / span_mm)
}

/// Inverse of [`focal_from_hand_span`] — given a known focal length,
/// recover the camera→hands distance from the observed pixel span.
pub fn distance_from_hand_span(span_mm: f32, span_px: f32, focal_px: f32) -> Option<f32> {
    if span_mm <= 0.0 || span_px <= 0.0 || focal_px <= 0.0 {
        return None;
    }
    Some((span_mm * focal_px) / span_px)
}

/// Estimate camera→face Z from the face bbox width using the same
/// pinhole math, treating the bbox width as a proxy for IOD with a
/// fixed multiplier (`face_width_mm ≈ ipd_mm * 1.6` is a reasonable
/// adult average — interpupillary distance is ~63 mm and a face is
/// ~100 mm wide eye-to-eye including temples).
pub fn distance_to_face(face: &FaceDetection, focal_px: f32, ipd_mm: f32) -> Option<f32> {
    let face_width_mm = ipd_mm * 1.6;
    if face.width <= 0.0 || focal_px <= 0.0 {
        return None;
    }
    Some((face_width_mm * focal_px) / face.width)
}

/// All the per-frame quantities the calibration produces. Non-`Option`
/// fields are direct observations; `Option` fields require a known
/// focal length (provided by the caller after bootstrap, or by the
/// Kinect's native depth).
#[derive(Debug, Clone, Copy)]
pub struct HandFiducialFrame {
    /// Pixel span between hands in this frame.
    pub hand_span_px: f32,
    /// Y-row that the hands trace in the image (horizon proxy).
    pub horizon_y_px: f32,
    /// Tilt of the hand baseline (rad). Caller should reject the frame
    /// for horizon-update purposes when `|tilt|` exceeds a threshold
    /// (~10° suggests non-standard stance).
    pub hand_tilt_radians: f32,
    /// Focal length recovered this frame, when a distance was provided.
    pub focal_px: Option<f32>,
    /// Camera→face distance recovered this frame, when a focal was
    /// provided.
    pub face_distance_mm: Option<f32>,
}

/// One-shot computation: given a hand pair, an optional face
/// detection, the cabinet geometry, and either a known distance OR a
/// known focal length, produce all derived quantities.
///
/// Pass `Some(distance_mm)` and `None` for `focal_px` when bootstrapping
/// from a known camera→hands distance (Kinect depth at the lockbar
/// plane, or a one-time printed-disc pass). Pass `None` and
/// `Some(focal_px)` afterwards once focal is known: distance is
/// recovered from the hand span instead.
pub fn observe(
    hands: &HandPair,
    face: Option<&FaceDetection>,
    geom: &LockbarGeometry,
    distance_hint_mm: Option<f32>,
    focal_hint_px: Option<f32>,
) -> HandFiducialFrame {
    let hand_span_px = hands.span_px();
    let horizon_y_px = hands.horizon_y_px();
    let hand_tilt_radians = hands.tilt_radians();

    let focal_px = distance_hint_mm
        .and_then(|d| focal_from_hand_span(geom.hand_span_mm, hand_span_px, d))
        .or(focal_hint_px);

    let face_distance_mm = focal_px.and_then(|f| {
        face.and_then(|fd| distance_to_face(fd, f, geom.ipd_mm))
            // Fallback: when no face but we still have a focal, infer
            // the camera→hands distance via the hand span (less useful
            // than face distance but a sanity datum).
            .or_else(|| distance_from_hand_span(geom.hand_span_mm, hand_span_px, f))
    });

    HandFiducialFrame {
        hand_span_px,
        horizon_y_px,
        hand_tilt_radians,
        focal_px,
        face_distance_mm,
    }
}

// ============================================================ Tests

#[cfg(test)]
mod tests {
    use super::*;

    fn h(x: f32, y: f32) -> HandDetection {
        HandDetection {
            center_x: x,
            center_y: y,
            confidence: 0.95,
            x: x - 30.0,
            y: y - 30.0,
            width: 60.0,
            height: 60.0,
        }
    }

    #[test]
    fn hand_span_is_positive_distance() {
        let pair = HandPair {
            left: h(100.0, 500.0),
            right: h(700.0, 500.0),
        };
        assert!((pair.span_px() - 600.0).abs() < 1e-3);
    }

    #[test]
    fn hand_span_handles_y_offset() {
        let pair = HandPair {
            left: h(100.0, 500.0),
            right: h(400.0, 900.0),
        };
        // 3-4-5 triangle scaled ×100 → 500.
        assert!((pair.span_px() - 500.0).abs() < 1e-3);
    }

    #[test]
    fn horizon_is_mean_y() {
        let pair = HandPair {
            left: h(100.0, 480.0),
            right: h(800.0, 520.0),
        };
        assert!((pair.horizon_y_px() - 500.0).abs() < 1e-3);
    }

    #[test]
    fn focal_recovers_from_known_distance() {
        // Given hands separated by 660 mm in world, projected to 600 px
        // at camera→hands distance 1500 mm, focal should be:
        //   fx = 600 * 1500 / 660 = 1363.6363...
        let f = focal_from_hand_span(660.0, 600.0, 1500.0).unwrap();
        assert!((f - (600.0 * 1500.0 / 660.0)).abs() < 1e-3);
    }

    #[test]
    fn focal_then_distance_round_trip() {
        let geom = LockbarGeometry::default();
        let f = focal_from_hand_span(geom.hand_span_mm, 600.0, 1500.0).unwrap();
        let d = distance_from_hand_span(geom.hand_span_mm, 600.0, f).unwrap();
        assert!((d - 1500.0).abs() < 1e-3);
    }

    #[test]
    fn rejects_zero_or_negative_inputs() {
        assert!(focal_from_hand_span(0.0, 600.0, 1500.0).is_none());
        assert!(focal_from_hand_span(660.0, 0.0, 1500.0).is_none());
        assert!(focal_from_hand_span(660.0, 600.0, -1.0).is_none());
        assert!(distance_from_hand_span(660.0, 600.0, 0.0).is_none());
    }

    #[test]
    fn distance_to_face_uses_ipd_proxy() {
        let face = FaceDetection {
            x: 400.0,
            y: 300.0,
            width: 120.0,
            height: 150.0,
            confidence: 0.9,
            ..Default::default()
        };
        // face_width_mm = 63 * 1.6 = 100.8; at fx=1500 px, distance =
        //   100.8 * 1500 / 120 = 1260 mm.
        let d = distance_to_face(&face, 1500.0, 63.0).unwrap();
        assert!((d - 1260.0).abs() < 1e-2);
    }

    #[test]
    fn observe_bootstraps_focal_from_distance_hint() {
        let geom = LockbarGeometry::default();
        let hands = HandPair {
            left: h(200.0, 600.0),
            right: h(800.0, 600.0),
        };
        let frame = observe(&hands, None, &geom, Some(1500.0), None);
        assert!(frame.focal_px.is_some());
        assert!((frame.hand_span_px - 600.0).abs() < 1e-3);
        assert!((frame.horizon_y_px - 600.0).abs() < 1e-3);
        assert!(frame.hand_tilt_radians.abs() < 1e-3);
    }

    #[test]
    fn observe_recovers_face_distance_when_focal_known() {
        let geom = LockbarGeometry::default();
        let hands = HandPair {
            left: h(200.0, 600.0),
            right: h(800.0, 600.0),
        };
        let face = FaceDetection {
            x: 400.0,
            y: 200.0,
            width: 100.0,
            height: 130.0,
            confidence: 0.92,
            ..Default::default()
        };
        let frame = observe(&hands, Some(&face), &geom, None, Some(1500.0));
        assert!(frame.focal_px == Some(1500.0));
        // Should produce a face distance via the IPD proxy.
        let d = frame.face_distance_mm.unwrap();
        assert!((d - (geom.ipd_mm * 1.6 * 1500.0 / 100.0)).abs() < 1e-2);
    }

    #[test]
    fn tilt_detects_non_horizontal_baseline() {
        let pair = HandPair {
            left: h(100.0, 500.0),
            right: h(500.0, 500.0),
        };
        assert!(pair.tilt_radians().abs() < 1e-3);

        let tilted = HandPair {
            left: h(100.0, 500.0),
            right: h(500.0, 600.0),
        };
        // Slope 100/400 = 0.25 → atan ≈ 14°, well above the 10°
        // skip-update threshold the consumer uses.
        let tilt_deg = tilted.tilt_radians().to_degrees();
        assert!((tilt_deg - 14.036).abs() < 0.1);
    }
}
