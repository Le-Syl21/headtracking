//! MediaPipe **BlazePose** in Rust via ONNX Runtime (`ort`).
//!
//! Two-stage pipeline (same as MediaPipe): a lightweight **pose detector**
//! (224×224) locates the person and yields a rotated ROI, then the **landmark**
//! model (256×256) predicts 33 body keypoints inside that ROI. Unlike the
//! silhouette+skeleton path, this needs **no segmentation** — it runs straight
//! on the RGB frame and finds shoulders/elbows/wrists even with arms down.
//!
//! [`BlazePose::detect`] runs the full pipeline and returns the 33 body
//! landmarks in frame pixels. Validated against the MediaPipe Python reference
//! on real captures.

use ort::session::Session;
use ort::value::Tensor;

pub mod smoothing;

/// Embedded models (see `crates/blazepose/models/`). ~12–15 MB each.
const DETECTOR_ONNX: &[u8] = include_bytes!("../models/pose_detection.onnx");
const LANDMARK_ONNX: &[u8] = include_bytes!("../models/pose_landmark_full.onnx");

/// Detector input side (square).
pub const DETECTOR_SIDE: u32 = 224;
/// Landmark input side (square).
pub const LANDMARK_SIDE: u32 = 256;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("onnxruntime: {0}")]
    Ort(#[from] ort::Error),
}

/// One raw detector output tensor, for inspection while the decode is built.
#[derive(Debug, Clone)]
pub struct RawOutput {
    pub name: String,
    pub shape: Vec<i64>,
    pub min: f32,
    pub max: f32,
    /// Count of raw values > 0 (i.e. sigmoid > 0.5) — a proxy for "how many
    /// anchors fire" on the score tensor.
    pub positive: usize,
}

/// A fixed-size SSD anchor centre (normalised 0..1 of the 224 square).
#[derive(Debug, Clone, Copy)]
struct Anchor {
    xc: f32,
    yc: f32,
}

/// A decoded pose detection, coordinates in the **original frame** pixel space
/// (letterbox undone). `keypoints[0]` is the mid-hip, `keypoints[1]` the
/// full-body alignment point — used later to build the rotated landmark ROI.
#[derive(Debug, Clone)]
pub struct Detection {
    pub score: f32,
    /// Bounding box centre + size, in frame pixels.
    pub cx: f32,
    pub cy: f32,
    pub w: f32,
    pub h: f32,
    /// 4 detector keypoints, in frame pixels.
    pub keypoints: [[f32; 2]; 4],
}

/// One decoded body landmark; `x`/`y` in frame pixels, `z` relative depth.
#[derive(Debug, Clone, Copy, Default)]
pub struct Landmark {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub visibility: f32,
    pub presence: f32,
}

/// The 33 BlazePose body landmarks (frame pixels) + overall pose presence.
#[derive(Debug, Clone)]
pub struct Pose {
    pub landmarks: [Landmark; 33],
    pub presence: f32,
}

/// Body-landmark indices (subset we use downstream).
pub mod idx {
    pub const NOSE: usize = 0;
    pub const LEFT_SHOULDER: usize = 11;
    pub const RIGHT_SHOULDER: usize = 12;
    pub const LEFT_ELBOW: usize = 13;
    pub const RIGHT_ELBOW: usize = 14;
    pub const LEFT_WRIST: usize = 15;
    pub const RIGHT_WRIST: usize = 16;
}

/// Bilinear sample of an RGB888 frame; out-of-bounds → black.
/// How a frame's bytes are laid out.
///
/// The model reads a 224² or 256² patch, so repacking a 1080p frame into
/// tightly-packed RGB just to feed it costs far more than reading the
/// driver's own layout with a stride. Callers hand over whatever their
/// camera produced.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum PixelLayout {
    /// 3 bytes per pixel, `[R, G, B]`.
    #[default]
    Rgb888,
    /// 4 bytes per pixel, `[B, G, R, X]` — the Kinect v2's native colour
    /// frame, and what most Windows capture APIs hand out.
    Bgrx8888,
}

impl PixelLayout {
    /// Bytes per pixel, and where R, G and B sit inside one.
    const fn stride(self) -> (usize, [usize; 3]) {
        match self {
            Self::Rgb888 => (3, [0, 1, 2]),
            Self::Bgrx8888 => (4, [2, 1, 0]),
        }
    }

    /// Expected buffer length for a `w`×`h` frame.
    #[must_use]
    pub const fn byte_len(self, w: u32, h: u32) -> usize {
        (w as usize) * (h as usize) * self.stride().0
    }
}

fn sample_bilinear(frame: &[u8], w: u32, h: u32, x: f32, y: f32, layout: PixelLayout) -> [u8; 3] {
    if x < 0.0 || y < 0.0 || x > (w - 1) as f32 || y > (h - 1) as f32 {
        return [0, 0, 0];
    }
    let x0 = x.floor() as u32;
    let y0 = y.floor() as u32;
    let x1 = (x0 + 1).min(w - 1);
    let y1 = (y0 + 1).min(h - 1);
    let fx = x - x0 as f32;
    let fy = y - y0 as f32;
    let (bpp, ch) = layout.stride();
    let px = |xx: u32, yy: u32, c: usize| {
        f32::from(frame[(yy as usize * w as usize + xx as usize) * bpp + ch[c]])
    };
    let mut out = [0u8; 3];
    for (c, o) in out.iter_mut().enumerate() {
        let top = px(x0, y0, c) * (1.0 - fx) + px(x1, y0, c) * fx;
        let bot = px(x0, y1, c) * (1.0 - fx) + px(x1, y1, c) * fx;
        *o = (top * (1.0 - fy) + bot * fy).round().clamp(0.0, 255.0) as u8;
    }
    out
}

/// MediaPipe `pose_detection` SSD anchors (224 input, 5 layers, strides
/// 8/16/32/32/32, fixed size, aspect 1.0 + interpolated). Yields 2254 anchors.
fn generate_anchors() -> Vec<Anchor> {
    const NUM_LAYERS: usize = 5;
    const STRIDES: [usize; 5] = [8, 16, 32, 32, 32];
    const INPUT: f32 = DETECTOR_SIDE as f32;
    const OFFSET: f32 = 0.5;
    let mut anchors = Vec::new();
    let mut layer = 0;
    while layer < NUM_LAYERS {
        // Merge consecutive same-stride layers; each contributes one aspect
        // (1.0) + one interpolated anchor = 2 anchors per cell.
        let mut per_cell = 0;
        let mut last = layer;
        while last < NUM_LAYERS && STRIDES[last] == STRIDES[layer] {
            per_cell += 2;
            last += 1;
        }
        let fm = (INPUT / STRIDES[layer] as f32).ceil() as usize;
        for y in 0..fm {
            for x in 0..fm {
                let xc = (x as f32 + OFFSET) / fm as f32;
                let yc = (y as f32 + OFFSET) / fm as f32;
                for _ in 0..per_cell {
                    anchors.push(Anchor { xc, yc });
                }
            }
        }
        layer = last;
    }
    debug_assert_eq!(anchors.len(), 2254, "unexpected anchor count");
    anchors
}

fn sigmoid(x: f32) -> f32 {
    1.0 / (1.0 + (-x.clamp(-100.0, 100.0)).exp())
}

fn iou(a: &Detection, b: &Detection) -> f32 {
    let (ax0, ay0, ax1, ay1) = (
        a.cx - a.w * 0.5,
        a.cy - a.h * 0.5,
        a.cx + a.w * 0.5,
        a.cy + a.h * 0.5,
    );
    let (bx0, by0, bx1, by1) = (
        b.cx - b.w * 0.5,
        b.cy - b.h * 0.5,
        b.cx + b.w * 0.5,
        b.cy + b.h * 0.5,
    );
    let ix = (ax1.min(bx1) - ax0.max(bx0)).max(0.0);
    let iy = (ay1.min(by1) - ay0.max(by0)).max(0.0);
    let inter = ix * iy;
    let uni = a.w * a.h + b.w * b.h - inter;
    if uni <= 0.0 {
        0.0
    } else {
        inter / uni
    }
}

/// Below this landmark presence, tracking is considered lost and we re-run the
/// detector (MediaPipe's detect-once-then-track).
const TRACK_MIN_PRESENCE: f32 = 0.5;

/// A decoded pose plus its two auxiliary ROI points (landmarks 33 & 34, frame
/// px) that seed the next frame's tracking ROI.
type TrackedPose = (Pose, [f32; 2], [f32; 2]);

/// ROI (centre, square size, rotation) from two alignment points — the exact
/// MediaPipe pose `AlignmentPointsRects` math: centre = `c`, box = 2·|c→a|·1.25,
/// rotate so `c→a` points "up" (90°). Shared by the detector path and tracking.
/// The rotated region of interest the landmark model runs in. The three
/// values are computed together and always travel together.
#[derive(Clone, Copy)]
struct Roi {
    center: [f32; 2],
    box_size: f32,
    angle: f32,
}

fn roi_from_points(c: [f32; 2], a: [f32; 2]) -> Roi {
    let (dx, dy) = (a[0] - c[0], a[1] - c[1]);
    let dist = (dx * dx + dy * dy).sqrt();
    let box_size = 2.0 * dist * 1.25;
    let angle = std::f32::consts::FRAC_PI_2 - (-dy).atan2(dx);
    Roi {
        center: c,
        box_size,
        angle,
    }
}

// NOTE on smoothing: an earlier landmark-smoothing port was removed because
// two cascaded One-Euro stages (2D landmarks here + the consumer's mm-space
// filter) doubled the perceived lag and split the smoothing across two
// places. The current port avoids that trap by using MediaPipe's stock
// `pose_landmark_filtering.pbtxt` parameters: beta 80 on the visible
// landmarks means the filter follows any real motion within a frame or two
// and only attenuates near-static sub-pixel noise — it is source
// *de-noising*, while response shaping stays entirely in the consumer's
// tunable mm-space filter. The auxiliary ROI points get their own stiffer
// bank (beta 10), which stabilises the landmark model's INPUT crop —
// removing jitter before it is even produced, with zero output lag.

pub struct BlazePose {
    detector: Session,
    landmark: Session,
    anchors: Vec<Anchor>,
    /// The two auxiliary ROI points (landmarks 33 & 34) from the last successful
    /// frame — the seed for the next frame's ROI so we track without re-detecting.
    last_aux: Option<([f32; 2], [f32; 2])>,
    /// MediaPipe-style smoothing banks (see `smoothing`): visible landmarks…
    lm_smooth: smoothing::PointBank<33>,
    /// …and the auxiliary ROI points, which stabilise the tracking crop.
    aux_smooth: smoothing::PointBank<2>,
    /// Monotonic time base for the One-Euro timestamps.
    clock: std::time::Instant,
}

impl BlazePose {
    pub fn new() -> Result<Self, Error> {
        // Cap the thread pools and disable spinning: ort's defaults build one
        // pool per session sized to every physical core AND busy-spin workers
        // between runs — three sessions in this process would burn cores doing
        // nothing. The end goal is a VPX plugin sharing the machine with the
        // game, so the model budget is "a couple of quiet threads", not "all
        // cores, hot". ~7 ms inference grows a little; still far inside a
        // 33 ms camera frame.
        // `.unwrap_or_else(|e| e.recover())` is ort's own idiom for config
        // options: on failure the error hands the builder back, so an
        // unsupported option degrades to the default instead of aborting —
        // acceptable here since these knobs only affect CPU footprint.
        let detector = Session::builder()?
            .with_intra_threads(2)
            .unwrap_or_else(|e| e.recover())
            .with_inter_threads(1)
            .unwrap_or_else(|e| e.recover())
            .with_intra_op_spinning(false)
            .unwrap_or_else(|e| e.recover())
            .with_inter_op_spinning(false)
            .unwrap_or_else(|e| e.recover())
            .commit_from_memory(DETECTOR_ONNX)?;
        let landmark = Session::builder()?
            .with_intra_threads(2)
            .unwrap_or_else(|e| e.recover())
            .with_inter_threads(1)
            .unwrap_or_else(|e| e.recover())
            .with_intra_op_spinning(false)
            .unwrap_or_else(|e| e.recover())
            .with_inter_op_spinning(false)
            .unwrap_or_else(|e| e.recover())
            .commit_from_memory(LANDMARK_ONNX)?;
        Ok(Self {
            detector,
            landmark,
            anchors: generate_anchors(),
            last_aux: None,
            lm_smooth: smoothing::PointBank::new(
                smoothing::LANDMARK_MIN_CUTOFF,
                smoothing::LANDMARK_BETA,
            ),
            aux_smooth: smoothing::PointBank::new(smoothing::AUX_MIN_CUTOFF, smoothing::AUX_BETA),
            clock: std::time::Instant::now(),
        })
    }

    /// Apply the MediaPipe smoothing banks to a fresh landmark result: the 33
    /// visible landmarks for the output, the 2 auxiliary points for the next
    /// frame's tracking ROI. Speeds are normalised by the subject's on-screen
    /// size, like MediaPipe's `LandmarksSmoothingCalculator`.
    fn smooth_tracked(
        &mut self,
        mut pose: Pose,
        aux0: [f32; 2],
        aux1: [f32; 2],
    ) -> (Pose, [f32; 2], [f32; 2]) {
        let t = self.clock.elapsed().as_secs_f64();
        let scale = 1.0 / smoothing::object_scale(pose.landmarks.iter().map(|l| [l.x, l.y]));
        for (i, lm) in pose.landmarks.iter_mut().enumerate() {
            let [x, y, z] = self.lm_smooth.apply(i, t, scale, [lm.x, lm.y, lm.z]);
            lm.x = x;
            lm.y = y;
            lm.z = z;
        }
        let a0 = self.aux_smooth.apply(0, t, scale, [aux0[0], aux0[1], 0.0]);
        let a1 = self.aux_smooth.apply(1, t, scale, [aux1[0], aux1[1], 0.0]);
        (pose, [a0[0], a0[1]], [a1[0], a1[1]])
    }

    /// Forget smoothing history — the subject was (re)acquired by the
    /// detector, so the filters must re-initialise instead of dragging the
    /// output from wherever the previous subject was.
    fn reset_smoothing(&mut self) {
        self.lm_smooth.reset();
        self.aux_smooth.reset();
    }

    /// Run the detector and return the best person detection (score-thresholded,
    /// NMS-suppressed), with coordinates in the original frame's pixel space.
    pub fn detect_person(
        &mut self,
        frame: &[u8],
        w: u32,
        h: u32,
        layout: PixelLayout,
        score_threshold: f32,
    ) -> Result<Option<Detection>, Error> {
        let side = DETECTOR_SIDE as f32;
        let input = letterbox_nhwc(frame, w, h, DETECTOR_SIDE, layout);
        let val = Tensor::from_array((
            vec![1usize, DETECTOR_SIDE as usize, DETECTOR_SIDE as usize, 3],
            input,
        ))?;
        let outputs = self.detector.run(ort::inputs![val])?;
        let (_, boxes) = outputs["Identity"].try_extract_tensor::<f32>()?;
        let (_, scores) = outputs["Identity_1"].try_extract_tensor::<f32>()?;

        // Letterbox: frame was padded to a `long`×`long` square, then resized.
        let long = w.max(h) as f32;
        let ox = (long - w as f32) * 0.5;
        let oy = (long - h as f32) * 0.5;
        // Normalised (square) → original frame pixels.
        let to_frame = |nx: f32, ny: f32| [nx * long - ox, ny * long - oy];

        let mut dets: Vec<Detection> = Vec::new();
        for (i, a) in self.anchors.iter().enumerate() {
            let s = sigmoid(scores[i]);
            if s < score_threshold {
                continue;
            }
            let b = &boxes[i * 12..i * 12 + 12];
            // Box centre/size decoded in normalised-square coords…
            let cx = b[0] / side + a.xc;
            let cy = b[1] / side + a.yc;
            let bw = b[2] / side;
            let bh = b[3] / side;
            let [fcx, fcy] = to_frame(cx, cy);
            let mut kp = [[0.0f32; 2]; 4];
            for k in 0..4 {
                let kx = b[4 + 2 * k] / side + a.xc;
                let ky = b[5 + 2 * k] / side + a.yc;
                kp[k] = to_frame(kx, ky);
            }
            dets.push(Detection {
                score: s,
                cx: fcx,
                cy: fcy,
                w: bw * long,
                h: bh * long,
                keypoints: kp,
            });
        }
        // Simple NMS: keep the highest score, drop overlaps (>0.3 IoU).
        dets.sort_by(|a, b| b.score.total_cmp(&a.score));
        let mut kept: Vec<Detection> = Vec::new();
        for d in dets {
            if kept.iter().any(|k| iou(k, &d) > 0.3) {
                continue;
            }
            kept.push(d);
        }
        Ok(kept.into_iter().next())
    }

    /// One-shot pipeline: detect the person, build the ROI, run the landmark
    /// model, return the 33 body landmarks in **frame pixels** (`None` if no
    /// person). Re-detects on every call — use [`BlazePose::poll`] for a live
    /// stream (it tracks and only re-detects when the subject is lost).
    pub fn detect(
        &mut self,
        frame: &[u8],
        w: u32,
        h: u32,
        layout: PixelLayout,
    ) -> Result<Option<Pose>, Error> {
        match self.detect_full(frame, w, h, layout)? {
            Some((pose, a0, a1)) => {
                // One-shot = a fresh acquisition: restart the smoothing
                // banks (their first sample passes through unchanged).
                self.reset_smoothing();
                let (pose, a0, a1) = self.smooth_tracked(pose, a0, a1);
                self.last_aux = Some((a0, a1));
                Ok(Some(pose))
            }
            None => {
                self.last_aux = None;
                Ok(None)
            }
        }
    }

    /// Live entry point — MediaPipe's **detect-once-then-track**. While a subject
    /// is tracked we skip the detector entirely (its ROI jitter is what makes a
    /// still skeleton tremble) and seed the ROI from the previous frame's
    /// auxiliary landmarks; the detector re-runs only when tracking is lost.
    /// Also faster than [`BlazePose::detect`] (no detector while tracking).
    pub fn poll(
        &mut self,
        frame: &[u8],
        w: u32,
        h: u32,
        layout: PixelLayout,
    ) -> Result<Option<Pose>, Error> {
        if let Some((a0, a1)) = self.last_aux {
            let roi = roi_from_points(a0, a1);
            if let Some((pose, na0, na1)) = self.run_landmarks(frame, w, h, layout, roi)? {
                if pose.presence >= TRACK_MIN_PRESENCE {
                    let (pose, na0, na1) = self.smooth_tracked(pose, na0, na1);
                    self.last_aux = Some((na0, na1));
                    return Ok(Some(pose));
                }
            }
        }
        // No track yet, or tracking lost → re-acquire with the detector,
        // restarting the smoothing banks on the fresh subject.
        match self.detect_full(frame, w, h, layout)? {
            Some((pose, a0, a1)) => {
                self.reset_smoothing();
                let (pose, a0, a1) = self.smooth_tracked(pose, a0, a1);
                self.last_aux = Some((a0, a1));
                Ok(Some(pose))
            }
            None => {
                self.last_aux = None;
                Ok(None)
            }
        }
    }

    /// Detector → ROI → landmark model. Returns the pose plus the two auxiliary
    /// ROI points (landmarks 33 & 34, frame px) that seed the next frame's ROI.
    fn detect_full(
        &mut self,
        frame: &[u8],
        w: u32,
        h: u32,
        layout: PixelLayout,
    ) -> Result<Option<TrackedPose>, Error> {
        let Some(det) = self.detect_person(frame, w, h, layout, 0.5)? else {
            return Ok(None);
        };
        // ROI from the detector keypoints (MediaPipe AlignmentPointsRects for
        // pose): centre = kp0 (mid-hip), kp0→kp1 defines scale + rotation.
        let roi = roi_from_points(det.keypoints[0], det.keypoints[1]);
        self.run_landmarks(frame, w, h, layout, roi)
    }

    /// Warp the rotated ROI, run the landmark model, decode the 33 body
    /// landmarks to frame pixels, and inverse-warp the two auxiliary ROI points
    /// (landmarks 33 = centre, 34 = scale/rotation) for tracking. `None` if the
    /// model output is too small to hold them.
    fn run_landmarks(
        &mut self,
        frame: &[u8],
        w: u32,
        h: u32,
        layout: PixelLayout,
        roi: Roi,
    ) -> Result<Option<TrackedPose>, Error> {
        let Roi {
            center,
            box_size,
            angle,
        } = roi;
        let (cos, sin) = (angle.cos(), angle.sin());
        let side = LANDMARK_SIDE as f32;

        // Warp the rotated ROI into a side×side NHWC buffer, [-1,1].
        let mut input = vec![0f32; (LANDMARK_SIDE * LANDMARK_SIDE * 3) as usize];
        let mut idx = 0;
        for v in 0..LANDMARK_SIDE {
            for u in 0..LANDMARK_SIDE {
                let nx = (u as f32 + 0.5) / side - 0.5;
                let ny = (v as f32 + 0.5) / side - 0.5;
                let rx = nx * box_size;
                let ry = ny * box_size;
                let fx = center[0] + rx * cos - ry * sin;
                let fy = center[1] + rx * sin + ry * cos;
                let [r, g, b] = sample_bilinear(frame, w, h, fx, fy, layout);
                input[idx] = f32::from(r) / 127.5 - 1.0;
                input[idx + 1] = f32::from(g) / 127.5 - 1.0;
                input[idx + 2] = f32::from(b) / 127.5 - 1.0;
                idx += 3;
            }
        }
        let val = Tensor::from_array((
            vec![1usize, LANDMARK_SIDE as usize, LANDMARK_SIDE as usize, 3],
            input,
        ))?;
        let outputs = self.landmark.run(ort::inputs![val])?;
        let (_, ld) = outputs["Identity"].try_extract_tensor::<f32>()?;
        let (_, pres) = outputs["Identity_1"].try_extract_tensor::<f32>()?;
        // Need 35 landmarks: 33 body + the 2 auxiliary ROI points (33 & 34).
        if ld.len() < 35 * 5 {
            return Ok(None);
        }
        let presence = sigmoid(pres[0]);

        // Inverse-warp a ROI-space point (px, 0..side) back to frame pixels.
        let unwarp = |lx: f32, ly: f32| -> [f32; 2] {
            let nx = lx / side - 0.5;
            let ny = ly / side - 0.5;
            let rx = nx * box_size;
            let ry = ny * box_size;
            [
                center[0] + rx * cos - ry * sin,
                center[1] + rx * sin + ry * cos,
            ]
        };

        // Decode the 33 body landmarks.
        let mut landmarks = [Landmark::default(); 33];
        for (i, lm) in landmarks.iter_mut().enumerate() {
            let [x, y] = unwarp(ld[i * 5], ld[i * 5 + 1]);
            *lm = Landmark {
                x,
                y,
                z: ld[i * 5 + 2],
                visibility: sigmoid(ld[i * 5 + 3]),
                presence: sigmoid(ld[i * 5 + 4]),
            };
        }
        // Auxiliary ROI points → seed the next frame's tracking ROI.
        let aux0 = unwarp(ld[33 * 5], ld[33 * 5 + 1]);
        let aux1 = unwarp(ld[34 * 5], ld[34 * 5 + 1]);

        Ok(Some((
            Pose {
                landmarks,
                presence,
            },
            aux0,
            aux1,
        )))
    }

    /// Probe: run the landmark model on a blank ROI and return output
    /// tensor names/shapes — scaffolding to pin down the landmark decode.
    pub fn debug_landmark(&mut self) -> Result<Vec<RawOutput>, Error> {
        let n = (LANDMARK_SIDE * LANDMARK_SIDE * 3) as usize;
        let val = Tensor::from_array((
            vec![1usize, LANDMARK_SIDE as usize, LANDMARK_SIDE as usize, 3],
            vec![0f32; n],
        ))?;
        let outputs = self.landmark.run(ort::inputs![val])?;
        let mut info = Vec::new();
        for (name, value) in outputs.iter() {
            let (shape, data) = value.try_extract_tensor::<f32>()?;
            let min = data.iter().copied().fold(f32::INFINITY, f32::min);
            let max = data.iter().copied().fold(f32::NEG_INFINITY, f32::max);
            info.push(RawOutput {
                name: name.to_string(),
                shape: shape.to_vec(),
                min,
                max,
                positive: data.len(),
            });
        }
        Ok(info)
    }
}

/// Letterbox `rgb` (w×h, RGB888) into a `side`×`side` NHWC f32 buffer,
/// normalised to `[-1, 1]`. Pads the shorter side with black to keep aspect
/// (MediaPipe pads to the longer side, then resizes). Returns row-major
/// `[side*side*3]`.
fn letterbox_nhwc(frame: &[u8], w: u32, h: u32, side: u32, layout: PixelLayout) -> Vec<f32> {
    // Fit the frame into `side` preserving aspect, then centre-pad to a square.
    // Same geometry as padding to `long²` first (the decoder undoes it with
    // `long`), but WITHOUT allocating/overlaying/resizing that huge square —
    // for a 1080p frame the old path built an 1920² (~11 MB) buffer every call.
    let long = w.max(h) as f32;
    let scale = side as f32 / long;
    let nw = ((w as f32 * scale).round() as u32).clamp(1, side);
    let nh = ((h as f32 * scale).round() as u32).clamp(1, side);
    // Black padding (Rgb 0) normalises to -1.0.
    let mut out = vec![-1.0f32; (side * side * 3) as usize];
    let ox = (side - nw) / 2;
    let oy = (side - nh) / 2;
    // The source buffer is *borrowed*, not copied: `ImageBuffer` accepts any
    // `Deref<Target = [u8]>` container. It used to be `to_vec()`d, which for a
    // 1080p frame is 6.2 MB of memcpy per call, per model.
    match layout {
        PixelLayout::Rgb888 => {
            let img = image::ImageBuffer::<image::Rgb<u8>, &[u8]>::from_raw(w, h, frame)
                .expect("rgb buffer length must be w*h*3");
            let small =
                image::imageops::resize(&img, nw, nh, image::imageops::FilterType::Triangle);
            fill_nhwc(&mut out, &small, side, ox, oy, [0, 1, 2]);
        }
        // Resizing is per-channel and channel-agnostic, so BGRX rides through
        // as if it were RGBA; only the read-out order differs, and X is dropped
        // there. One wasted channel through the filter buys a whole repack.
        PixelLayout::Bgrx8888 => {
            let img = image::ImageBuffer::<image::Rgba<u8>, &[u8]>::from_raw(w, h, frame)
                .expect("bgrx buffer length must be w*h*4");
            let small =
                image::imageops::resize(&img, nw, nh, image::imageops::FilterType::Triangle);
            fill_nhwc(&mut out, &small, side, ox, oy, [2, 1, 0]);
        }
    }
    out
}

/// Write the resized frame into the centre of the padded NHWC buffer,
/// normalised to `[-1, 1]`, picking channels in `order`.
fn fill_nhwc<P>(
    out: &mut [f32],
    small: &image::ImageBuffer<P, Vec<u8>>,
    side: u32,
    ox: u32,
    oy: u32,
    order: [usize; 3],
) where
    P: image::Pixel<Subpixel = u8>,
{
    for y in 0..small.height() {
        let row = ((oy + y) * side + ox) as usize * 3;
        for x in 0..small.width() {
            let p = small.get_pixel(x, y).channels();
            let di = row + x as usize * 3;
            for (c, &src) in order.iter().enumerate() {
                out[di + c] = f32::from(p[src]) / 127.5 - 1.0;
            }
        }
    }
}

#[cfg(test)]
mod layout_tests {
    use super::*;

    /// A frame with structure in every channel, in both layouts.
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
    /// repacked RGB888 produced — otherwise skipping the repack would quietly
    /// change what the model sees.
    #[test]
    fn letterbox_is_layout_independent() {
        let (w, h) = (160u32, 90u32);
        let (rgb, bgrx) = pair(w, h);
        let from_rgb = letterbox_nhwc(&rgb, w, h, DETECTOR_SIDE, PixelLayout::Rgb888);
        let from_bgrx = letterbox_nhwc(&bgrx, w, h, DETECTOR_SIDE, PixelLayout::Bgrx8888);
        assert_eq!(from_rgb.len(), from_bgrx.len());
        // Bit-for-bit: the resize is per-channel, so relabelling the channels
        // cannot move a value.
        assert!(
            from_rgb
                .iter()
                .zip(&from_bgrx)
                .all(|(a, b)| a.to_bits() == b.to_bits()),
            "letterbox differs between RGB888 and BGRX"
        );
        // Guard against a degenerate all-padding tensor passing vacuously.
        assert!(from_rgb.iter().any(|v| *v > -1.0));
    }

    #[test]
    fn bilinear_sampling_is_layout_independent() {
        let (w, h) = (64u32, 48u32);
        let (rgb, bgrx) = pair(w, h);
        for (x, y) in [(0.0, 0.0), (10.4, 7.6), (31.5, 23.5), (62.9, 46.9)] {
            assert_eq!(
                sample_bilinear(&rgb, w, h, x, y, PixelLayout::Rgb888),
                sample_bilinear(&bgrx, w, h, x, y, PixelLayout::Bgrx8888),
                "sample differs at ({x}, {y})"
            );
        }
    }
}
