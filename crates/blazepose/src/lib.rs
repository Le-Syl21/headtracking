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
fn sample_bilinear(rgb: &[u8], w: u32, h: u32, x: f32, y: f32) -> [u8; 3] {
    if x < 0.0 || y < 0.0 || x > (w - 1) as f32 || y > (h - 1) as f32 {
        return [0, 0, 0];
    }
    let x0 = x.floor() as u32;
    let y0 = y.floor() as u32;
    let x1 = (x0 + 1).min(w - 1);
    let y1 = (y0 + 1).min(h - 1);
    let fx = x - x0 as f32;
    let fy = y - y0 as f32;
    let px = |xx: u32, yy: u32, c: usize| f32::from(rgb[((yy * w + xx) * 3) as usize + c]);
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

pub struct BlazePose {
    detector: Session,
    landmark: Session,
    anchors: Vec<Anchor>,
}

impl BlazePose {
    pub fn new() -> Result<Self, Error> {
        let detector = Session::builder()?.commit_from_memory(DETECTOR_ONNX)?;
        let landmark = Session::builder()?.commit_from_memory(LANDMARK_ONNX)?;
        Ok(Self {
            detector,
            landmark,
            anchors: generate_anchors(),
        })
    }

    /// Run the detector and return the best person detection (score-thresholded,
    /// NMS-suppressed), with coordinates in the original frame's pixel space.
    pub fn detect_person(
        &mut self,
        rgb: &[u8],
        w: u32,
        h: u32,
        score_threshold: f32,
    ) -> Result<Option<Detection>, Error> {
        let side = DETECTOR_SIDE as f32;
        let input = letterbox_nhwc(rgb, w, h, DETECTOR_SIDE);
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

    /// Full pipeline: detect the person, build the rotated ROI, run the
    /// landmark model, and return the 33 body landmarks in **frame pixels**.
    /// `None` if no person is detected.
    pub fn detect(&mut self, rgb: &[u8], w: u32, h: u32) -> Result<Option<Pose>, Error> {
        let Some(det) = self.detect_person(rgb, w, h, 0.5)? else {
            return Ok(None);
        };
        // ROI from the detector keypoints (MediaPipe AlignmentPointsRects for
        // pose): centre = kp0 (mid-hip), rotate so kp0→kp1 points "up" (90°),
        // square size = 2·|kp0→kp1| scaled by 1.25.
        let c = det.keypoints[0];
        let a = det.keypoints[1];
        let (dx, dy) = (a[0] - c[0], a[1] - c[1]);
        let dist = (dx * dx + dy * dy).sqrt();
        let box_size = 2.0 * dist * 1.25;
        let angle = std::f32::consts::FRAC_PI_2 - (-dy).atan2(dx);
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
                let fx = c[0] + rx * cos - ry * sin;
                let fy = c[1] + rx * sin + ry * cos;
                let [r, g, b] = sample_bilinear(rgb, w, h, fx, fy);
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
        let presence = sigmoid(pres[0]);

        // Decode the first 33 landmarks: ROI px (0..side) → inverse warp → frame.
        let mut landmarks = [Landmark::default(); 33];
        for (i, lm) in landmarks.iter_mut().enumerate() {
            let lx = ld[i * 5];
            let ly = ld[i * 5 + 1];
            let lz = ld[i * 5 + 2];
            let nx = lx / side - 0.5;
            let ny = ly / side - 0.5;
            let rx = nx * box_size;
            let ry = ny * box_size;
            *lm = Landmark {
                x: c[0] + rx * cos - ry * sin,
                y: c[1] + rx * sin + ry * cos,
                z: lz,
                visibility: sigmoid(ld[i * 5 + 3]),
                presence: sigmoid(ld[i * 5 + 4]),
            };
        }
        Ok(Some(Pose {
            landmarks,
            presence,
        }))
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

    /// Run the detector on a frame and return every output tensor's
    /// name/shape/range — a scaffolding probe to pin down the decode target
    /// (output order, whether scores need a sigmoid, anchor count).
    pub fn debug_detector(&mut self, rgb: &[u8], w: u32, h: u32) -> Result<Vec<RawOutput>, Error> {
        let input = letterbox_nhwc(rgb, w, h, DETECTOR_SIDE);
        let val = Tensor::from_array((
            vec![1usize, DETECTOR_SIDE as usize, DETECTOR_SIDE as usize, 3],
            input,
        ))?;
        let outputs = self.detector.run(ort::inputs![val])?;
        let mut info = Vec::new();
        for (name, value) in outputs.iter() {
            let (shape, data) = value.try_extract_tensor::<f32>()?;
            let min = data.iter().copied().fold(f32::INFINITY, f32::min);
            let max = data.iter().copied().fold(f32::NEG_INFINITY, f32::max);
            let positive = data.iter().filter(|&&x| x > 0.0).count();
            info.push(RawOutput {
                name: name.to_string(),
                shape: shape.to_vec(),
                min,
                max,
                positive,
            });
        }
        Ok(info)
    }
}

/// Letterbox `rgb` (w×h, RGB888) into a `side`×`side` NHWC f32 buffer,
/// normalised to `[-1, 1]`. Pads the shorter side with black to keep aspect
/// (MediaPipe pads to the longer side, then resizes). Returns row-major
/// `[side*side*3]`.
fn letterbox_nhwc(rgb: &[u8], w: u32, h: u32, side: u32) -> Vec<f32> {
    let img =
        image::RgbImage::from_raw(w, h, rgb.to_vec()).expect("rgb buffer length must be w*h*3");
    // Fit the frame into `side` preserving aspect, then centre-pad to a square.
    // Same geometry as padding to `long²` first (the decoder undoes it with
    // `long`), but WITHOUT allocating/overlaying/resizing that huge square —
    // for a 1080p frame the old path built an 1920² (~11 MB) buffer every call.
    let long = w.max(h) as f32;
    let scale = side as f32 / long;
    let nw = ((w as f32 * scale).round() as u32).clamp(1, side);
    let nh = ((h as f32 * scale).round() as u32).clamp(1, side);
    let small = image::imageops::resize(&img, nw, nh, image::imageops::FilterType::Triangle);
    // Black padding (Rgb 0) normalises to -1.0.
    let mut out = vec![-1.0f32; (side * side * 3) as usize];
    let ox = (side - nw) / 2;
    let oy = (side - nh) / 2;
    for y in 0..nh {
        let row = ((oy + y) * side + ox) as usize * 3;
        for x in 0..nw {
            let p = small.get_pixel(x, y).0;
            let di = row + x as usize * 3;
            out[di] = f32::from(p[0]) / 127.5 - 1.0;
            out[di + 1] = f32::from(p[1]) / 127.5 - 1.0;
            out[di + 2] = f32::from(p[2]) / 127.5 - 1.0;
        }
    }
    out
}
