//! Turning a Kinect's raw infrared frame into what the IR model was trained on.
//!
//! The sensors hand over 16-bit infrared. Rendering that to 8 bits by a plain
//! rescale — which is what the demo and the shipped `_irview` captures used to
//! do — leaves a near-black image: the median of a real frame lands around 17
//! out of 255. The anchor model detected the cabinet in 59% of those. After the
//! treatment below it detects in 100%, on the same 176 captures and with the
//! same weights, so this is not a training gain but an exposure one.
//!
//! Two steps, and a third that depends on the sensor:
//!
//! * **Square root.** Active illumination falls off with the square of
//!   distance, so a linear rescale spends most of its range on whatever is
//!   nearest. The root compresses that back.
//! * **CLAHE** — contrast-limited adaptive histogram equalisation. A global
//!   auto-level is decided by the brightest region; equalising per tile lets a
//!   dark corner have its own mapping, which is what makes the cabinet's edges
//!   appear at all. The clip limit stops a flat tile from amplifying its noise
//!   into texture.
//! * **A median first, on the Kinect v1 only.** The v1 measures depth by
//!   projecting a dot pattern, and its infrared camera sees that pattern. It is
//!   signal for the sensor and noise for us, and CLAHE would amplify it: the
//!   variance of the Laplacian comes out at 7831 without the median and 163
//!   with it, against 1587 for a v2 frame that needs none.
//!
//! Whatever runs here must match what the training corpus was rendered with.
//! A model fed a distribution it was not trained on is the failure this module
//! exists to prevent, so the corpus is generated from these same steps.

/// Which sensor produced the frame — they need different preparation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrSensor {
    /// Structured light: the infrared image carries the projected dot pattern.
    KinectV1,
    /// Time of flight: no pattern, and a median would only blunt real edges.
    KinectV2,
}

/// Percentile normalisation to 8 bits, the same 0.5/99.5 bounds the corpus uses.
fn to_u8(src: &[f32]) -> Vec<u8> {
    if src.is_empty() {
        return Vec::new();
    }
    let mut sorted: Vec<f32> = src.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let lo = sorted[(sorted.len() as f32 * 0.005) as usize];
    let hi = sorted[((sorted.len() as f32 * 0.995) as usize).min(sorted.len() - 1)];
    let span = (hi - lo).max(1e-6);
    src.iter()
        .map(|&v| (((v - lo) * 255.0 / span).clamp(0.0, 255.0)) as u8)
        .collect()
}

/// 5x5 median, to erase the v1's projected dots without rounding its edges.
fn median5(src: &[u8], w: usize, h: usize) -> Vec<u8> {
    let mut out = src.to_vec();
    let mut win = [0u8; 25];
    for y in 2..h.saturating_sub(2) {
        for x in 2..w.saturating_sub(2) {
            let mut n = 0;
            for dy in 0..5 {
                for dx in 0..5 {
                    win[n] = src[(y + dy - 2) * w + (x + dx - 2)];
                    n += 1;
                }
            }
            win.sort_unstable();
            out[y * w + x] = win[12];
        }
    }
    out
}

/// Number of tiles per axis, and the clip limit, both matching the corpus.
const TILES: usize = 8;
const CLIP: f32 = 3.0;

/// Contrast-limited adaptive histogram equalisation.
///
/// One mapping per tile, then bilinear interpolation between the four
/// surrounding tile centres so no tile boundary shows.
fn clahe(src: &[u8], w: usize, h: usize) -> Vec<u8> {
    if w == 0 || h == 0 {
        return Vec::new();
    }
    let (tw, th) = (w.div_ceil(TILES), h.div_ceil(TILES));
    let limit = ((CLIP * (tw * th) as f32 / 256.0) as u32).max(1);

    // A lookup table per tile.
    let mut luts = vec![[0u8; 256]; TILES * TILES];
    for ty in 0..TILES {
        for tx in 0..TILES {
            let mut hist = [0u32; 256];
            let (x0, y0) = (tx * tw, ty * th);
            let (x1, y1) = ((x0 + tw).min(w), (y0 + th).min(h));
            for y in y0..y1 {
                for x in x0..x1 {
                    hist[src[y * w + x] as usize] += 1;
                }
            }
            // Clip, then hand the excess back evenly: that is what keeps a flat
            // tile from turning its own noise into contrast.
            let mut excess = 0u32;
            for c in &mut hist {
                if *c > limit {
                    excess += *c - limit;
                    *c = limit;
                }
            }
            let share = excess / 256;
            let mut rest = excess % 256;
            for c in &mut hist {
                *c += share;
                if rest > 0 {
                    *c += 1;
                    rest -= 1;
                }
            }
            let total: u32 = hist.iter().sum();
            let scale = 255.0 / total.max(1) as f32;
            let mut acc = 0u32;
            let lut = &mut luts[ty * TILES + tx];
            for (i, &c) in hist.iter().enumerate() {
                acc += c;
                lut[i] = (acc as f32 * scale).min(255.0) as u8;
            }
        }
    }

    // Interpolate between tile mappings.
    let mut out = vec![0u8; w * h];
    for y in 0..h {
        let fy = (y as f32 / th as f32) - 0.5;
        let ty0 = fy.floor().max(0.0) as usize;
        let ty1 = (ty0 + 1).min(TILES - 1);
        let wy = (fy - fy.floor()).clamp(0.0, 1.0);
        let wy = if fy < 0.0 { 0.0 } else { wy };
        for x in 0..w {
            let fx = (x as f32 / tw as f32) - 0.5;
            let tx0 = fx.floor().max(0.0) as usize;
            let tx1 = (tx0 + 1).min(TILES - 1);
            let wx = (fx - fx.floor()).clamp(0.0, 1.0);
            let wx = if fx < 0.0 { 0.0 } else { wx };
            let v = src[y * w + x] as usize;
            let (a, b) = (
                luts[ty0 * TILES + tx0][v] as f32,
                luts[ty0 * TILES + tx1][v] as f32,
            );
            let (c, d) = (
                luts[ty1 * TILES + tx0][v] as f32,
                luts[ty1 * TILES + tx1][v] as f32,
            );
            let top = a + (b - a) * wx;
            let bot = c + (d - c) * wx;
            out[y * w + x] = (top + (bot - top) * wy).round().clamp(0.0, 255.0) as u8;
        }
    }
    out
}

/// Prepare a raw 16-bit infrared frame for the anchor model, in grayscale.
#[must_use]
pub fn prepare_ir(raw: &[u16], w: u32, h: u32, sensor: IrSensor) -> Vec<u8> {
    let (wu, hu) = (w as usize, h as usize);
    let rooted: Vec<f32> = raw.iter().map(|&v| f32::from(v).sqrt()).collect();
    let mut g = to_u8(&rooted);
    if sensor == IrSensor::KinectV1 {
        g = median5(&g, wu, hu);
    }
    clahe(&g, wu, hu)
}

/// The same, packed to RGB888 for a detector that wants three channels.
#[must_use]
pub fn prepare_ir_rgb888(raw: &[u16], w: u32, h: u32, sensor: IrSensor) -> Vec<u8> {
    let g = prepare_ir(raw, w, h, sensor);
    let mut out = Vec::with_capacity(g.len() * 3);
    for v in g {
        out.extend_from_slice(&[v, v, v]);
    }
    out
}
