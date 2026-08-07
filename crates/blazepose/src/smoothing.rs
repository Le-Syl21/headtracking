//! Port of MediaPipe's pose landmark smoothing stage
//! (`pose_landmark_filtering.pbtxt` + `LandmarksSmoothingCalculator`).
//!
//! Two One-Euro banks with MediaPipe's stock parameters:
//!
//! * the 33 **visible landmarks** — `min_cutoff 0.05, beta 80,
//!   derivate_cutoff 1.0`. beta 80 makes the filter follow any real motion
//!   almost instantly; only near-static sub-pixel noise is attenuated, so
//!   the added lag is negligible.
//! * the 2 **auxiliary ROI points** — `min_cutoff 0.01, beta 10,
//!   derivate_cutoff 1.0`. These seed the next frame's tracking crop, so
//!   smoothing them stabilises the landmark model's *input* — it removes
//!   jitter at the source and adds zero output lag by construction.
//!
//! Like MediaPipe, speeds are measured relative to the subject's on-screen
//! size (`value_scale = 1 / object_scale`), which makes the filtering
//! strength independent of camera distance and resolution.

/// MediaPipe `pose_landmark_filtering.pbtxt`, visible-landmarks branch.
pub const LANDMARK_MIN_CUTOFF: f32 = 0.05;
pub const LANDMARK_BETA: f32 = 80.0;
/// MediaPipe `pose_landmark_filtering.pbtxt`, auxiliary-landmarks branch.
pub const AUX_MIN_CUTOFF: f32 = 0.01;
pub const AUX_BETA: f32 = 10.0;
const DERIVATE_CUTOFF: f32 = 1.0;

/// Exponential low-pass; the first sample initialises pass-through.
#[derive(Default, Clone, Copy)]
struct LowPass {
    y: Option<f32>,
}

impl LowPass {
    fn apply(&mut self, alpha: f32, v: f32) -> f32 {
        let y = match self.y {
            None => v,
            Some(prev) => alpha * v + (1.0 - alpha) * prev,
        };
        self.y = Some(y);
        y
    }
}

/// One scalar One-Euro filter (MediaPipe `one_euro_filter.cc` semantics:
/// the speed term is scaled by `value_scale` before entering beta).
#[derive(Clone, Copy)]
struct OneEuro {
    min_cutoff: f32,
    beta: f32,
    x: LowPass,
    dx: LowPass,
    last_raw: Option<f32>,
    last_t: Option<f64>,
}

impl OneEuro {
    fn new(min_cutoff: f32, beta: f32) -> Self {
        Self {
            min_cutoff,
            beta,
            x: LowPass::default(),
            dx: LowPass::default(),
            last_raw: None,
            last_t: None,
        }
    }

    fn alpha(cutoff: f32, dt: f32) -> f32 {
        let tau = 1.0 / (2.0 * std::f32::consts::PI * cutoff);
        1.0 / (1.0 + tau / dt)
    }

    fn apply(&mut self, t: f64, value_scale: f32, v: f32) -> f32 {
        let Some(prev_t) = self.last_t else {
            self.last_t = Some(t);
            self.last_raw = Some(v);
            self.dx.apply(1.0, 0.0);
            return self.x.apply(1.0, v);
        };
        #[allow(clippy::cast_possible_truncation)]
        let dt = ((t - prev_t) as f32).max(1e-6);
        let d_raw = (v - self.last_raw.unwrap_or(v)) * value_scale / dt;
        self.last_t = Some(t);
        self.last_raw = Some(v);
        let d = self.dx.apply(Self::alpha(DERIVATE_CUTOFF, dt), d_raw);
        let cutoff = self.min_cutoff + self.beta * d.abs();
        self.x.apply(Self::alpha(cutoff, dt), v)
    }

    fn reset(&mut self) {
        self.x = LowPass::default();
        self.dx = LowPass::default();
        self.last_raw = None;
        self.last_t = None;
    }
}

/// A bank of One-Euro filters over `N` 3-component points, sharing one
/// per-frame `value_scale`.
pub struct PointBank<const N: usize> {
    filters: [[OneEuro; 3]; N],
}

impl<const N: usize> PointBank<N> {
    #[must_use]
    pub fn new(min_cutoff: f32, beta: f32) -> Self {
        Self {
            filters: [[OneEuro::new(min_cutoff, beta); 3]; N],
        }
    }

    /// Filter point `i` at timestamp `t` (seconds, monotonic).
    pub fn apply(&mut self, i: usize, t: f64, value_scale: f32, p: [f32; 3]) -> [f32; 3] {
        let f = &mut self.filters[i];
        [
            f[0].apply(t, value_scale, p[0]),
            f[1].apply(t, value_scale, p[1]),
            f[2].apply(t, value_scale, p[2]),
        ]
    }

    /// Drop all history — call when tracking is (re)acquired so the filters
    /// re-initialise on the new subject instead of dragging from the old one.
    pub fn reset(&mut self) {
        for point in &mut self.filters {
            for f in point {
                f.reset();
            }
        }
    }
}

/// MediaPipe `GetObjectScale`: mean of the landmark bounding-box sides.
/// The reciprocal feeds the banks as `value_scale`.
#[must_use]
pub fn object_scale(points: impl Iterator<Item = [f32; 2]>) -> f32 {
    let mut min = [f32::INFINITY; 2];
    let mut max = [f32::NEG_INFINITY; 2];
    for [x, y] in points {
        min[0] = min[0].min(x);
        min[1] = min[1].min(y);
        max[0] = max[0].max(x);
        max[1] = max[1].max(y);
    }
    if min[0] > max[0] {
        return 1.0;
    }
    (((max[0] - min[0]) + (max[1] - min[1])) * 0.5).max(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1.0 / 30.0;

    /// Pseudo-random noise in [-1, 1] without pulling in a rand dep.
    fn noise(i: u32) -> f32 {
        let h = i.wrapping_mul(2_654_435_761) >> 16;
        (f32::from(h as u16) / f32::from(u16::MAX)) * 2.0 - 1.0
    }

    #[test]
    fn static_noise_is_attenuated() {
        let mut bank = PointBank::<1>::new(LANDMARK_MIN_CUTOFF, LANDMARK_BETA);
        let scale = 1.0 / 300.0; // subject ~300 px on screen
        let mut raw_dev = 0.0f32;
        let mut out_dev = 0.0f32;
        for i in 0..200 {
            let v = 100.0 + noise(i); // ±1 px jitter around a still point
            let out = bank.apply(0, f64::from(i) * DT, scale, [v, v, 0.0]);
            if i > 20 {
                raw_dev += (v - 100.0).abs();
                out_dev += (out[0] - 100.0).abs();
            }
        }
        assert!(
            out_dev < raw_dev * 0.35,
            "smoothing should cut static jitter: raw {raw_dev}, out {out_dev}"
        );
    }

    #[test]
    fn fast_motion_follows_with_negligible_lag() {
        let mut bank = PointBank::<1>::new(LANDMARK_MIN_CUTOFF, LANDMARK_BETA);
        let scale = 1.0 / 300.0;
        bank.apply(0, 0.0, scale, [0.0, 0.0, 0.0]);
        // 300 px jump (a real head move), then hold.
        let mut out = [0.0f32; 3];
        for i in 1..=3 {
            out = bank.apply(0, f64::from(i) * DT, scale, [300.0, 0.0, 0.0]);
        }
        assert!(
            out[0] > 290.0,
            "beta 80 must catch a real move within 3 frames: {out:?}"
        );
    }

    #[test]
    fn aux_bank_is_stiffer_than_landmark_bank() {
        let mut lm = PointBank::<1>::new(LANDMARK_MIN_CUTOFF, LANDMARK_BETA);
        let mut aux = PointBank::<1>::new(AUX_MIN_CUTOFF, AUX_BETA);
        let scale = 1.0 / 300.0;
        let mut lm_dev = 0.0f32;
        let mut aux_dev = 0.0f32;
        for i in 0..200 {
            let v = 100.0 + 2.0 * noise(i);
            let t = f64::from(i) * DT;
            let l = lm.apply(0, t, scale, [v, 0.0, 0.0]);
            let a = aux.apply(0, t, scale, [v, 0.0, 0.0]);
            if i > 20 {
                lm_dev += (l[0] - 100.0).abs();
                aux_dev += (a[0] - 100.0).abs();
            }
        }
        assert!(
            aux_dev < lm_dev,
            "ROI points must be stiffer: lm {lm_dev}, aux {aux_dev}"
        );
    }

    #[test]
    fn reset_forgets_history() {
        let mut bank = PointBank::<1>::new(LANDMARK_MIN_CUTOFF, LANDMARK_BETA);
        bank.apply(0, 0.0, 1.0, [500.0, 0.0, 0.0]);
        bank.reset();
        let out = bank.apply(0, 1.0, 1.0, [10.0, 0.0, 0.0]);
        assert!(
            (out[0] - 10.0).abs() < f32::EPSILON,
            "first sample after reset passes through: {out:?}"
        );
    }

    #[test]
    fn object_scale_is_mean_bbox_side() {
        let pts = [[0.0, 0.0], [100.0, 50.0]];
        let s = object_scale(pts.into_iter());
        assert!((s - 75.0).abs() < f32::EPSILON, "got {s}");
    }
}
