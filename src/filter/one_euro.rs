//! 1€ filter (Casiez, Roussel, Vogel, CHI 2012) — adaptive low-pass for
//! head-tracking pose smoothing. The cutoff frequency increases with motion
//! speed so fast moves stay responsive while idle holds get jitter-free.
//!
//! Reference: <https://gery.casiez.net/1euro/>
//!
//! Typical parameters for head tracking at 30 Hz:
//! * `min_cutoff_hz` ≈ 1.0   — baseline smoothing when the user is still.
//! * `beta` ≈ 0.01           — how much faster motion bumps the cutoff.
//! * `derivative_cutoff_hz` ≈ 1.0  — smoothing of the derivative itself.

use std::f32::consts::TAU;

const DEFAULT_MIN_CUTOFF_HZ: f32 = 1.0;
const DEFAULT_BETA: f32 = 0.01;
const DEFAULT_DERIVATIVE_CUTOFF_HZ: f32 = 1.0;

#[derive(Debug, Clone, Copy)]
pub struct OneEuroParams {
    pub min_cutoff_hz: f32,
    pub beta: f32,
    pub derivative_cutoff_hz: f32,
}

impl Default for OneEuroParams {
    fn default() -> Self {
        Self {
            min_cutoff_hz: DEFAULT_MIN_CUTOFF_HZ,
            beta: DEFAULT_BETA,
            derivative_cutoff_hz: DEFAULT_DERIVATIVE_CUTOFF_HZ,
        }
    }
}

/// Adaptive low-pass on a single channel (one axis of a pose).
#[derive(Debug, Clone)]
pub struct OneEuro {
    params: OneEuroParams,
    prev: Option<State>,
}

#[derive(Debug, Clone, Copy)]
struct State {
    x: f32,
    dx: f32,
    t_us: u64,
}

impl OneEuro {
    pub fn new(params: OneEuroParams) -> Self {
        Self { params, prev: None }
    }

    pub fn with_defaults() -> Self {
        Self::new(OneEuroParams::default())
    }

    /// Forget all history. Useful when the underlying signal changes
    /// reference (different sensor, different game session, etc.).
    pub fn reset(&mut self) {
        self.prev = None;
    }

    /// Mutate the live parameters without dropping the filter state.
    pub fn set_params(&mut self, params: OneEuroParams) {
        self.params = params;
    }

    /// Push a fresh sample and return the smoothed value.
    pub fn update(&mut self, x: f32, t_us: u64) -> f32 {
        let Some(prev) = self.prev else {
            self.prev = Some(State { x, dx: 0.0, t_us });
            return x;
        };
        let dt = t_us.saturating_sub(prev.t_us) as f32 * 1e-6;
        if dt <= 0.0 {
            // Duplicate or out-of-order timestamp: hand back the last
            // smoothed value without poisoning the state.
            return prev.x;
        }

        // Smooth the derivative first.
        let dx_raw = (x - prev.x) / dt;
        let alpha_d = alpha_for(self.params.derivative_cutoff_hz, dt);
        let dx_smoothed = alpha_d * dx_raw + (1.0 - alpha_d) * prev.dx;

        // Adaptive cutoff: faster motion → higher cutoff → more responsive.
        let cutoff = self.params.min_cutoff_hz + self.params.beta * dx_smoothed.abs();
        let alpha_x = alpha_for(cutoff, dt);
        let x_smoothed = alpha_x * x + (1.0 - alpha_x) * prev.x;

        self.prev = Some(State {
            x: x_smoothed,
            dx: dx_smoothed,
            t_us,
        });
        x_smoothed
    }
}

fn alpha_for(cutoff_hz: f32, dt: f32) -> f32 {
    let tau = 1.0 / (TAU * cutoff_hz);
    1.0 / (1.0 + tau / dt)
}

/// Three independent 1€ filters bundled for a [`crate::tracker::Pose`]'s
/// `position_mm`. Each axis carries its own state.
#[derive(Debug, Clone)]
pub struct OneEuroPose3D {
    axes: [OneEuro; 3],
}

impl OneEuroPose3D {
    pub fn new(params: OneEuroParams) -> Self {
        Self {
            axes: [
                OneEuro::new(params),
                OneEuro::new(params),
                OneEuro::new(params),
            ],
        }
    }

    pub fn with_defaults() -> Self {
        Self::new(OneEuroParams::default())
    }

    pub fn reset(&mut self) {
        for a in &mut self.axes {
            a.reset();
        }
    }

    pub fn set_params(&mut self, params: OneEuroParams) {
        for a in &mut self.axes {
            a.set_params(params);
        }
    }

    pub fn update(&mut self, position_mm: [f32; 3], t_us: u64) -> [f32; 3] {
        [
            self.axes[0].update(position_mm[0], t_us),
            self.axes[1].update(position_mm[1], t_us),
            self.axes[2].update(position_mm[2], t_us),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_sample_passes_through() {
        let mut f = OneEuro::with_defaults();
        assert_eq!(f.update(42.0, 0), 42.0);
    }

    #[test]
    fn constant_input_converges() {
        let mut f = OneEuro::with_defaults();
        let mut t = 0_u64;
        let _ = f.update(100.0, t);
        let mut last = 100.0;
        for _ in 0..200 {
            t += 33_333; // ≈30 Hz
            last = f.update(100.0, t);
        }
        assert!(
            (last - 100.0).abs() < 1e-3,
            "constant input should stabilise at the value, got {last}"
        );
    }

    #[test]
    fn step_response_is_smoothed_not_instant() {
        let mut f = OneEuro::with_defaults();
        let mut t = 0_u64;
        // Settle on 0.0
        let _ = f.update(0.0, t);
        for _ in 0..5 {
            t += 33_333;
            f.update(0.0, t);
        }
        // Step to 100 mm in one sample
        t += 33_333;
        let after_step = f.update(100.0, t);
        assert!(
            (0.0..100.0).contains(&after_step),
            "step output should be smoothed, got {after_step}"
        );
    }

    #[test]
    fn out_of_order_timestamps_dont_panic() {
        let mut f = OneEuro::with_defaults();
        let _ = f.update(1.0, 1_000);
        // Same or earlier timestamp — should hand back the previous output.
        let same_t = f.update(2.0, 1_000);
        assert_eq!(same_t, 1.0);
        let earlier = f.update(3.0, 500);
        assert_eq!(earlier, 1.0);
    }

    #[test]
    fn pose3d_axes_are_independent() {
        let mut f = OneEuroPose3D::with_defaults();
        let out = f.update([10.0, 20.0, 30.0], 0);
        assert_eq!(out, [10.0, 20.0, 30.0]);
        let out2 = f.update([10.0, 20.0, 30.0], 33_333);
        // Same input → axes converge to the same input independently.
        assert!((out2[0] - 10.0).abs() < 1e-3);
        assert!((out2[1] - 20.0).abs() < 1e-3);
        assert!((out2[2] - 30.0).abs() < 1e-3);
    }

    #[test]
    fn high_beta_tracks_fast_motion_with_less_lag() {
        // A higher beta should let the filter follow a fast ramp more
        // closely than the default. Checked by comparing the filter
        // output's distance to the input at a fixed step.
        let ramp = (0..30).map(|i| (i as f32) * 5.0); // 0, 5, …, 145
        let dt_us: u64 = 33_333;

        let mut low = OneEuro::with_defaults();
        let mut high = OneEuro::new(OneEuroParams {
            beta: 1.0,
            ..OneEuroParams::default()
        });
        let mut t = 0_u64;
        let mut last_low = 0.0;
        let mut last_high = 0.0;
        for x in ramp {
            last_low = low.update(x, t);
            last_high = high.update(x, t);
            t += dt_us;
        }
        // Final input is 145.0; high-beta should be closer to it.
        let lag_low = (145.0_f32 - last_low).abs();
        let lag_high = (145.0_f32 - last_high).abs();
        assert!(
            lag_high < lag_low,
            "high-beta lag={lag_high}, low-beta lag={lag_low}"
        );
    }
}
