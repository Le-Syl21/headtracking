//! Sliding component-median gate, applied AHEAD of the One-Euro filter.
//!
//! The One-Euro filter is *designed* to follow fast motion, so a one-frame
//! outlier (a BlazePose landmark re-lock, a depth flicker) passes straight
//! through it whatever the tuning. A short median window erases isolated
//! spikes completely, at the cost of `window/2` frames of latency — at
//! 30 fps, a 3-frame window costs ~33 ms, a 9-frame window ~133 ms.

/// Per-component median over the last `window` samples of a 3-vector.
#[derive(Debug, Clone)]
pub struct MedianGate {
    hist: [[f32; 3]; Self::MAX],
    /// Total samples pushed since the last reset (ring write cursor).
    len: usize,
    window: usize,
}

impl MedianGate {
    /// Largest supported window (frames).
    pub const MAX: usize = 9;

    #[must_use]
    pub fn new(window: usize) -> Self {
        let mut gate = Self {
            hist: [[0.0; 3]; Self::MAX],
            len: 0,
            window: 1,
        };
        gate.set_window(window);
        gate
    }

    /// Clamp to `1..=MAX` and force odd (an even-sized median is biased).
    /// A window of 1 disables the gate.
    pub fn set_window(&mut self, window: usize) {
        self.window = window.clamp(1, Self::MAX) | 1;
    }

    #[must_use]
    pub fn window(&self) -> usize {
        self.window
    }

    /// Forget the history (recenter / tracking loss).
    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// Push a sample and get the gated (median) value out. Until the
    /// window fills, the median runs over the samples available so far.
    pub fn push(&mut self, sample: [f32; 3]) -> [f32; 3] {
        self.hist[self.len % Self::MAX] = sample;
        self.len += 1;
        let n = self.len.min(self.window);
        if n < 2 {
            return sample;
        }
        std::array::from_fn(|axis| {
            let mut vals = [0.0f32; Self::MAX];
            for (k, v) in vals.iter_mut().take(n).enumerate() {
                *v = self.hist[(self.len - 1 - k) % Self::MAX][axis];
            }
            let vals = &mut vals[..n];
            vals.sort_by(f32::total_cmp);
            vals[n / 2]
        })
    }
}

#[cfg(test)]
mod tests {
    use super::MedianGate;

    #[test]
    fn window_is_clamped_and_odd() {
        assert_eq!(MedianGate::new(0).window(), 1);
        assert_eq!(MedianGate::new(4).window(), 5);
        assert_eq!(MedianGate::new(99).window(), MedianGate::MAX);
    }

    #[test]
    fn window_of_one_passes_samples_through() {
        let mut g = MedianGate::new(1);
        assert_eq!(g.push([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
        assert_eq!(g.push([9.0, 8.0, 7.0]), [9.0, 8.0, 7.0]);
    }

    #[test]
    fn kills_a_single_frame_spike() {
        let mut g = MedianGate::new(3);
        g.push([700.0; 3]);
        g.push([2400.0; 3]); // wild one-frame outlier
        assert_eq!(g.push([701.0; 3]), [701.0; 3]);
    }

    #[test]
    fn median_runs_per_component() {
        let mut g = MedianGate::new(3);
        g.push([1.0, 10.0, 100.0]);
        g.push([2.0, 30.0, 300.0]);
        assert_eq!(g.push([3.0, 20.0, 200.0]), [2.0, 20.0, 200.0]);
    }

    #[test]
    fn wider_window_survives_two_frame_spikes() {
        let mut g = MedianGate::new(5);
        for v in [700.0, 700.0, 700.0] {
            g.push([v; 3]);
        }
        g.push([2400.0; 3]);
        let out = g.push([2400.0; 3]); // two-frame burst, window 5
        assert_eq!(out, [700.0; 3]);
    }

    #[test]
    fn reset_forgets_history() {
        let mut g = MedianGate::new(3);
        g.push([700.0; 3]);
        g.push([700.0; 3]);
        g.reset();
        assert_eq!(g.push([100.0; 3]), [100.0; 3]);
    }
}
