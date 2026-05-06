//! VPU (Visual Pinball Units) <-> metric conversions.
//!
//! Source: `../vpinball/plugins/plugins/VPXPlugin.h`. 50 VPU == 1.0625 inch
//! (the size of a standard pinball, 26.9875 mm).

// Computed in f64 to match the precision of VPX's `MMTOVPU` / `VPUTOMM` macros
// (which evaluate in C `double`). We only narrow to f32 at the public boundary
// so chained conversions don't drift; otherwise f32-only arithmetic on these
// constants accumulates ~0.02% error per round-trip.
const MM_PER_VPU: f64 = 25.4 * 1.0625 / 50.0;
const VPU_PER_MM: f64 = 50.0 / (25.4 * 1.0625);

#[inline]
pub fn mm_to_vpu(mm: f32) -> f32 {
    (f64::from(mm) * VPU_PER_MM) as f32
}

#[inline]
pub fn vpu_to_mm(vpu: f32) -> f32 {
    (f64::from(vpu) * MM_PER_VPU) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_is_identity() {
        for mm in [0.0_f32, 1.0, 12.34, -100.0, 1500.0] {
            let back = vpu_to_mm(mm_to_vpu(mm));
            assert!(
                (back - mm).abs() < 1e-3,
                "round trip failed: {mm} -> {back}"
            );
        }
    }

    #[test]
    fn ball_diameter_is_50_vpu() {
        // 1.0625 inches == 26.9875 mm == 50 VPU.
        let vpu = mm_to_vpu(26.987_5);
        assert!((vpu - 50.0).abs() < 1e-3, "ball diameter: {vpu} VPU");
    }
}
