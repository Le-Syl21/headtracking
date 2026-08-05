//! **Automatic cabinet calibration** — the core idea that lets head-tracking
//! work from a plain webcam with *zero install and no manual calibration*.
//!
//! The anchor detector provides the lockbar and the two sidebars. What each
//! measurement is trusted for (settled 2026-08-05 after field validation):
//!
//! * The lockbar's **known physical width** is the one and only metric ruler:
//!   `distance = f · width_mm / width_px`.
//! * The **sidebars** are seen as partial segments; extended, they give the
//!   playfield's depth **vanishing point** — direction only, no metric role.
//! * The lockbar's **band thickness (~70 mm) is deliberately NOT used as a
//!   measurement**: it's a handful of pixels, near-degenerate in any frontal
//!   view. Two focal estimators built on it (orthogonal-VP dot product, and a
//!   Zhang homography of the 610×70 band) were implemented, measured wrong on
//!   real captures (VP: degenerate; band homography: −29 %/+92 % focal error),
//!   and removed — see git history if they're ever needed as reference.
//!
//! Focal length therefore comes from, in order: the sensor's factory value
//! (both Kinects), or — webcam — a homography of the **full playfield
//! rectangle** (lockbar width × playfield depth, both known from the VPX
//! table config; the rails' long lever arm keeps it well-conditioned even
//! with a centred camera). That rectangle calibration is the planned
//! replacement and will produce a [`CabCalibration`].
//!
//! A head pixel (e.g. the BlazePose nose) then deprojects with `f` and maps
//! into the playfield frame via `(R, t)` — full 3D, from one webcam.
//!
//! Pure `f32`, no external maths crate, so it drops into the plugin as-is.

/// Recovered camera calibration relative to the playfield plane.
#[derive(Debug, Clone, Copy)]
pub struct CabCalibration {
    /// Focal length in pixels (shared for x/y — square pixels assumed).
    pub fx: f32,
    /// Principal point (image centre).
    pub cx: f32,
    pub cy: f32,
    /// Playfield axes expressed in **camera** coordinates, as column vectors:
    /// `[lateral (X), normal (Y, out of playfield), depth (Z, into playfield)]`.
    pub r: [[f32; 3]; 3],
    /// Playfield origin (front-edge centre) in **camera** coordinates, mm.
    pub t: [f32; 3],
}

impl CabCalibration {
    /// Map an image pixel at focal-plane depth (via the pinhole) into the
    /// **playfield frame** (mm), given its distance `z_cam` along the camera
    /// optical axis. `z_cam` comes from the head model (shoulder-width or
    /// depth). Returns `[x_lateral, y_normal, z_depth]`.
    pub fn pixel_to_playfield(&self, u: f32, v: f32, z_cam: f32) -> [f32; 3] {
        // Deproject to camera coordinates.
        let cam = [
            (u - self.cx) * z_cam / self.fx,
            (v - self.cy) * z_cam / self.fx,
            z_cam,
        ];
        // Into the playfield frame: pf = Rᵀ · (cam − t).
        let d = [cam[0] - self.t[0], cam[1] - self.t[1], cam[2] - self.t[2]];
        [
            self.r[0][0] * d[0] + self.r[1][0] * d[1] + self.r[2][0] * d[2],
            self.r[0][1] * d[0] + self.r[1][1] * d[1] + self.r[2][1] * d[2],
            self.r[0][2] * d[0] + self.r[1][2] * d[1] + self.r[2][2] * d[2],
        ]
    }
}
