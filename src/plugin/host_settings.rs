//! Read the HOST's own settings the plugin must never re-ask the user for.
//!
//! VPX stores the physical cabinet geometry as `[Player]` settings in
//! `VPinballX.ini` (under `VPXInfo::prefPath`): `LockbarWidth` and
//! `LockbarHeight`, both in **centimeters**, measured on the cabinet
//! (`Settings_properties.inl:620-621`). The lockbar width is the anchor's
//! metric reference; the height (ground → top of lockbar) is a vertical
//! sanity reference for the derived camera pose. VPX's own VR cab model
//! anchors on the lockbar too — same reference frame.
//!
//! There is no plugin API for reading host settings today (candidate
//! upstream patch, tracked in the port plan), so this is a minimal ini
//! scan: find the `[Player]` section, pick the two keys.

use std::path::Path;

use tracing::{info, warn};

/// Cabinet lockbar geometry in millimeters, as configured in VPX.
#[derive(Debug, Clone, Copy)]
pub struct CabGeometry {
    pub lockbar_width_mm: f32,
    pub lockbar_height_mm: f32,
}

impl Default for CabGeometry {
    fn default() -> Self {
        // VPX's own declared defaults (70 cm / 85 cm).
        Self {
            lockbar_width_mm: 700.0,
            lockbar_height_mm: 850.0,
        }
    }
}

/// Read the lockbar geometry from `<pref_path>/VPinballX.ini`. Missing
/// file, section or keys fall back to VPX's declared defaults.
#[must_use]
pub fn read_cab_geometry(pref_path: &Path) -> CabGeometry {
    let path = pref_path.join("VPinballX.ini");
    let Ok(text) = std::fs::read_to_string(&path) else {
        warn!(
            ?path,
            "VPinballX.ini not readable; using VPX default lockbar geometry"
        );
        return CabGeometry::default();
    };
    let mut geo = CabGeometry::default();
    let mut in_player = false;
    for line in text.lines() {
        let line = line.trim();
        if let Some(section) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
            in_player = section.eq_ignore_ascii_case("Player");
            continue;
        }
        if !in_player || line.starts_with(';') {
            continue;
        }
        let Some((key, value)) = line.split_once('=') else {
            continue;
        };
        let (key, value) = (key.trim(), value.trim());
        let parse_cm_to_mm = |v: &str| v.parse::<f32>().ok().map(|cm| cm * 10.0);
        if key.eq_ignore_ascii_case("LockbarWidth") {
            if let Some(mm) = parse_cm_to_mm(value) {
                geo.lockbar_width_mm = mm;
            }
        } else if key.eq_ignore_ascii_case("LockbarHeight")
            && let Some(mm) = parse_cm_to_mm(value)
        {
            geo.lockbar_height_mm = mm;
        }
    }
    info!(
        lockbar_width_mm = geo.lockbar_width_mm,
        lockbar_height_mm = geo.lockbar_height_mm,
        "cab geometry read from VPX host settings"
    );
    geo
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_player_section_and_converts_cm_to_mm() {
        let dir = std::env::temp_dir().join(format!("ht-ini-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            dir.join("VPinballX.ini"),
            "[Standalone]\nLockbarWidth = 999\n[Player]\n; comment\nLockbarWidth = 61.0\nLockbarHeight = 77.5\n[Editor]\n",
        )
        .unwrap();
        let geo = read_cab_geometry(&dir);
        assert!((geo.lockbar_width_mm - 610.0).abs() < f32::EPSILON);
        assert!((geo.lockbar_height_mm - 775.0).abs() < f32::EPSILON);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn missing_file_yields_vpx_defaults() {
        let geo = read_cab_geometry(Path::new("/nonexistent-ht-test"));
        assert!((geo.lockbar_width_mm - 700.0).abs() < f32::EPSILON);
    }
}
