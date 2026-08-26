//! What the selected sensor needs from USB, and what it actually got.
//!
//! A Kinect that enumerates on the wrong bus speed still works -- it just
//! starves. The v2 in particular needs USB 3.0 and does not say so: it opens,
//! streams, and drops isochronous packets, which surfaces much later as a
//! frame rate nobody can explain. This turns that into one answer at the
//! moment the device is picked.
//!
//! Topology comes from [`nusb`] rather than our vendored libusb: it is pure
//! Rust, needs no device handle (so it never fights the capture thread for the
//! device), and exposes `bus_id` -- which is what "alone on its controller"
//! actually means.

use nusb::{DeviceInfo, MaybeFuture as _, Speed};

/// Microsoft. Every Kinect function of both generations lives under it.
const VID_MICROSOFT: u16 = 0x045e;

/// Kinect v1 functions, from the driver packages we ship: models 1414 and
/// 1473 (camera / motor / audio each enumerate separately) plus the Kinect
/// for Windows variant.
const PIDS_V1: &[u16] = &[0x02ae, 0x02b0, 0x02ad, 0x02bf, 0x02be, 0x02bb, 0x02c2];

/// Kinect v2: Xbox One sensor and the Kinect for Windows one.
const PIDS_V2: &[u16] = &[0x02c4, 0x02d8];

/// How serious the mismatch is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// Everything the sensor needs is there.
    Ok,
    /// Works, but something will bite under load.
    Warn,
    /// The sensor cannot reach its rated throughput like this.
    Bad,
    /// Nothing matching is on the bus.
    Absent,
}

impl Level {
    /// Colour for the dot beside the device. Chosen for the dark cabinet
    /// theme, and paired with a glyph so it does not rely on colour alone.
    #[must_use]
    pub fn colour(self) -> egui::Color32 {
        match self {
            Self::Ok => egui::Color32::from_rgb(0x2e, 0xa0, 0x43),
            Self::Warn => egui::Color32::from_rgb(0xd2, 0x9a, 0x22),
            Self::Bad => egui::Color32::from_rgb(0xc0, 0x39, 0x2b),
            Self::Absent => egui::Color32::GRAY,
        }
    }

    #[must_use]
    pub fn glyph(self) -> &'static str {
        match self {
            Self::Ok => "OK",
            Self::Warn => "!",
            Self::Bad => "X",
            Self::Absent => "-",
        }
    }
}

/// One sensor's USB situation: what it wants, what it has, and why that is or
/// is not a problem.
#[derive(Debug, Clone)]
pub struct UsbReport {
    pub level: Level,
    /// Human-readable requirement, e.g. "USB 3.0 (SuperSpeed), sole device on
    /// its controller".
    pub want: String,
    /// What the bus actually reports.
    pub got: String,
    /// One line per finding, shown on hover.
    pub notes: Vec<String>,
}

/// Which sensor we are asking about.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sensor {
    KinectV1,
    KinectV2,
}

fn speed_name(s: Option<Speed>) -> &'static str {
    match s {
        Some(Speed::Low) => "USB 1.0 (Low, 1.5 Mbit)",
        Some(Speed::Full) => "USB 1.1 (Full, 12 Mbit)",
        Some(Speed::High) => "USB 2.0 (High, 480 Mbit)",
        Some(Speed::Super) => "USB 3.0 (SuperSpeed, 5 Gbit)",
        Some(Speed::SuperPlus) => "USB 3.1 (SuperSpeed+, 10 Gbit)",
        _ => "unknown",
    }
}

fn pids(sensor: Sensor) -> &'static [u16] {
    match sensor {
        Sensor::KinectV1 => PIDS_V1,
        Sensor::KinectV2 => PIDS_V2,
    }
}

/// Inspect the bus and judge `sensor`'s connection.
///
/// Returns `None` only when the platform refuses to enumerate at all; a sensor
/// that is simply not plugged in comes back as [`Level::Absent`], which is a
/// different thing and worth saying differently.
#[must_use]
pub fn check(sensor: Sensor) -> Option<UsbReport> {
    let devices: Vec<DeviceInfo> = nusb::list_devices().wait().ok()?.collect();
    let ours: Vec<&DeviceInfo> = devices
        .iter()
        .filter(|d| d.vendor_id() == VID_MICROSOFT && pids(sensor).contains(&d.product_id()))
        .collect();

    let want = match sensor {
        Sensor::KinectV1 => "USB 2.0 (High Speed)".to_string(),
        Sensor::KinectV2 => "USB 3.0 (SuperSpeed), sole device on its controller".to_string(),
    };

    let Some(first) = ours.first() else {
        return Some(UsbReport {
            level: Level::Absent,
            want,
            got: "not on the bus".to_string(),
            notes: vec![match sensor {
                Sensor::KinectV1 => "No Kinect v1 function found.".to_string(),
                Sensor::KinectV2 => "No Kinect v2 found. It needs its own power adapter -- \
                     the USB cable alone does not power it."
                    .to_string(),
            }],
        });
    };

    let speed = first.speed();
    let mut notes = Vec::new();
    let mut level = Level::Ok;

    // Speed. The v2 streams depth isochronously and simply cannot fit in USB
    // 2.0; the v1 needs High Speed for its video stream.
    let enough = match sensor {
        Sensor::KinectV1 => matches!(speed, Some(Speed::High | Speed::Super | Speed::SuperPlus)),
        Sensor::KinectV2 => matches!(speed, Some(Speed::Super | Speed::SuperPlus)),
    };
    if !enough {
        level = Level::Bad;
        notes.push(format!(
            "Connected at {}. Move it to a port that is {want}.",
            speed_name(speed)
        ));
        if sensor == Sensor::KinectV2 {
            notes.push(
                "A blue port is not proof: front-panel and hub ports often fall \
                 back to USB 2.0. A rear port on the motherboard is the safe one."
                    .to_string(),
            );
        }
    }

    // Company on the controller. Only meaningful for the v2, whose depth
    // stream reserves isochronous bandwidth for the whole controller.
    if sensor == Sensor::KinectV2 {
        let bus = first.bus_id().to_string();
        let ours_addrs: Vec<u8> = ours.iter().map(|d| d.device_address()).collect();
        let strangers: Vec<String> = devices
            .iter()
            .filter(|d| d.bus_id() == bus && !ours_addrs.contains(&d.device_address()))
            // Hubs carry no traffic of their own.
            .filter(|d| d.class() != 0x09)
            .map(|d| {
                d.product_string().map_or_else(
                    || format!("{:04x}:{:04x}", d.vendor_id(), d.product_id()),
                    str::to_string,
                )
            })
            .collect();
        if !strangers.is_empty() {
            if level == Level::Ok {
                level = Level::Warn;
            }
            notes.push(format!(
                "Shares its controller with {}: {}. The depth stream reserves \
                 isochronous bandwidth for the whole controller, so a busy \
                 neighbour costs it packets.",
                strangers.len(),
                strangers.join(", ")
            ));
        }
    }

    if notes.is_empty() {
        notes.push("Everything this sensor needs from USB is there.".to_string());
    }

    Some(UsbReport {
        level,
        want,
        got: speed_name(speed).to_string(),
        notes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_shipped_driver_id_is_covered() {
        // The lists must stay in step with tools/windows-release/setup/drivers:
        // an ID we bind a driver for but do not recognise here would show as
        // "not on the bus" while streaming perfectly well.
        assert_eq!(
            PIDS_V1.len(),
            7,
            "v1: 1414 + 1473 (camera/motor/audio) + KfW"
        );
        assert_eq!(PIDS_V2.len(), 2, "v2: Xbox One + Kinect for Windows");
        for p in PIDS_V1.iter().chain(PIDS_V2) {
            assert!(
                (0x02ad..=0x02d8).contains(p),
                "{p:#06x} outside the Kinect range"
            );
        }
    }

    #[test]
    fn speed_names_say_the_usb_generation() {
        // The generation is what a user acts on; the Mbit figure only backs it up.
        assert!(speed_name(Some(Speed::High)).starts_with("USB 2.0"));
        assert!(speed_name(Some(Speed::Super)).starts_with("USB 3.0"));
        assert_eq!(speed_name(None), "unknown");
    }

    #[test]
    fn a_v1_is_happy_on_high_but_a_v2_is_not() {
        let v1_ok = matches!(
            Some(Speed::High),
            Some(Speed::High | Speed::Super | Speed::SuperPlus)
        );
        let v2_ok = matches!(Some(Speed::High), Some(Speed::Super | Speed::SuperPlus));
        assert!(v1_ok, "the v1 is a USB 2.0 device");
        assert!(!v2_ok, "the v2 cannot fit its depth stream in USB 2.0");
    }
}
