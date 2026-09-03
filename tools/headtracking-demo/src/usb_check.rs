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
    ///
    /// There is deliberately no middle rung any more. The only amber case was
    /// "something else shares this controller", which on a board with a single
    /// controller is permanent and harmless — the bus tree in the USB window
    /// answers that question far better than a colour did.
    Ok,
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
            Self::Bad => egui::Color32::from_rgb(0xc0, 0x39, 0x2b),
            Self::Absent => egui::Color32::GRAY,
        }
    }

    #[must_use]
    pub fn glyph(self) -> &'static str {
        match self {
            Self::Ok => "OK",
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
        // Deliberately NOT a warning any more. The v2 peaks around 2 Gbit/s
        // of the 5 a SuperSpeed controller provides, so a keyboard, a mouse
        // and a cabinet I/O board alongside it are a non-event. Flagging them
        // amber told an owner whose motherboard has a single controller that
        // his hardware was wrong when it was fine, and sent him hunting for a
        // second controller that does not exist on the board (field report,
        // ASUS Z790-P). What actually competes is another *camera*: two
        // isochronous video streams on one controller is the case worth
        // knowing about, and the tree in the USB window shows it far better
        // than a colour ever could.
        if !strangers.is_empty() {
            notes.push(format!(
                "Shares its controller with {} other device(s): {}. Normally \
                 harmless -- the v2 peaks near 2 Gbit/s of the 5 this \
                 controller carries. Another camera on the same controller is \
                 the one combination to avoid.",
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

/// Log the bus context once at startup.
///
/// Deliberately narrow: the Kinect's own connection, and how many other
/// devices share its controller -- **counted, never named**. A full
/// enumeration would list every keyboard, phone and licence dongle on the
/// machine, and this log is meant to be pasteable into a bug report and to
/// travel with a contribution. The count is what diagnoses contention; the
/// names only identify the person.
pub fn log_startup() {
    let Ok(devices) = nusb::list_devices().wait() else {
        tracing::info!(target: "usb", "bus enumeration unavailable on this platform");
        return;
    };
    let devices: Vec<DeviceInfo> = devices.collect();
    for (sensor, name) in [
        (Sensor::KinectV1, "kinect-v1"),
        (Sensor::KinectV2, "kinect-v2"),
    ] {
        let ours: Vec<&DeviceInfo> = devices
            .iter()
            .filter(|d| d.vendor_id() == VID_MICROSOFT && pids(sensor).contains(&d.product_id()))
            .collect();
        let Some(first) = ours.first() else { continue };
        let bus = first.bus_id();
        let neighbours = devices
            .iter()
            .filter(|d| d.bus_id() == bus && d.class() != 0x09)
            .count()
            .saturating_sub(ours.len());
        tracing::info!(
            target: "usb",
            sensor = name,
            functions = ours.len(),
            speed = speed_name(first.speed()),
            port = ?first.port_chain(),
            neighbours_on_controller = neighbours,
            "usb context"
        );
    }
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

/// One line of the bus tree: how deep it sits, what it is, and whether it is
/// the sensor we care about.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BusNode {
    /// 0 = the controller itself, 1 = plugged straight into it, 2+ = behind
    /// a hub.
    pub depth: usize,
    /// `USB 3.0`, `USB 2.0`, … — first on the line and fixed width on
    /// purpose. Someone opening this window because the Kinect lags is
    /// looking for exactly one thing, and it should be readable down a
    /// column rather than hunted for at the end of a sentence.
    pub generation: String,
    /// Device name, port, and the detailed rate.
    pub label: String,
    /// The sensor being asked about, so the UI can pick it out of the list.
    pub is_sensor: bool,
    /// The sensor is plugged in below the speed it needs — the single fact
    /// this whole window exists to make obvious.
    pub sensor_underspeed: bool,
}

/// Just the generation, for the column: `USB 2.0` rather than
/// `USB 2.0 (High, 480 Mbit)`.
fn generation(s: Option<Speed>) -> &'static str {
    match s {
        Some(Speed::Low) => "USB 1.0",
        Some(Speed::Full) => "USB 1.1",
        Some(Speed::High) => "USB 2.0",
        Some(Speed::Super) => "USB 3.0",
        Some(Speed::SuperPlus) => "USB 3.1",
        _ => "USB ?",
    }
}

/// The detailed rate, without repeating the generation already in its own
/// column: `High, 480 Mbit`.
fn rate_detail(s: Option<Speed>) -> &'static str {
    match s {
        Some(Speed::Low) => "Low, 1.5 Mbit",
        Some(Speed::Full) => "Full, 12 Mbit",
        Some(Speed::High) => "High, 480 Mbit",
        Some(Speed::Super) => "SuperSpeed, 5 Gbit",
        Some(Speed::SuperPlus) => "SuperSpeed+, 10 Gbit",
        _ => "unknown rate",
    }
}

/// What this sensor needs to work at all.
fn fast_enough(sensor: Sensor, s: Option<Speed>) -> bool {
    match sensor {
        Sensor::KinectV1 => matches!(s, Some(Speed::High | Speed::Super | Speed::SuperPlus)),
        Sensor::KinectV2 => matches!(s, Some(Speed::Super | Speed::SuperPlus)),
    }
}

/// Collect what is on the bus, as `lsusb -t` would draw it.
///
/// Built from `nusb` rather than shelling out to `lsusb`: that command does
/// not exist on Windows, where most of the cabinets are, and the port chain
/// nusb reports is the same information it prints.
///
/// This is for the screen, never for the log. `log_startup` counts a sensor's
/// neighbours without naming them, because that log travels with a
/// contribution; here the names are the entire point and nothing leaves the
/// machine.
#[must_use]
pub fn topology(sensor: Sensor) -> Vec<BusNode> {
    let Ok(devices) = nusb::list_devices().wait() else {
        return Vec::new();
    };
    let wanted = pids(sensor);
    let mut rows: Vec<Row> = devices
        .map(|d| {
            let name = d.product_string().map_or_else(
                || format!("{:04x}:{:04x}", d.vendor_id(), d.product_id()),
                str::to_string,
            );
            let hub = if d.class() == 0x09 { " (hub)" } else { "" };
            let is_sensor = d.vendor_id() == VID_MICROSOFT && wanted.contains(&d.product_id());
            let speed = d.speed();
            let port = d
                .port_chain()
                .last()
                .map_or_else(|| "?".to_string(), u8::to_string);
            Row {
                bus: d.bus_id().to_string(),
                chain: d.port_chain().to_vec(),
                generation: generation(speed).to_string(),
                label: format!("{name}{hub}  ·  port {port}, {}", rate_detail(speed)),
                is_sensor,
                sensor_underspeed: is_sensor && !fast_enough(sensor, speed),
            }
        })
        .collect();
    rows.sort_by(|a, b| a.bus.cmp(&b.bus).then_with(|| a.chain.cmp(&b.chain)));
    render_tree(&rows)
}

/// A device as collected, before it is arranged into a tree.
struct Row {
    bus: String,
    chain: Vec<u8>,
    generation: String,
    label: String,
    is_sensor: bool,
    sensor_underspeed: bool,
}

/// Arrange sorted rows into indented lines, one controller heading per bus.
/// Split out from [`topology`] so the shape can be tested without a USB bus.
fn render_tree(rows: &[Row]) -> Vec<BusNode> {
    let mut out = Vec::new();
    let mut bus = None;
    for row in rows {
        if bus.as_ref() != Some(&row.bus) {
            out.push(BusNode {
                depth: 0,
                generation: String::new(),
                label: format!("Controller {}", row.bus),
                is_sensor: false,
                sensor_underspeed: false,
            });
            bus = Some(row.bus.clone());
        }
        out.push(BusNode {
            // A device plugged into the controller has a one-element chain and
            // sits at depth 1; every hub in between adds one.
            depth: row.chain.len().max(1),
            generation: row.generation.clone(),
            label: row.label.clone(),
            is_sensor: row.is_sensor,
            sensor_underspeed: row.sensor_underspeed,
        });
    }
    out
}

/// How many controllers and devices the tree holds.
///
/// Worth stating outright, because "only one controller?" on a brand-new
/// USB 3.2 board reads as a broken enumeration rather than as the normal
/// thing it is — it cost an afternoon of doubt on a real machine before
/// anyone thought to check whether the count was even wrong.
#[must_use]
pub fn counts(tree: &[BusNode]) -> (usize, usize) {
    let controllers = tree.iter().filter(|n| n.depth == 0).count();
    (controllers, tree.len() - controllers)
}

#[cfg(test)]
mod topology_tests {
    use super::{Row, Sensor, fast_enough, generation, render_tree};
    use nusb::Speed;

    fn row(bus: &str, chain: &[u8], name: &str, speed: Option<Speed>, sensor: bool) -> Row {
        Row {
            bus: bus.into(),
            chain: chain.to_vec(),
            generation: generation(speed).into(),
            label: name.into(),
            is_sensor: sensor,
            sensor_underspeed: sensor && !fast_enough(Sensor::KinectV2, speed),
        }
    }

    /// Depth has to follow the port chain, because that is the whole point of
    /// the window: someone who cannot see that two devices hang off the same
    /// controller cannot act on being told they do.
    #[test]
    fn devices_nest_under_their_controller_and_their_hub() {
        let rows = vec![
            row("usb1", &[1], "Kinect v2", Some(Speed::Super), true),
            row("usb1", &[3], "Hub (hub)", Some(Speed::High), false),
            row("usb1", &[3, 2], "Keyboard", Some(Speed::Full), false),
            row("usb2", &[1], "Webcam", Some(Speed::High), false),
        ];
        let tree = render_tree(&rows);
        let shape: Vec<(usize, bool)> = tree.iter().map(|n| (n.depth, n.is_sensor)).collect();
        assert_eq!(
            shape,
            [
                (0, false), // Controller usb1
                (1, true),  // Kinect, straight into it
                (1, false), // the hub
                (2, false), // behind the hub
                (0, false), // Controller usb2
                (1, false), // the webcam
            ]
        );
        assert!(tree[0].label.starts_with("Controller usb1"));
        assert!(tree[4].label.starts_with("Controller usb2"));
    }

    /// Two cameras on one controller is the only combination worth acting on,
    /// so the tree must make them visibly siblings.
    #[test]
    fn two_cameras_on_one_controller_read_as_siblings() {
        let rows = vec![
            row("usb1", &[1], "Kinect v2", Some(Speed::Super), true),
            row("usb1", &[2], "Webcam", Some(Speed::High), false),
        ];
        let tree = render_tree(&rows);
        assert_eq!(tree.len(), 3, "one heading, two devices");
        assert_eq!(tree[1].depth, tree[2].depth, "siblings share a depth");
        assert_eq!(tree[0].depth, 0);
    }

    /// The reason someone opens this window: a v2 on a USB 2 port. The
    /// generation must be readable on its own, and the sensor flagged —
    /// buried in prose at the end of a line it is exactly what gets missed.
    #[test]
    fn a_v2_on_a_usb2_port_says_so_in_its_own_column() {
        let tree = render_tree(&[row("usb1", &[1], "Kinect v2", Some(Speed::High), true)]);
        let kinect = &tree[1];
        assert_eq!(kinect.generation, "USB 2.0");
        assert!(
            kinect.sensor_underspeed,
            "a v2 below SuperSpeed has to be flagged"
        );

        let ok = render_tree(&[row("usb1", &[1], "Kinect v2", Some(Speed::Super), true)]);
        assert_eq!(ok[1].generation, "USB 3.0");
        assert!(!ok[1].sensor_underspeed);
    }

    /// The count has to separate headings from devices, since a heading is
    /// not something anyone plugged in.
    #[test]
    fn counting_tells_controllers_from_devices() {
        let tree = super::render_tree(&[
            row("usb1", &[1], "Kinect v2", Some(Speed::Super), true),
            row("usb1", &[2], "Keyboard", Some(Speed::Full), false),
            row("usb2", &[1], "Webcam", Some(Speed::High), false),
        ]);
        assert_eq!(super::counts(&tree), (2, 3));

        // The case that caused the head-scratching: one controller, several
        // devices, and nothing wrong.
        let one = super::render_tree(&[
            row("usb1", &[1], "Kinect v2", Some(Speed::Super), true),
            row("usb1", &[2], "Keyboard", Some(Speed::Full), false),
        ]);
        assert_eq!(super::counts(&one), (1, 2));

        assert_eq!(super::counts(&[]), (0, 0), "an empty bus counts as nothing");
    }

    /// A v1 is happy on High Speed where a v2 is not — the flag has to follow
    /// the sensor, not a fixed threshold.
    #[test]
    fn the_speed_a_sensor_needs_depends_on_the_sensor() {
        assert!(fast_enough(Sensor::KinectV1, Some(Speed::High)));
        assert!(!fast_enough(Sensor::KinectV2, Some(Speed::High)));
        assert!(fast_enough(Sensor::KinectV2, Some(Speed::Super)));
        assert!(!fast_enough(Sensor::KinectV1, Some(Speed::Full)));
    }
}
