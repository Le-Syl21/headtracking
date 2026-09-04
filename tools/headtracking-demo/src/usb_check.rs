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
        Sensor::KinectV2 => "USB 3.0 (SuperSpeed), no other camera on its bus".to_string(),
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

    // Company on the bus. Only meaningful for the v2, whose depth stream
    // reserves isochronous bandwidth out of that bus's budget.
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
                 harmless: isochronous bandwidth is reserved per streaming \
                 endpoint, and a keyboard or an I/O board reserves next to \
                 nothing. Another camera is the one neighbour that matters -- \
                 two video reservations that do not fit means the second \
                 stream is refused outright, not slowed.",
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
    /// `480M`, `5000M` — the link rate, right-aligned in its own column so a
    /// glance down the list compares like with like. On a bus heading it is
    /// the bus's own rate, from the root hub.
    ///
    /// Deliberately *not* a share of the bus. What enumeration gives is the
    /// negotiated link rate, not consumption: five 12M devices on a 480M bus
    /// would read as 12% while sending a few bytes every 8 ms, and a Kinect
    /// alone on a SuperSpeed bus would read as 100% while reserving about
    /// two fifths of it. A percentage built from these numbers would cry
    /// wolf on the one healthy case that matters.
    pub rate: String,
    /// Device name, or `Bus <id>` on a heading. Nothing else — the rate is
    /// its own column and the arrow carries the nesting.
    pub label: String,
    /// The sensor being asked about, so the UI can pick it out of the list.
    pub is_sensor: bool,
    /// The sensor is plugged in below the speed it needs — the single fact
    /// this whole window exists to make obvious. Set on the device and on
    /// the bus heading that carries it.
    pub sensor_underspeed: bool,
    /// What this device reserves while it streams, in Mbit/s — see
    /// [`demand_mbit`]. Zero for the great majority, which reserve nothing
    /// worth counting or could not be identified.
    ///
    /// On a bus heading this is the total of its devices, and
    /// [`BusNode::budget_mbit`] is what that total has to fit inside.
    pub demand_mbit: u32,
    /// On a bus heading: what periodic transfers may claim here — 80% of a
    /// USB 2.0 bus, 90% of a USB 3.x one. Zero on a device line.
    pub budget_mbit: u32,
}

impl BusNode {
    /// The claims on this bus exceed what periodic transfers may have. The
    /// case this is for is two Kinect v2 on one bus: 2160 Mbit/s each against
    /// a 4500 Mbit/s ceiling, so the second one's stream is refused rather
    /// than slowed.
    #[must_use]
    pub fn over_budget(&self) -> bool {
        self.depth == 0 && self.budget_mbit > 0 && self.demand_mbit > self.budget_mbit
    }

    /// Within budget, but with less than a quarter of it left — close enough
    /// that one more camera would not fit.
    #[must_use]
    pub fn tight(&self) -> bool {
        self.depth == 0
            && self.budget_mbit > 0
            && !self.over_budget()
            && self.demand_mbit * 4 >= self.budget_mbit * 3
    }
}

/// Link rate as `lsusb --tree` prints it: `480M`, `5000M`. Short on purpose —
/// it is a column, not a sentence.
fn rate(s: Option<Speed>) -> &'static str {
    match s {
        Some(Speed::Low) => "1.5M",
        Some(Speed::Full) => "12M",
        Some(Speed::High) => "480M",
        Some(Speed::Super) => "5000M",
        Some(Speed::SuperPlus) => "10000M",
        _ => "?",
    }
}

/// Kinect v2 camera function — the one that streams. The firmware-update PID
/// enumerates without streaming anything.
const PIDS_V2_STREAMING: &[u16] = &[0x02c4];

/// The Kinect v1 camera functions. Motor and audio enumerate separately and
/// carry nothing like this load.
const PIDS_V1_STREAMING: &[u16] = &[0x02ae, 0x02bf, 0x02be];

/// What a device reserves on its bus while it streams, in Mbit/s.
///
/// Isochronous bandwidth is claimed up front, per bus, and the claim is what
/// has to fit. These figures are the claims themselves, taken from what the
/// drivers demand rather than measured or guessed:
///
/// * **Kinect v2** - libfreenect2 refuses to start unless endpoint 0x84 offers
///   `0x8400` (33792) bytes per isochronous packet. A SuperSpeed service
///   interval is 125 us, so that is 33792 / 125 us = 270 MB/s = **2160
///   Mbit/s**, or 43% of a 5 Gbit/s bus. Two come to 87% against a 90% ceiling
///   for periodic transfers - which is why two on one bus is the case worth
///   colouring, and why three cannot start at all.
/// * **Kinect v1** - libfreenect asks for `DEPTH_PKTSIZE` 1760 and
///   `VIDEO_PKTSIZE` 1920 bytes per 125 us microframe: 235 Mbit/s together,
///   49% of a 480 Mbit/s bus against an 80% ceiling. That is the arithmetic
///   behind the old advice to give a v1 its own controller.
/// * **A video-class device** - a UVC camera can ask for uncompressed
///   1920x1080 at 60 fps, which at two bytes a pixel is 1990 Mbit/s. Capped by
///   what its own link carries, since a 12 Mbit device cannot demand more.
/// * **An audio-class device** - eight channels of 24-bit 48 kHz is under 10
///   Mbit/s. Counted because it is not zero, but it never decides anything.
///
/// Zero for anything unidentified, which makes the total an
/// **under**-estimate and never an over-estimate. That is the right direction
/// for the error: this must not cry wolf, which is precisely what the warning
/// it replaces did.
fn demand_mbit(d: &DeviceInfo) -> u32 {
    let link_mbit = match d.speed() {
        Some(Speed::Low) => 1,
        Some(Speed::Full) => 12,
        Some(Speed::High) => 480,
        Some(Speed::Super) => 5000,
        Some(Speed::SuperPlus) => 10000,
        _ => 0,
    };
    if d.vendor_id() == VID_MICROSOFT {
        if PIDS_V2_STREAMING.contains(&d.product_id()) {
            return 2160;
        }
        if PIDS_V1_STREAMING.contains(&d.product_id()) {
            return 235;
        }
    }
    let classes: Vec<u8> = d.interfaces().map(|i| i.class()).collect();
    if classes.contains(&0x0e) {
        return 1990.min(link_mbit);
    }
    if classes.contains(&0x01) {
        return 10.min(link_mbit);
    }
    0
}

/// How much of a bus periodic transfers may claim, in Mbit/s.
///
/// USB 2.0 caps them at 80% of the frame and USB 3.x at 90%; the rest is left
/// for bulk and control, which is why a bus fills up well before its nominal
/// rate.
fn periodic_budget_mbit(bus_speed: Option<Speed>) -> u32 {
    match bus_speed {
        Some(Speed::Low) => 1,
        Some(Speed::Full) => 12 * 80 / 100,
        Some(Speed::High) => 480 * 80 / 100,
        Some(Speed::Super) => 5000 * 90 / 100,
        Some(Speed::SuperPlus) => 10000 * 90 / 100,
        _ => 0,
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
    let Ok(buses) = nusb::list_buses().wait() else {
        return Vec::new();
    };
    let Ok(devices) = nusb::list_devices().wait() else {
        return Vec::new();
    };
    let devices: Vec<DeviceInfo> = devices.collect();
    let wanted = pids(sensor);

    let mut buses: Vec<(String, Option<Speed>)> = buses
        .map(|b| (b.bus_id().to_string(), b.root_hub().speed()))
        .collect();
    buses.sort_by(|a, b| a.0.cmp(&b.0));

    let mut out = Vec::new();
    for (bus_id, bus_speed) in buses {
        let mut rows: Vec<Row> = devices
            .iter()
            .filter(|d| d.bus_id() == bus_id)
            .map(|d| {
                let speed = d.speed();
                let is_sensor = d.vendor_id() == VID_MICROSOFT && wanted.contains(&d.product_id());
                let name = d.product_string().map_or_else(
                    || format!("{:04x}:{:04x}", d.vendor_id(), d.product_id()),
                    str::to_string,
                );
                let hub = if d.class() == 0x09 { " (hub)" } else { "" };
                let port = d
                    .port_chain()
                    .last()
                    .map_or_else(|| "?".to_string(), u8::to_string);
                Row {
                    chain: d.port_chain().to_vec(),
                    rate: rate(speed).to_string(),
                    label: format!("port {port}  {name}{hub}"),
                    is_sensor,
                    sensor_underspeed: is_sensor && !fast_enough(sensor, speed),
                    demand_mbit: demand_mbit(d),
                }
            })
            .collect();
        rows.sort_by(|a, b| a.chain.cmp(&b.chain));
        out.extend(bus_block(&bus_id, bus_speed, rows));
    }
    out
}

/// A device as collected, before it joins its bus.
struct Row {
    chain: Vec<u8>,
    rate: String,
    label: String,
    is_sensor: bool,
    sensor_underspeed: bool,
    demand_mbit: u32,
}

/// One bus heading followed by its devices. Split out from [`topology`] so the
/// shape can be tested without a USB bus.
fn bus_block(bus_id: &str, bus_speed: Option<Speed>, rows: Vec<Row>) -> Vec<BusNode> {
    // An empty bus is kept on purpose: a free SuperSpeed socket is exactly
    // what someone whose sensor sits on a 480M bus needs to be told about.
    let mut out = vec![BusNode {
        depth: 0,
        rate: rate(bus_speed).to_string(),
        label: format!("Bus {bus_id}"),
        is_sensor: false,
        // A heading is flagged when it is the bus starving the sensor.
        sensor_underspeed: rows.iter().any(|r| r.sensor_underspeed),
        demand_mbit: rows.iter().map(|r| r.demand_mbit).sum(),
        budget_mbit: periodic_budget_mbit(bus_speed),
    }];
    out.extend(rows.into_iter().map(|row| BusNode {
        depth: row.chain.len().max(1),
        rate: row.rate,
        label: row.label,
        is_sensor: row.is_sensor,
        sensor_underspeed: row.sensor_underspeed,
        demand_mbit: row.demand_mbit,
        budget_mbit: 0,
    }));
    out
}

#[must_use]
pub fn counts(tree: &[BusNode]) -> (usize, usize) {
    let buses = tree.iter().filter(|n| n.depth == 0).count();
    (buses, tree.len() - buses)
}

#[cfg(test)]
mod topology_tests {
    use super::{BusNode, Row, Sensor, bus_block, fast_enough, periodic_budget_mbit, rate};
    use nusb::Speed;

    fn row(chain: &[u8], name: &str, speed: Option<Speed>, sensor: bool, demand: u32) -> Row {
        Row {
            chain: chain.to_vec(),
            rate: rate(speed).into(),
            label: name.into(),
            is_sensor: sensor,
            sensor_underspeed: sensor && !fast_enough(Sensor::KinectV2, speed),
            demand_mbit: demand,
        }
    }

    fn tree(bus: &str, speed: Option<Speed>, rows: Vec<Row>) -> Vec<BusNode> {
        bus_block(bus, speed, rows)
    }

    /// The heading carries the bus's own rate, not a device's — the number
    /// that says whether a SuperSpeed sensor is on the wrong bus.
    #[test]
    fn a_bus_heading_carries_the_buss_own_rate() {
        let t = tree(
            "usb1",
            Some(Speed::High),
            vec![row(&[5], "Keyboard", Some(Speed::Full), false, 0)],
        );
        assert_eq!(t[0].label, "Bus usb1");
        assert_eq!(t[0].rate, "480M", "the bus, not the 12M keyboard on it");
        assert_eq!(t[1].rate, "12M");
        assert_eq!((t[0].depth, t[1].depth), (0, 1));
    }

    /// An empty bus stays: a free SuperSpeed socket is exactly what someone
    /// whose sensor sits on a 480M bus needs to see.
    #[test]
    fn an_empty_bus_is_still_worth_showing() {
        let t = tree("usb2", Some(Speed::Super), Vec::new());
        assert_eq!(t.len(), 1);
        assert_eq!(t[0].rate, "5000M");
        assert!(!t[0].over_budget(), "nothing claimed, nothing over");
    }

    /// The case this arithmetic exists for. One v2 claims 2160 of the 4500
    /// Mbit/s periodic budget of a SuperSpeed bus; two claim 4320, which
    /// still fits but leaves under a quarter; three do not fit at all.
    #[test]
    fn two_kinect_v2_on_one_bus_is_tight_and_three_do_not_fit() {
        let v2 = |port: u8| row(&[port], "Kinect v2", Some(Speed::Super), false, 2160);

        let one = tree("usb1", Some(Speed::Super), vec![v2(1)]);
        assert!(!one[0].over_budget() && !one[0].tight(), "2160 of 4500");

        let two = tree("usb1", Some(Speed::Super), vec![v2(1), v2(2)]);
        assert_eq!(two[0].demand_mbit, 4320);
        assert_eq!(two[0].budget_mbit, 4500);
        assert!(!two[0].over_budget(), "4320 still fits inside 4500");
        assert!(two[0].tight(), "but with 180 Mbit to spare");

        let three = tree("usb1", Some(Speed::Super), vec![v2(1), v2(2), v2(3)]);
        assert!(three[0].over_budget(), "6480 cannot fit 4500");
    }

    /// A Kinect v1 takes half a USB 2.0 bus, which is the arithmetic behind
    /// the old advice to give it a controller of its own.
    #[test]
    fn two_kinect_v1_do_not_fit_on_one_usb2_bus() {
        let v1 = |port: u8| row(&[port], "Kinect v1", Some(Speed::High), false, 235);
        assert_eq!(periodic_budget_mbit(Some(Speed::High)), 384, "80% of 480");

        let one = tree("usb1", Some(Speed::High), vec![v1(1)]);
        assert!(!one[0].over_budget() && !one[0].tight());

        let two = tree("usb1", Some(Speed::High), vec![v1(1), v1(2)]);
        assert_eq!(two[0].demand_mbit, 470);
        assert!(two[0].over_budget(), "470 does not fit inside 384");
        assert!(
            !two[0].tight(),
            "over budget is not 'nearly full' - the two states are exclusive"
        );
    }

    /// A v2 beside an uncompressed FullHD60 webcam does not fit either — the
    /// pairing the window warns about in words.
    #[test]
    fn a_v2_and_a_full_hd_webcam_do_not_fit_together() {
        let t = tree(
            "usb1",
            Some(Speed::Super),
            vec![
                row(&[1], "Kinect v2", Some(Speed::Super), true, 2160),
                row(&[2], "Webcam", Some(Speed::Super), false, 1990),
            ],
        );
        assert_eq!(t[0].demand_mbit, 4150);
        assert!(!t[0].over_budget(), "4150 fits 4500, but only just");
        assert!(t[0].tight());
    }

    /// Ordinary devices claim nothing worth counting, so a busy-looking bus
    /// of keyboards stays quiet. Crying wolf here is what this replaced.
    #[test]
    fn a_bus_full_of_keyboards_claims_nothing() {
        let t = tree(
            "usb1",
            Some(Speed::High),
            vec![
                row(&[1], "Keyboard", Some(Speed::Full), false, 0),
                row(&[2], "Mouse", Some(Speed::Full), false, 0),
                row(&[3], "Bluetooth", Some(Speed::Full), false, 0),
                row(&[4], "USB DAC", Some(Speed::Full), false, 10),
            ],
        );
        assert_eq!(t[0].demand_mbit, 10, "only the audio device counts");
        assert!(!t[0].over_budget() && !t[0].tight());
    }

    /// A v2 on a 480M bus flags itself and the bus starving it.
    #[test]
    fn an_underspeed_sensor_flags_itself_and_its_bus() {
        let t = tree(
            "usb1",
            Some(Speed::High),
            vec![row(&[1], "Kinect v2", Some(Speed::High), true, 2160)],
        );
        assert!(t[1].sensor_underspeed, "the device");
        assert!(t[0].sensor_underspeed, "and the bus carrying it");
    }

    /// Depth follows the port chain, so a device behind a hub reads as its
    /// child rather than the bus's.
    #[test]
    fn a_device_behind_a_hub_nests_under_it() {
        let t = tree(
            "usb1",
            Some(Speed::High),
            vec![
                row(&[3], "Hub (hub)", Some(Speed::High), false, 0),
                row(&[3, 2], "Keyboard", Some(Speed::Full), false, 0),
            ],
        );
        assert_eq!((t[1].depth, t[2].depth), (1, 2));
    }

    /// The budget is a share of the bus, not the whole of it: periodic
    /// transfers get 80% of USB 2.0 and 90% of USB 3.x, and the rest is left
    /// for bulk and control.
    #[test]
    fn the_periodic_budget_is_a_share_not_the_whole_bus() {
        assert_eq!(periodic_budget_mbit(Some(Speed::High)), 384);
        assert_eq!(periodic_budget_mbit(Some(Speed::Super)), 4500);
        assert_eq!(periodic_budget_mbit(Some(Speed::SuperPlus)), 9000);
        assert_eq!(periodic_budget_mbit(None), 0, "unknown claims nothing");
    }

    /// A v1 is happy on High Speed where a v2 is not.
    #[test]
    fn the_speed_a_sensor_needs_depends_on_the_sensor() {
        assert!(fast_enough(Sensor::KinectV1, Some(Speed::High)));
        assert!(!fast_enough(Sensor::KinectV2, Some(Speed::High)));
        assert!(fast_enough(Sensor::KinectV2, Some(Speed::Super)));
    }

    /// The rate column is the short form lsusb uses, because it is a column.
    #[test]
    fn rates_read_as_lsusb_prints_them() {
        assert_eq!(rate(Some(Speed::Full)), "12M");
        assert_eq!(rate(Some(Speed::High)), "480M");
        assert_eq!(rate(Some(Speed::Super)), "5000M");
        assert_eq!(rate(None), "?");
    }

    /// Headings and devices are counted apart.
    #[test]
    fn counting_tells_buses_from_devices() {
        let mut t = tree(
            "usb1",
            Some(Speed::High),
            vec![row(&[1], "A", Some(Speed::Full), false, 0)],
        );
        t.extend(tree("usb2", Some(Speed::Super), Vec::new()));
        assert_eq!(super::counts(&t), (2, 1));
    }
}
