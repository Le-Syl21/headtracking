//! `ht-debug`: standalone Kinect viewer for the head-tracker pipeline.
//!
//! Detects connected Kinect v1 / v2 sensors and exposes a dropdown to pick
//! the active input. The center pane shows the live RGB feed with a
//! crosshair on the detected head; the bottom panel splits into a tracing
//! log on the left and the VPX-style view delta on the right.
//!
//! Run with `cargo run --release -p ht-debug`.

use std::collections::VecDeque;
use std::io::{self, Write};
use std::sync::Arc;
use std::time::{Duration, Instant};

use eframe::egui::{
    self, Align, CentralPanel, Color32, ColorImage, ComboBox, Layout, Pos2, Rect, RichText,
    ScrollArea, Sense, Stroke, TextureHandle, TopBottomPanel, Vec2,
};
use parking_lot::Mutex;
use tracing::{error, info, warn};
use tracing_subscriber::fmt::MakeWriter;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;

const DEPTH_MIN_MM: f32 = 500.0;
const DEPTH_MAX_MM: f32 = 2_500.0;
const WINDOW_HALF: i32 = 25;
const MIN_VALID_PIXELS: u32 = 100;

const LOG_BUFFER_LINES: usize = 1_000;

fn main() -> eframe::Result {
    let logs: Arc<Mutex<VecDeque<String>>> =
        Arc::new(Mutex::new(VecDeque::with_capacity(LOG_BUFFER_LINES)));
    init_tracing(Arc::clone(&logs));

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 800.0])
            .with_title("ht-debug"),
        ..Default::default()
    };

    eframe::run_native(
        "ht-debug",
        options,
        Box::new(move |_cc| Ok(Box::new(App::new(logs)))),
    )
}

// ============================================================ Backend dropdown

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Backend {
    None,
    KinectV1,
    KinectV2,
    /// Index in the enumerated webcam list.
    Webcam(u32),
}

#[derive(Debug, Clone)]
struct BackendEntry {
    backend: Backend,
    label: String,
}

/// Probe USB for connected sensors. Always returns `None (off)` first; the
/// other entries are added when the corresponding library reports a device.
fn detect_backends() -> Vec<BackendEntry> {
    let mut out = vec![BackendEntry {
        backend: Backend::None,
        label: "None (off)".to_string(),
    }];

    match freenect2::Context::new() {
        Ok(ctx) => {
            let n = ctx.enumerate();
            if n > 0 {
                out.push(BackendEntry {
                    backend: Backend::KinectV2,
                    label: "Kinect v2".to_string(),
                });
                info!(count = n, "kinect v2 detected");
            }
        }
        Err(e) => info!(?e, "kinect v2 enumerate failed"),
    }

    match freenect::Context::new() {
        Ok(ctx) => {
            let n = ctx.enumerate();
            if n > 0 {
                out.push(BackendEntry {
                    backend: Backend::KinectV1,
                    label: "Kinect v1".to_string(),
                });
                info!(count = n, "kinect v1 detected");
            }
        }
        Err(e) => info!(?e, "kinect v1 enumerate failed"),
    }

    match webcam::list() {
        Ok(cams) => {
            for cam in cams {
                let label = if cam.name.is_empty() {
                    format!("Webcam #{}", cam.id)
                } else {
                    format!("Webcam: {}", cam.name)
                };
                info!(index = cam.id, name = %cam.name, "webcam detected");
                out.push(BackendEntry {
                    backend: Backend::Webcam(cam.id),
                    label,
                });
            }
        }
        Err(e) => info!(?e, "webcam enumerate failed"),
    }

    out
}

// ============================================================ App state

struct App {
    selected: Backend,
    available: Vec<BackendEntry>,
    active: Option<Active>,
    error: Option<String>,
    logs: Arc<Mutex<VecDeque<String>>>,
}

impl App {
    fn label_for(&self, backend: Backend) -> String {
        self.available
            .iter()
            .find(|e| e.backend == backend)
            .map(|e| e.label.clone())
            .unwrap_or_else(|| match backend {
                Backend::None => "None (off)".to_string(),
                Backend::KinectV1 => "Kinect v1".to_string(),
                Backend::KinectV2 => "Kinect v2".to_string(),
                Backend::Webcam(i) => format!("Webcam #{i}"),
            })
    }
}

struct Active {
    backend: Backend,
    intrinsics: Intrinsics,
    rgb_texture: Option<TextureHandle>,
    /// 1€-smoothed head pose. Same shape as the raw `HeadPixel` but the
    /// position values come out of [`OneEuroPose3D`] so the crosshair and
    /// the VPX delta panel stop jittering at the pixel level.
    last_head: Option<HeadPixel>,
    baseline: Option<Baseline>,
    inner: Inner,
    /// `Some` only when [`Inner::KinectV1`] — drives the motorised base.
    v1_controls: Option<V1Controls>,
    pose_filter: filter_alias::OneEuroPose3D,
    started_at: Instant,
    /// Run lockbar detection on each depth frame and overlay it.
    lockbar_enabled: bool,
    last_lockbar: Option<headtracking::calibration::LockbarObservation>,
    /// `Some` when face detection is enabled (currently auto-enabled for
    /// the webcam backend). Cheap to keep around — YuNet runs in <10 ms
    /// at 320×320 on CPU.
    face_detector: Option<face::Detector>,
    last_faces: Vec<face::FaceDetection>,
}

mod filter_alias {
    // The plugin's filter module isn't exposed as a sibling crate, but it's
    // a standalone in-tree module — duplicate it here would mean another
    // copy of identical code. Instead, ht-debug pulls it directly via the
    // workspace's `headtracking` crate path.
    pub use headtracking::filter::OneEuroPose3D;
}

/// State for the Kinect v1 tilt + LED panel. The desired values are kept
/// here so the user can drag the slider freely; we only push commands to
/// the device on `drag_stopped` / combo change to avoid hammering the
/// fragile motor gears.
struct V1Controls {
    desired_tilt_deg: f32,
    last_sent_tilt_deg: f32,
    selected_led: freenect::LedState,
    last_sent_led: freenect::LedState,
    last_state: Option<freenect::TiltState>,
    last_refresh: Instant,
}

impl V1Controls {
    fn new() -> Self {
        Self {
            desired_tilt_deg: 0.0,
            last_sent_tilt_deg: 0.0,
            selected_led: freenect::LedState::Green,
            last_sent_led: freenect::LedState::Green,
            last_state: None,
            // Seed with a stale instant so the first poll triggers a refresh.
            last_refresh: Instant::now() - Duration::from_secs(60),
        }
    }
}

#[derive(Clone, Copy)]
struct Intrinsics {
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
}

enum Inner {
    KinectV2 {
        device: freenect2::Device,
        _ctx: freenect2::Context,
    },
    KinectV1 {
        device: freenect::Device,
        _ctx: freenect::Context,
    },
    Webcam {
        camera: webcam::Camera,
    },
}

impl Inner {
    /// `true` when this input pipeline produces 3D head poses (depth blob
    /// for Kinect, face landmarks + IOD triangulation for webcam).
    fn has_head_tracker(&self) -> bool {
        matches!(
            self,
            Inner::KinectV1 { .. } | Inner::KinectV2 { .. } | Inner::Webcam { .. }
        )
    }
}

/// Pick the largest detected face by bounding-box area. The largest face is
/// usually the one closest to the camera, which on a pincab is the player.
fn pick_largest_face(faces: &[face::FaceDetection]) -> Option<&face::FaceDetection> {
    faces.iter().max_by(|a, b| {
        let area_a = a.width * a.height;
        let area_b = b.width * b.height;
        area_a
            .partial_cmp(&area_b)
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

/// Build a [`HeadPixel`] from a face detection. Z is triangulated from the
/// interpupillary pixel distance assuming a 63 mm physical IOD and a
/// nominal `fx ≈ 0.85 × frame_width` (typical 60° HFOV webcam). These
/// numbers are placeholders until `ht-calibrate` measures the real focal
/// length via the lockbar fiducial.
fn face_to_head(face: &face::FaceDetection, frame_w: u32, frame_h: u32) -> HeadPixel {
    const IOD_MM: f32 = 63.0;
    let fx = 0.85 * frame_w as f32;
    let fy = fx;
    let cx = (frame_w as f32) / 2.0;
    let cy = (frame_h as f32) / 2.0;

    let dx = face.left_eye_x - face.right_eye_x;
    let dy = face.left_eye_y - face.right_eye_y;
    let pixel_iod = (dx * dx + dy * dy).sqrt().max(1.0);
    let depth_mm = IOD_MM * fx / pixel_iod;

    // Eye-midpoint as the head pixel.
    let u = (face.left_eye_x + face.right_eye_x) * 0.5;
    let v = (face.left_eye_y + face.right_eye_y) * 0.5;

    let x_mm = (u - cx) * depth_mm / fx;
    let y_mm = (v - cy) * depth_mm / fy;

    HeadPixel {
        u: u.max(0.0) as u32,
        v: v.max(0.0) as u32,
        frame_w,
        frame_h,
        depth_mm,
        x_mm,
        y_mm,
    }
}

#[derive(Debug, Clone, Copy)]
struct HeadPixel {
    /// Pixel coords inside the depth grid (different size per sensor).
    u: u32,
    v: u32,
    /// Frame width — needed by the painter to map back to display coords.
    frame_w: u32,
    frame_h: u32,
    depth_mm: f32,
    x_mm: f32,
    y_mm: f32,
}

#[derive(Debug, Clone, Copy)]
struct Baseline {
    x_mm: f32,
    y_mm: f32,
    z_mm: f32,
}

impl App {
    fn new(logs: Arc<Mutex<VecDeque<String>>>) -> Self {
        let available = detect_backends();
        Self {
            selected: Backend::None,
            available,
            active: None,
            error: None,
            logs,
        }
    }

    fn refresh_available(&mut self) {
        // Drop the active device first — libfreenect[2] can't reliably
        // enumerate while a sibling context holds an open device on Linux.
        if let Some(old) = self.active.take() {
            info!(backend = ?old.backend, "closing backend before scan");
            drop(old);
        }
        self.selected = Backend::None;
        self.available = detect_backends();
    }

    fn ensure_active(&mut self) {
        let needs_change = match (&self.active, self.selected) {
            (Some(a), sel) => a.backend != sel,
            (None, Backend::None) => false,
            (None, _) => true,
        };
        if !needs_change {
            return;
        }
        if let Some(old) = self.active.take() {
            info!(backend = ?old.backend, "closing backend");
            drop(old);
        }
        self.error = None;
        if matches!(self.selected, Backend::None) {
            return;
        }
        match open_backend(self.selected) {
            Ok(active) => {
                info!(
                    backend = ?active.backend,
                    fx = active.intrinsics.fx,
                    fy = active.intrinsics.fy,
                    cx = active.intrinsics.cx,
                    cy = active.intrinsics.cy,
                    "backend opened"
                );
                self.active = Some(active);
            }
            Err(e) => {
                error!(?e, "failed to open backend");
                self.error = Some(e);
                self.selected = Backend::None;
            }
        }
    }

    fn poll(&mut self, egui_ctx: &egui::Context) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        match &mut active.inner {
            Inner::KinectV2 { device, .. } => {
                if let Some(rgb) = device.poll_rgb() {
                    let img = bgrx_to_color_image(rgb.width, rgb.height, &rgb.data);
                    upload_texture(egui_ctx, &mut active.rgb_texture, img);
                }
                if let Some(depth) = device.poll_depth() {
                    let head =
                        find_head_f32(&depth.data, depth.width, depth.height, &active.intrinsics);
                    let smoothed = smooth_head(
                        head,
                        &active.intrinsics,
                        &mut active.pose_filter,
                        active.started_at,
                    );
                    capture_baseline(&mut active.baseline, smoothed);
                    active.last_head = smoothed;
                    if active.lockbar_enabled {
                        active.last_lockbar = headtracking::calibration::detect_lockbar(
                            &depth.data,
                            depth.width,
                            depth.height,
                            &headtracking::calibration::LockbarParams::default(),
                        );
                    }
                }
            }
            Inner::KinectV1 { device, .. } => {
                if let Some(rgb) = device.poll_rgb() {
                    let img = rgb888_to_color_image(rgb.width, rgb.height, &rgb.data);
                    upload_texture(egui_ctx, &mut active.rgb_texture, img);
                }
                if let Some(depth) = device.poll_depth() {
                    // libfreenect ships u16 mm; widen for the shared algo.
                    let f32_data: Vec<f32> = depth.data.iter().map(|&v| f32::from(v)).collect();
                    let head =
                        find_head_f32(&f32_data, depth.width, depth.height, &active.intrinsics);
                    let smoothed = smooth_head(
                        head,
                        &active.intrinsics,
                        &mut active.pose_filter,
                        active.started_at,
                    );
                    capture_baseline(&mut active.baseline, smoothed);
                    active.last_head = smoothed;
                    if active.lockbar_enabled {
                        active.last_lockbar = headtracking::calibration::detect_lockbar(
                            &f32_data,
                            depth.width,
                            depth.height,
                            &headtracking::calibration::LockbarParams::default(),
                        );
                    }
                }
            }
            Inner::Webcam { camera } => {
                if let Some(rgb) = camera.poll_rgb() {
                    // Face detection on the raw camera frame (before the
                    // ColorImage conversion strips the contiguous bytes).
                    if let Some(detector) = active.face_detector.as_mut() {
                        active.last_faces = detector.detect(&rgb.data, rgb.width, rgb.height);
                        if let Some(face) = pick_largest_face(&active.last_faces) {
                            let head = face_to_head(face, rgb.width, rgb.height);
                            let smoothed = smooth_head(
                                Some(head),
                                &active.intrinsics,
                                &mut active.pose_filter,
                                active.started_at,
                            );
                            capture_baseline(&mut active.baseline, smoothed);
                            active.last_head = smoothed;
                        } else {
                            active.last_head = None;
                        }
                    }
                    let img = rgb888_to_color_image(rgb.width, rgb.height, &rgb.data);
                    upload_texture(egui_ctx, &mut active.rgb_texture, img);
                }
            }
        }
    }
}

/// Apply the 1€ filter to the head pose in millimetres, then re-project
/// the smoothed (x, y, z) to pixel coordinates so the crosshair tracks the
/// same smoothed point. Returns `None` if the input was already `None`.
fn smooth_head(
    raw: Option<HeadPixel>,
    intr: &Intrinsics,
    filter: &mut filter_alias::OneEuroPose3D,
    started_at: Instant,
) -> Option<HeadPixel> {
    let mut head = raw?;
    let t_us = started_at.elapsed().as_micros() as u64;
    let smoothed = filter.update([head.x_mm, head.y_mm, head.depth_mm], t_us);
    head.x_mm = smoothed[0];
    head.y_mm = smoothed[1];
    head.depth_mm = smoothed[2];
    if head.depth_mm > 0.0 && intr.fx > 0.0 && intr.fy > 0.0 {
        let u = intr.cx + head.x_mm * intr.fx / head.depth_mm;
        let v = intr.cy + head.y_mm * intr.fy / head.depth_mm;
        head.u = u.max(0.0) as u32;
        head.v = v.max(0.0) as u32;
    }
    Some(head)
}

impl App {
    /// Render the Kinect v1 tilt + LED panel just below the toolbar.
    /// No-op when the active backend is anything else.
    fn show_v1_controls(&mut self, egui_ctx: &egui::Context) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        let (Inner::KinectV1 { device, .. }, Some(controls)) =
            (&mut active.inner, active.v1_controls.as_mut())
        else {
            return;
        };

        // Refresh tilt + accel every 500 ms (USB roundtrip).
        if controls.last_refresh.elapsed() >= Duration::from_millis(500) {
            match device.tilt_state() {
                Ok(state) => controls.last_state = Some(state),
                Err(e) => warn!(?e, "kinect v1: tilt_state refresh failed"),
            }
            controls.last_refresh = Instant::now();
        }

        TopBottomPanel::top("v1-controls").show(egui_ctx, |ui| {
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                ui.label(RichText::new("Kinect v1").strong());
                ui.separator();
                let response = ui.add(
                    egui::Slider::new(
                        &mut controls.desired_tilt_deg,
                        freenect::TILT_MIN_DEG..=freenect::TILT_MAX_DEG,
                    )
                    .text("tilt °")
                    .step_by(1.0),
                );
                let drag_release = response.drag_stopped();
                let typed_commit = response.lost_focus();
                if (drag_release || typed_commit)
                    && (controls.desired_tilt_deg - controls.last_sent_tilt_deg).abs() > 0.01
                {
                    if let Err(e) = device.set_tilt_degrees(controls.desired_tilt_deg) {
                        warn!(?e, "set_tilt failed");
                    } else {
                        controls.last_sent_tilt_deg = controls.desired_tilt_deg;
                        info!(angle = controls.desired_tilt_deg, "tilt command sent");
                    }
                }

                ui.separator();
                ui.label("LED:");
                let prev_led = controls.selected_led;
                ComboBox::from_id_salt("led")
                    .selected_text(led_label(controls.selected_led))
                    .show_ui(ui, |ui| {
                        for led in LED_OPTIONS {
                            ui.selectable_value(&mut controls.selected_led, *led, led_label(*led));
                        }
                    });
                if controls.selected_led != prev_led {
                    if let Err(e) = device.set_led(controls.selected_led) {
                        warn!(?e, "set_led failed");
                    } else {
                        controls.last_sent_led = controls.selected_led;
                    }
                }

                ui.separator();
                if let Some(state) = controls.last_state {
                    ui.label(
                        RichText::new(format!(
                            "current {:>+5.1}°  status {:?}  accel ({:>+5.2}, {:>+5.2}, {:>+5.2}) m/s²",
                            state.angle_deg,
                            state.status,
                            state.accel_mks[0],
                            state.accel_mks[1],
                            state.accel_mks[2],
                        ))
                        .monospace()
                        .color(Color32::GRAY),
                    );
                } else {
                    ui.label(RichText::new("waiting for motor state…").color(Color32::GRAY));
                }
            });
            ui.add_space(2.0);
        });
    }
}

const LED_OPTIONS: &[freenect::LedState] = &[
    freenect::LedState::Off,
    freenect::LedState::Green,
    freenect::LedState::Red,
    freenect::LedState::Yellow,
    freenect::LedState::BlinkGreen,
    freenect::LedState::BlinkRedYellow,
];

fn led_label(state: freenect::LedState) -> &'static str {
    match state {
        freenect::LedState::Off => "off",
        freenect::LedState::Green => "green",
        freenect::LedState::Red => "red",
        freenect::LedState::Yellow => "yellow",
        freenect::LedState::BlinkGreen => "blink green",
        freenect::LedState::BlinkRedYellow => "blink red/yellow",
    }
}

fn upload_texture(ctx: &egui::Context, slot: &mut Option<TextureHandle>, img: ColorImage) {
    match slot.as_mut() {
        Some(tex) => tex.set(img, egui::TextureOptions::LINEAR),
        None => *slot = Some(ctx.load_texture("rgb", img, egui::TextureOptions::LINEAR)),
    }
}

fn capture_baseline(slot: &mut Option<Baseline>, head: Option<HeadPixel>) {
    if slot.is_some() {
        return;
    }
    let Some(head) = head else { return };
    let baseline = Baseline {
        x_mm: head.x_mm,
        y_mm: head.y_mm,
        z_mm: head.depth_mm,
    };
    *slot = Some(baseline);
    info!(
        x_mm = baseline.x_mm,
        y_mm = baseline.y_mm,
        z_mm = baseline.z_mm,
        "baseline captured"
    );
}

impl eframe::App for App {
    fn update(&mut self, egui_ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.ensure_active();
        self.poll(egui_ctx);

        // ----- Top toolbar
        TopBottomPanel::top("toolbar").show(egui_ctx, |ui| {
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Input:");
                let selected_label = self.label_for(self.selected);
                ComboBox::from_id_salt("backend")
                    .selected_text(selected_label)
                    .show_ui(ui, |ui| {
                        let entries = self.available.clone();
                        for entry in &entries {
                            ui.selectable_value(
                                &mut self.selected,
                                entry.backend,
                                &entry.label,
                            );
                        }
                    });
                if ui.small_button("rescan").clicked() {
                    self.refresh_available();
                }
                ui.separator();
                if let Some(active) = self.active.as_mut()
                    && active.inner.has_head_tracker()
                {
                    ui.checkbox(&mut active.lockbar_enabled, "lockbar");
                    if !active.lockbar_enabled {
                        active.last_lockbar = None;
                    }
                    if let Some(bar) = active.last_lockbar {
                        ui.label(
                            RichText::new(format!(
                                "row {}, width {} px, depth {:.0} mm (σ {:.1})",
                                bar.row,
                                bar.width_px(),
                                bar.mean_depth_mm,
                                bar.depth_stddev_mm,
                            ))
                            .color(Color32::from_rgb(0xff, 0x40, 0x80))
                            .monospace()
                            .size(11.0),
                        );
                    }
                    ui.separator();
                }
                if let Some(active) = self.active.as_ref() {
                    let label = self.label_for(active.backend);
                    if !active.inner.has_head_tracker() {
                        ui.label(
                            RichText::new(format!(
                                "{label}  |  capture only — head tracking pending"
                            ))
                            .color(Color32::GRAY),
                        );
                    } else if let Some(head) = active.last_head {
                        ui.label(
                            RichText::new(format!(
                                "{label}  |  distance {:.0} mm  |  pixel ({}, {})  |  3D ({:.0}, {:.0}, {:.0}) mm",
                                head.depth_mm,
                                head.u,
                                head.v,
                                head.x_mm,
                                head.y_mm,
                                head.depth_mm
                            ))
                            .monospace(),
                        );
                    } else {
                        ui.label(
                            RichText::new(format!("{label}  |  waiting for head detection…"))
                                .color(Color32::GRAY),
                        );
                    }
                } else if let Some(err) = &self.error {
                    ui.colored_label(Color32::LIGHT_RED, err);
                } else if self.available.len() <= 1 {
                    ui.label(RichText::new("no input detected — plug a Kinect and click 'rescan'").color(Color32::GRAY));
                } else {
                    ui.label(RichText::new("select an input").color(Color32::GRAY));
                }
            });
            ui.add_space(4.0);
        });

        // ----- Optional Kinect v1 controls (tilt + LED)
        self.show_v1_controls(egui_ctx);

        // ----- Bottom split: logs (left) + VPX delta panel (right)
        TopBottomPanel::bottom("debug-panels")
            .resizable(true)
            .default_height(220.0)
            .min_height(80.0)
            .show(egui_ctx, |ui| {
                ui.add_space(4.0);
                ui.columns(2, |cols| {
                    // Left: tracing event log
                    cols[0].horizontal(|ui| {
                        ui.label(RichText::new("logs").strong());
                        if ui.small_button("clear").clicked() {
                            self.logs.lock().clear();
                        }
                    });
                    ScrollArea::vertical()
                        .id_salt("log-scroll")
                        .auto_shrink([false; 2])
                        .stick_to_bottom(true)
                        .show(&mut cols[0], |ui| {
                            let logs = self.logs.lock();
                            ui.with_layout(Layout::top_down(Align::LEFT), |ui| {
                                for line in logs.iter() {
                                    ui.label(RichText::new(line).monospace().size(12.0));
                                }
                            });
                        });

                    // Right: VPX delta panel
                    let mut reset_baseline = false;
                    cols[1].horizontal(|ui| {
                        ui.label(RichText::new("VPX output (Δ view)").strong());
                        if ui.small_button("reset baseline").clicked() {
                            reset_baseline = true;
                        }
                    });
                    cols[1].add_space(2.0);
                    if reset_baseline && let Some(active) = self.active.as_mut() {
                        active.baseline = None;
                    }
                    if let Some(active) = self.active.as_ref() {
                        if !active.inner.has_head_tracker() {
                            cols[1].label(
                                RichText::new(
                                    "this input has no head tracker yet\n\
                                     (face detection / monocular depth comes\n\
                                     with the webcam tracker — P3 roadmap)",
                                )
                                .color(Color32::GRAY)
                                .monospace(),
                            );
                            return;
                        }
                        match (active.baseline, active.last_head) {
                            (Some(base), Some(head)) => {
                                let dx_mm = head.x_mm - base.x_mm;
                                let dy_mm = head.y_mm - base.y_mm;
                                let dz_mm = head.depth_mm - base.z_mm;
                                let (vx, vy, vz) =
                                    pose_delta_to_view_delta_vpu(dx_mm, dy_mm, dz_mm);
                                cols[1].label(
                                    RichText::new(format!(
                                        "baseline (mm)  ({:>6.0}, {:>6.0}, {:>6.0})\n\
                                         current  (mm)  ({:>6.0}, {:>6.0}, {:>6.0})\n\
                                         Δ pose   (mm)  ({:>+6.0}, {:>+6.0}, {:>+6.0})\n\n\
                                         Δ view  (VPU)  ({:>+6.2}, {:>+6.2}, {:>+6.2})\n\
                                                       viewX += {:>+6.2}\n\
                                                       viewY += {:>+6.2}\n\
                                                       viewZ += {:>+6.2}",
                                        base.x_mm,
                                        base.y_mm,
                                        base.z_mm,
                                        head.x_mm,
                                        head.y_mm,
                                        head.depth_mm,
                                        dx_mm,
                                        dy_mm,
                                        dz_mm,
                                        vx,
                                        vy,
                                        vz,
                                        vx,
                                        vy,
                                        vz,
                                    ))
                                    .monospace()
                                    .size(12.0),
                                );
                            }
                            _ => {
                                cols[1].label(
                                    RichText::new("waiting for baseline…")
                                        .color(Color32::GRAY)
                                        .monospace(),
                                );
                            }
                        }
                    } else {
                        cols[1].label(
                            RichText::new("input is off")
                                .color(Color32::GRAY)
                                .monospace(),
                        );
                    }
                });
            });

        // ----- Center: image with crosshair
        CentralPanel::default().show(egui_ctx, |ui| {
            let avail = ui.available_size();
            let aspect = match self.active.as_ref() {
                Some(active) => match (&active.inner, active.rgb_texture.as_ref()) {
                    (_, Some(tex)) => {
                        let s = tex.size_vec2();
                        if s.y > 0.0 { s.x / s.y } else { 16.0 / 9.0 }
                    }
                    (Inner::KinectV2 { .. }, None) => 1920.0 / 1080.0,
                    (Inner::KinectV1 { .. }, None) => 640.0 / 480.0,
                    (Inner::Webcam { .. }, None) => 640.0 / 480.0,
                },
                None => 16.0 / 9.0,
            };
            let (img_w, img_h) = if avail.x / avail.y > aspect {
                (avail.y * aspect, avail.y)
            } else {
                (avail.x, avail.x / aspect)
            };
            let (rect, _) = ui.allocate_exact_size(Vec2::new(img_w, img_h), Sense::hover());

            if let Some(active) = self.active.as_ref() {
                if let Some(tex) = &active.rgb_texture {
                    ui.painter().image(
                        tex.id(),
                        rect,
                        Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
                        Color32::WHITE,
                    );
                    if let Some(head) = active.last_head {
                        draw_crosshair(ui.painter(), rect, head);
                    }
                    if let Some(bar) = active.last_lockbar {
                        draw_lockbar(ui.painter(), rect, bar);
                    }
                } else {
                    centered(ui, rect, "waiting for first RGB frame…");
                }
            } else {
                let msg = self
                    .error
                    .as_deref()
                    .unwrap_or("select an input device above to start streaming");
                centered(ui, rect, msg);
            }
        });

        egui_ctx.request_repaint();
    }
}

fn centered(ui: &mut egui::Ui, rect: Rect, text: &str) {
    ui.painter().rect_filled(rect, 4.0, Color32::from_gray(20));
    ui.painter().text(
        rect.center(),
        egui::Align2::CENTER_CENTER,
        text,
        egui::FontId::proportional(16.0),
        Color32::LIGHT_GRAY,
    );
}

fn draw_lockbar(
    painter: &egui::Painter,
    rect: Rect,
    bar: headtracking::calibration::LockbarObservation,
) {
    if bar.frame_width == 0 || bar.frame_height == 0 {
        return;
    }
    // Same caveat as the head crosshair: depth frame and RGB frame on the
    // Kinect v2 are not co-axial, so the bar visualisation is a few pixels
    // off the true RGB position. Good enough for "is it locked on?".
    let v_norm = bar.row as f32 / bar.frame_height as f32;
    let l_norm = bar.left_col as f32 / bar.frame_width as f32;
    let r_norm = bar.right_col as f32 / bar.frame_width as f32;
    let p_left = rect.left_top() + Vec2::new(l_norm * rect.width(), v_norm * rect.height());
    let p_right = rect.left_top() + Vec2::new(r_norm * rect.width(), v_norm * rect.height());
    let pink = Color32::from_rgb(0xff, 0x40, 0x80);
    painter.line_segment([p_left, p_right], Stroke::new(3.0, pink));
    // Tick marks at each end.
    painter.line_segment(
        [p_left + Vec2::new(0.0, -8.0), p_left + Vec2::new(0.0, 8.0)],
        Stroke::new(2.0, pink),
    );
    painter.line_segment(
        [
            p_right + Vec2::new(0.0, -8.0),
            p_right + Vec2::new(0.0, 8.0),
        ],
        Stroke::new(2.0, pink),
    );
}

fn draw_crosshair(painter: &egui::Painter, rect: Rect, head: HeadPixel) {
    // Map depth-frame pixel coords into the displayed image rectangle. The
    // IR/RGB sensors aren't co-axial so this is a few percent off the true
    // RGB pixel (parallax) — fine for "is the tracker on my head?" debugging.
    let u_norm = head.u as f32 / head.frame_w as f32;
    let v_norm = head.v as f32 / head.frame_h as f32;
    let center = rect.left_top() + Vec2::new(u_norm * rect.width(), v_norm * rect.height());

    let yellow = Color32::from_rgb(0xff, 0xee, 0x00);
    let green = Color32::from_rgb(0x00, 0xff, 0x66);
    painter.line_segment(
        [center - Vec2::X * 18.0, center + Vec2::X * 18.0],
        Stroke::new(2.0, yellow),
    );
    painter.line_segment(
        [center - Vec2::Y * 18.0, center + Vec2::Y * 18.0],
        Stroke::new(2.0, yellow),
    );
    painter.circle_stroke(center, 28.0, Stroke::new(2.0, green));
}

// ============================================================ Backend opening

fn open_backend(b: Backend) -> Result<Active, String> {
    match b {
        Backend::None => Err("no backend selected".to_string()),
        Backend::KinectV2 => open_kinect_v2(),
        Backend::KinectV1 => open_kinect_v1(),
        Backend::Webcam(idx) => open_webcam(idx),
    }
}

fn open_kinect_v2() -> Result<Active, String> {
    let ctx = freenect2::Context::new().map_err(|e| format!("freenect2 Context::new: {e}"))?;
    let count = ctx.enumerate();
    if count <= 0 {
        return Err("no Kinect v2 found on USB".to_string());
    }
    let device = ctx
        .open_default()
        .map_err(|e| format!("freenect2 open_default: {e}"))?;
    device
        .start_streams(true, true)
        .map_err(|e| format!("freenect2 start_streams: {e}"))?;
    let p = device.ir_params();
    Ok(Active {
        backend: Backend::KinectV2,
        intrinsics: Intrinsics {
            fx: p.fx,
            fy: p.fy,
            cx: p.cx,
            cy: p.cy,
        },
        rgb_texture: None,
        last_head: None,
        baseline: None,
        inner: Inner::KinectV2 { device, _ctx: ctx },
        v1_controls: None,
        pose_filter: filter_alias::OneEuroPose3D::with_defaults(),
        started_at: Instant::now(),
        lockbar_enabled: false,
        last_lockbar: None,
        face_detector: None,
        last_faces: Vec::new(),
    })
}

fn open_kinect_v1() -> Result<Active, String> {
    let ctx = freenect::Context::new().map_err(|e| format!("freenect Context::new: {e}"))?;
    let count = ctx.enumerate();
    if count <= 0 {
        return Err("no Kinect v1 found on USB".to_string());
    }
    let mut device = ctx.open(0).map_err(|e| format!("freenect open: {e}"))?;
    device
        .start_streams(true, true)
        .map_err(|e| format!("freenect start_streams: {e}"))?;

    // Seed the v1 controls with the device's current tilt so the slider
    // doesn't snap on first use. Failures are non-fatal — we just log.
    let mut controls = V1Controls::new();
    match device.tilt_state() {
        Ok(state) => {
            controls.desired_tilt_deg = state.angle_deg;
            controls.last_sent_tilt_deg = state.angle_deg;
            controls.last_state = Some(state);
            controls.last_refresh = Instant::now();
        }
        Err(e) => warn!(?e, "kinect v1: tilt_state read at open failed"),
    }
    if let Err(e) = device.set_led(controls.selected_led) {
        warn!(?e, "kinect v1: initial set_led failed");
    }

    Ok(Active {
        backend: Backend::KinectV1,
        intrinsics: Intrinsics {
            fx: freenect::FX,
            fy: freenect::FY,
            cx: freenect::CX,
            cy: freenect::CY,
        },
        rgb_texture: None,
        last_head: None,
        baseline: None,
        inner: Inner::KinectV1 { device, _ctx: ctx },
        v1_controls: Some(controls),
        pose_filter: filter_alias::OneEuroPose3D::with_defaults(),
        started_at: Instant::now(),
        lockbar_enabled: false,
        last_lockbar: None,
        face_detector: None,
        last_faces: Vec::new(),
    })
}

fn open_webcam(index: u32) -> Result<Active, String> {
    let camera = webcam::Camera::open(index).map_err(|e| format!("webcam open: {e}"))?;
    let detector = match face::Detector::new() {
        Ok(d) => {
            info!("face detector initialised (YuNet)");
            Some(d)
        }
        Err(e) => {
            warn!(
                ?e,
                "face detector failed to initialise; webcam will run capture-only"
            );
            None
        }
    };
    Ok(Active {
        backend: Backend::Webcam(index),
        // Without lockbar/disc calibration, fx ≈ 0.85 × frame_width is a
        // reasonable placeholder for a generic 60° HFOV webcam. The values
        // get replaced by ht-calibrate output when that lands.
        intrinsics: Intrinsics {
            fx: 0.0,
            fy: 0.0,
            cx: 0.0,
            cy: 0.0,
        },
        rgb_texture: None,
        last_head: None,
        baseline: None,
        inner: Inner::Webcam { camera },
        v1_controls: None,
        pose_filter: filter_alias::OneEuroPose3D::with_defaults(),
        started_at: Instant::now(),
        lockbar_enabled: false,
        last_lockbar: None,
        face_detector: detector,
        last_faces: Vec::new(),
    })
}

// ============================================================ Image conversion

fn bgrx_to_color_image(width: u32, height: u32, data: &[u8]) -> ColorImage {
    debug_assert_eq!(data.len(), (width * height * 4) as usize);
    let mut pixels = Vec::with_capacity((width * height) as usize);
    for chunk in data.chunks_exact(4) {
        // libfreenect2 ships pixels as B, G, R, X.
        pixels.push(Color32::from_rgb(chunk[2], chunk[1], chunk[0]));
    }
    ColorImage {
        size: [width as usize, height as usize],
        pixels,
    }
}

fn rgb888_to_color_image(width: u32, height: u32, data: &[u8]) -> ColorImage {
    debug_assert_eq!(data.len(), (width * height * 3) as usize);
    let mut pixels = Vec::with_capacity((width * height) as usize);
    for chunk in data.chunks_exact(3) {
        // libfreenect ships v1 video as R, G, B.
        pixels.push(Color32::from_rgb(chunk[0], chunk[1], chunk[2]));
    }
    ColorImage {
        size: [width as usize, height as usize],
        pixels,
    }
}

// ============================================================ Head blob algo

fn find_head_f32(data: &[f32], width: u32, height: u32, intr: &Intrinsics) -> Option<HeadPixel> {
    let w = width as i32;
    let h = height as i32;
    if w <= 0 || h <= 0 {
        return None;
    }

    // Pass 1: closest valid pixel.
    let valid = DEPTH_MIN_MM..=DEPTH_MAX_MM;
    let mut min_z = f32::INFINITY;
    let mut min_idx: i32 = -1;
    for (i, &z) in data.iter().enumerate() {
        if !valid.contains(&z) {
            continue;
        }
        if z < min_z {
            min_z = z;
            min_idx = i as i32;
        }
    }
    if min_idx < 0 {
        return None;
    }
    let cu = min_idx % w;
    let cv = min_idx / w;

    // Pass 2: 50×50 window centroid in [min_z, min_z + 150 mm].
    let z_max = min_z + 150.0;
    let (mut sx, mut sy, mut sz) = (0.0_f64, 0.0_f64, 0.0_f64);
    let (mut wsum_u, mut wsum_v) = (0.0_f64, 0.0_f64);
    let mut count: u32 = 0;
    let u0 = (cu - WINDOW_HALF).max(0);
    let u1 = (cu + WINDOW_HALF).min(w - 1);
    let v0 = (cv - WINDOW_HALF).max(0);
    let v1 = (cv + WINDOW_HALF).min(h - 1);
    for v in v0..=v1 {
        let row = (v * w) as usize;
        for u in u0..=u1 {
            let z = data[row + u as usize];
            if z < DEPTH_MIN_MM || z > z_max {
                continue;
            }
            let zf = f64::from(z);
            sx += f64::from(u as f32 - intr.cx) * zf / f64::from(intr.fx);
            sy += f64::from(v as f32 - intr.cy) * zf / f64::from(intr.fy);
            sz += zf;
            wsum_u += f64::from(u);
            wsum_v += f64::from(v);
            count += 1;
        }
    }
    if count < MIN_VALID_PIXELS {
        return None;
    }
    let n = f64::from(count);
    Some(HeadPixel {
        u: (wsum_u / n) as u32,
        v: (wsum_v / n) as u32,
        frame_w: width,
        frame_h: height,
        depth_mm: (sz / n) as f32,
        x_mm: (sx / n) as f32,
        y_mm: (sy / n) as f32,
    })
}

// ============================================================ VPU mapping
//
// Mirrors `crate::camera::mapping::pose_delta_to_view_delta` from the plugin.
// Keep in sync with `src/camera/mapping.rs` if the axis convention changes.

const VPU_PER_MM: f64 = 50.0 / (25.4 * 1.0625);

fn pose_delta_to_view_delta_vpu(dx_mm: f32, dy_mm: f32, dz_mm: f32) -> (f32, f32, f32) {
    let to_vpu = |mm: f32| (f64::from(mm) * VPU_PER_MM) as f32;
    // Kinect Y points down (head going up → -Y) and Z grows away from the
    // sensor (head approaching → -Z). VPX Camera-mode Y is "forward away
    // from the player" and Z is upward.
    (to_vpu(dx_mm), -to_vpu(dz_mm), -to_vpu(dy_mm))
}

// ============================================================ Tracing capture

fn init_tracing(sink: Arc<Mutex<VecDeque<String>>>) {
    let env_filter = tracing_subscriber::EnvFilter::try_from_env("HEADTRACKING_LOG")
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));
    let stderr_layer = tracing_subscriber::fmt::layer()
        .with_target(false)
        .with_writer(std::io::stderr);
    let panel_layer = tracing_subscriber::fmt::layer()
        .with_target(false)
        .with_ansi(false)
        .with_writer(LogQueue { sink });
    tracing_subscriber::registry()
        .with(env_filter)
        .with(stderr_layer)
        .with(panel_layer)
        .init();
}

#[derive(Clone)]
struct LogQueue {
    sink: Arc<Mutex<VecDeque<String>>>,
}

impl<'a> MakeWriter<'a> for LogQueue {
    type Writer = LogWriter;
    fn make_writer(&'a self) -> LogWriter {
        LogWriter {
            sink: Arc::clone(&self.sink),
            buf: Vec::with_capacity(256),
        }
    }
}

struct LogWriter {
    sink: Arc<Mutex<VecDeque<String>>>,
    buf: Vec<u8>,
}

impl Write for LogWriter {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        self.buf.extend_from_slice(data);
        Ok(data.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        if self.buf.is_empty() {
            return Ok(());
        }
        let text = String::from_utf8_lossy(&self.buf).into_owned();
        self.buf.clear();
        let mut sink = self.sink.lock();
        for line in text.lines() {
            if line.is_empty() {
                continue;
            }
            if sink.len() >= LOG_BUFFER_LINES {
                sink.pop_front();
            }
            sink.push_back(line.to_string());
        }
        Ok(())
    }
}

impl Drop for LogWriter {
    fn drop(&mut self) {
        let _ = self.flush();
    }
}
