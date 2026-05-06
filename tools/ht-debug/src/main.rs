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

use eframe::egui::{
    self, Align, CentralPanel, Color32, ColorImage, ComboBox, Layout, Pos2, Rect, RichText,
    ScrollArea, Sense, Stroke, TextureHandle, TopBottomPanel, Vec2,
};
use parking_lot::Mutex;
use tracing::{error, info};
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
}

impl Backend {
    fn label(self) -> &'static str {
        match self {
            Backend::None => "None (off)",
            Backend::KinectV1 => "Kinect v1",
            Backend::KinectV2 => "Kinect v2",
        }
    }
}

/// Probe USB for connected Kinect sensors. Always returns `None` first; the
/// other entries are added when the corresponding library reports a device.
fn detect_backends() -> Vec<Backend> {
    let mut out = vec![Backend::None];

    match freenect2::Context::new() {
        Ok(ctx) => {
            let n = ctx.enumerate();
            if n > 0 {
                out.push(Backend::KinectV2);
                info!(count = n, "kinect v2 detected");
            }
        }
        Err(e) => info!(?e, "kinect v2 enumerate failed"),
    }

    match freenect::Context::new() {
        Ok(ctx) => {
            let n = ctx.enumerate();
            if n > 0 {
                out.push(Backend::KinectV1);
                info!(count = n, "kinect v1 detected");
            }
        }
        Err(e) => info!(?e, "kinect v1 enumerate failed"),
    }

    out
}

// ============================================================ App state

struct App {
    selected: Backend,
    available: Vec<Backend>,
    active: Option<Active>,
    error: Option<String>,
    logs: Arc<Mutex<VecDeque<String>>>,
}

struct Active {
    backend: Backend,
    intrinsics: Intrinsics,
    rgb_texture: Option<TextureHandle>,
    last_head: Option<HeadPixel>,
    baseline: Option<Baseline>,
    inner: Inner,
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
                    capture_baseline(&mut active.baseline, head);
                    active.last_head = head;
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
                    capture_baseline(&mut active.baseline, head);
                    active.last_head = head;
                }
            }
        }
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
                ComboBox::from_id_salt("backend")
                    .selected_text(self.selected.label())
                    .show_ui(ui, |ui| {
                        for b in &self.available {
                            ui.selectable_value(&mut self.selected, *b, b.label());
                        }
                    });
                if ui.small_button("rescan").clicked() {
                    self.refresh_available();
                }
                ui.separator();
                if let Some(active) = self.active.as_ref() {
                    if let Some(head) = active.last_head {
                        ui.label(
                            RichText::new(format!(
                                "{}  |  distance {:.0} mm  |  pixel ({}, {})  |  3D ({:.0}, {:.0}, {:.0}) mm",
                                active.backend.label(),
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
                            RichText::new(format!("{}  |  waiting for head detection…", active.backend.label()))
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
                Some(active) => match active.inner {
                    Inner::KinectV2 { .. } => 1920.0 / 1080.0,
                    Inner::KinectV1 { .. } => 640.0 / 480.0,
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
