//! `ht-debug`: standalone Kinect-v2 viewer for the head-tracker pipeline.
//!
//! Displays the live RGB feed with a crosshair on the detected head, the
//! current distance, and a scrollable log panel — all in one window.
//! A dropdown at the top selects the input backend (currently None / Kinect
//! v2 — Kinect v1 and webcam will plug into the same selector).
//!
//! Run with `cargo run --release -p ht-debug`.

use std::collections::VecDeque;
use std::io::{self, Write};
use std::sync::Arc;

use eframe::egui::{
    self, Align, CentralPanel, Color32, ColorImage, ComboBox, Layout, Pos2, Rect, RichText,
    ScrollArea, Sense, Stroke, TextureHandle, TopBottomPanel, Vec2,
};
use freenect2::{Context, DepthFrame, Device, IrCameraParams, RgbFrame};
use parking_lot::Mutex;
use tracing::{error, info};
use tracing_subscriber::fmt::MakeWriter;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;

const RGB_WIDTH: usize = 1920;
const RGB_HEIGHT: usize = 1080;
const DEPTH_WIDTH: usize = 512;
const DEPTH_HEIGHT: usize = 424;

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
    KinectV2,
}

impl Backend {
    fn label(self) -> &'static str {
        match self {
            Backend::None => "None (off)",
            Backend::KinectV2 => "Kinect v2",
        }
    }
}

// ============================================================ App

struct App {
    selected: Backend,
    active: Option<Active>,
    error: Option<String>,
    logs: Arc<Mutex<VecDeque<String>>>,
}

struct Active {
    backend: Backend,
    // `device` holds a USB session; keep `_ctx` alive until after `device`
    // tears down (struct fields drop in declaration order).
    device: Device,
    _ctx: Context,
    intrinsics: IrCameraParams,
    rgb_texture: Option<TextureHandle>,
    last_head: Option<HeadPixel>,
    /// First valid head pose since the device opened (or since the user hit
    /// "reset baseline"). The plugin would capture this on the first frame
    /// of a game and apply subsequent moves as deltas around it.
    baseline: Option<Baseline>,
}

#[derive(Debug, Clone, Copy)]
struct HeadPixel {
    /// Pixel coords inside the 512×424 depth grid.
    u: u32,
    v: u32,
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
        Self {
            selected: Backend::None,
            active: None,
            error: None,
            logs,
        }
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

        // Drop the old device first so libfreenect2 releases its USB session.
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
            }
        }
    }

    fn poll(&mut self, egui_ctx: &egui::Context) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        if let Some(rgb) = active.device.poll_rgb() {
            let img = bgrx_to_color_image(&rgb);
            match active.rgb_texture.as_mut() {
                Some(tex) => tex.set(img, egui::TextureOptions::LINEAR),
                None => {
                    active.rgb_texture =
                        Some(egui_ctx.load_texture("rgb", img, egui::TextureOptions::LINEAR));
                }
            }
        }
        if let Some(depth) = active.device.poll_depth() {
            active.last_head = find_head(&depth, &active.intrinsics);
            if active.baseline.is_none()
                && let Some(head) = active.last_head
            {
                let baseline = Baseline {
                    x_mm: head.x_mm,
                    y_mm: head.y_mm,
                    z_mm: head.depth_mm,
                };
                active.baseline = Some(baseline);
                info!(
                    x_mm = baseline.x_mm,
                    y_mm = baseline.y_mm,
                    z_mm = baseline.z_mm,
                    "baseline captured"
                );
            }
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, egui_ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.ensure_active();
        self.poll(egui_ctx);

        TopBottomPanel::top("toolbar").show(egui_ctx, |ui| {
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Input:");
                ComboBox::from_id_salt("backend")
                    .selected_text(self.selected.label())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.selected, Backend::None, Backend::None.label());
                        ui.selectable_value(
                            &mut self.selected,
                            Backend::KinectV2,
                            Backend::KinectV2.label(),
                        );
                    });
                ui.separator();
                if let Some(active) = self.active.as_ref() {
                    if let Some(head) = active.last_head {
                        ui.label(
                            RichText::new(format!(
                                "distance {:.0} mm  |  pixel ({}, {})  |  3D ({:.0}, {:.0}, {:.0}) mm",
                                head.depth_mm, head.u, head.v, head.x_mm, head.y_mm, head.depth_mm
                            ))
                            .monospace(),
                        );
                    } else {
                        ui.label(
                            RichText::new("waiting for head detection…").color(Color32::GRAY),
                        );
                    }
                } else if let Some(err) = &self.error {
                    ui.colored_label(Color32::LIGHT_RED, err);
                } else {
                    ui.label(RichText::new("no input selected").color(Color32::GRAY));
                }
            });
            ui.add_space(4.0);
        });

        TopBottomPanel::bottom("debug-panels")
            .resizable(true)
            .default_height(220.0)
            .min_height(80.0)
            .show(egui_ctx, |ui| {
                ui.add_space(4.0);
                ui.columns(2, |cols| {
                    // ----- Left column: tracing event log
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

                    // ----- Right column: what the plugin would push to VPX
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

        CentralPanel::default().show(egui_ctx, |ui| {
            let avail = ui.available_size();
            let aspect = RGB_WIDTH as f32 / RGB_HEIGHT as f32;
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

        // Continuous repaint so the camera frames keep flowing.
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
    // IR/RGB sensors aren't co-axial, so this is a few percent off the true
    // RGB pixel (parallax) — fine for "is the tracker on my head?" debugging.
    let u_norm = head.u as f32 / DEPTH_WIDTH as f32;
    let v_norm = head.v as f32 / DEPTH_HEIGHT as f32;
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
    }
}

fn open_kinect_v2() -> Result<Active, String> {
    let ctx = Context::new().map_err(|e| format!("Context::new: {e}"))?;
    let count = ctx.enumerate();
    if count <= 0 {
        return Err("no Kinect v2 found on USB".to_string());
    }
    let device = ctx
        .open_default()
        .map_err(|e| format!("open_default: {e}"))?;
    device
        .start_streams(true, true)
        .map_err(|e| format!("start_streams: {e}"))?;
    let intrinsics = device.ir_params();
    Ok(Active {
        backend: Backend::KinectV2,
        device,
        _ctx: ctx,
        intrinsics,
        rgb_texture: None,
        last_head: None,
        baseline: None,
    })
}

// ============================================================ VPU mapping
//
// Mirrors `crate::camera::mapping::pose_delta_to_view_delta` from the plugin.
// We don't depend on the `headtracking` cdylib here, so the constants and
// axis convention are duplicated — keep them in sync if either side changes.

const VPU_PER_MM: f64 = 50.0 / (25.4 * 1.0625);

fn pose_delta_to_view_delta_vpu(dx_mm: f32, dy_mm: f32, dz_mm: f32) -> (f32, f32, f32) {
    let to_vpu = |mm: f32| (f64::from(mm) * VPU_PER_MM) as f32;
    // Kinect Y points down (so head going up → -Y) and Z grows away from the
    // sensor (head approaching → -Z). VPX Camera-mode Y is "forward away from
    // the player" and Z is upward.
    (to_vpu(dx_mm), -to_vpu(dz_mm), -to_vpu(dy_mm))
}

// ============================================================ Image conversion

fn bgrx_to_color_image(frame: &RgbFrame) -> ColorImage {
    debug_assert_eq!(frame.width as usize, RGB_WIDTH);
    debug_assert_eq!(frame.height as usize, RGB_HEIGHT);
    let mut pixels = Vec::with_capacity(RGB_WIDTH * RGB_HEIGHT);
    for chunk in frame.data.chunks_exact(4) {
        // libfreenect2 ships pixels as B, G, R, X.
        pixels.push(Color32::from_rgb(chunk[2], chunk[1], chunk[0]));
    }
    ColorImage {
        size: [RGB_WIDTH, RGB_HEIGHT],
        pixels,
    }
}

// ============================================================ Head blob algo

fn find_head(frame: &DepthFrame, intr: &IrCameraParams) -> Option<HeadPixel> {
    let w = frame.width as i32;
    let h = frame.height as i32;
    if w <= 0 || h <= 0 {
        return None;
    }

    let valid = DEPTH_MIN_MM..=DEPTH_MAX_MM;
    let mut min_z = f32::INFINITY;
    let mut min_idx: i32 = -1;
    for (i, &z) in frame.data.iter().enumerate() {
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
            let z = frame.data[row + u as usize];
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
        depth_mm: (sz / n) as f32,
        x_mm: (sx / n) as f32,
        y_mm: (sy / n) as f32,
    })
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
