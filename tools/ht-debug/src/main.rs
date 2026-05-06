//! `ht-debug`: open a Kinect v2 outside VPX, stream RGB + depth, run the
//! head-tracking blob algorithm, and display the result in a window with a
//! crosshair on the detected head and the live distance in the title bar.
//!
//! Usage: `cargo run --release -p ht-debug` (Kinect v2 plugged in).
//! Press `Esc` or close the window to quit.

use std::time::{Duration, Instant};

use freenect2::{Context, DepthFrame, IrCameraParams, RgbFrame};
use minifb::{Key, ScaleMode, Window, WindowOptions};
use tracing::{error, info, warn};

const RGB_WIDTH: usize = 1920;
const RGB_HEIGHT: usize = 1080;
const DEPTH_WIDTH: usize = 512;
const DEPTH_HEIGHT: usize = 424;
// Display the RGB feed downscaled 2× so it fits typical monitors.
const DISPLAY_WIDTH: usize = RGB_WIDTH / 2;
const DISPLAY_HEIGHT: usize = RGB_HEIGHT / 2;

const DEPTH_MIN_MM: f32 = 500.0;
const DEPTH_MAX_MM: f32 = 2_500.0;
const WINDOW_HALF: i32 = 25;
const MIN_VALID_PIXELS: u32 = 100;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_env("HEADTRACKING_LOG")
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let ctx = Context::new()?;
    let count = ctx.enumerate();
    if count <= 0 {
        error!("no Kinect v2 found on USB");
        return Err("no Kinect v2 connected".into());
    }
    let device = ctx.open_default()?;
    device.start_streams(true, true)?;
    let intrinsics = device.ir_params();
    info!(
        fx = intrinsics.fx,
        fy = intrinsics.fy,
        cx = intrinsics.cx,
        cy = intrinsics.cy,
        "Kinect v2 streaming RGB + depth"
    );

    let mut window = Window::new(
        "ht-debug — Kinect v2 (Esc to quit)",
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
        WindowOptions {
            resize: true,
            scale_mode: ScaleMode::AspectRatioStretch,
            ..WindowOptions::default()
        },
    )?;
    window.set_target_fps(60);

    let mut framebuffer = vec![0u32; DISPLAY_WIDTH * DISPLAY_HEIGHT];
    let mut latest_rgb: Option<RgbFrame> = None;
    let mut latest_head: Option<HeadPixel> = None;
    let mut last_log = Instant::now();

    while window.is_open() && !window.is_key_down(Key::Escape) {
        if let Some(rgb) = device.poll_rgb() {
            latest_rgb = Some(rgb);
        }
        if let Some(depth) = device.poll_depth() {
            latest_head = find_head(&depth, &intrinsics);
        }

        if let Some(rgb) = latest_rgb.as_ref() {
            paint_bgrx_downscaled(&mut framebuffer, rgb);
            if let Some(head) = latest_head {
                let (u_rgb, v_rgb) = depth_pixel_to_display(head.u, head.v);
                draw_crosshair(&mut framebuffer, u_rgb, v_rgb);
                window.set_title(&format!(
                    "ht-debug — distance: {:.0} mm  |  pixel ({}, {})  |  Esc to quit",
                    head.depth_mm, head.u, head.v
                ));
            } else {
                window.set_title("ht-debug — no head detected (Esc to quit)");
            }
            if last_log.elapsed() >= Duration::from_secs(2) {
                if let Some(head) = latest_head {
                    info!(
                        x_mm = head.x_mm,
                        y_mm = head.y_mm,
                        z_mm = head.depth_mm,
                        "head pose"
                    );
                }
                last_log = Instant::now();
            }
        }

        if let Err(e) = window.update_with_buffer(&framebuffer, DISPLAY_WIDTH, DISPLAY_HEIGHT) {
            warn!(?e, "minifb update_with_buffer failed");
        }
    }

    device.stop()?;
    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct HeadPixel {
    /// Pixel coords inside the 512×424 depth grid.
    u: u32,
    v: u32,
    /// Centroid depth (mm).
    depth_mm: f32,
    /// Centroid 3D position in the IR-camera frame.
    x_mm: f32,
    y_mm: f32,
}

fn find_head(frame: &DepthFrame, intr: &IrCameraParams) -> Option<HeadPixel> {
    let w = frame.width as i32;
    let h = frame.height as i32;
    if w <= 0 || h <= 0 {
        return None;
    }

    // Pass 1: closest valid pixel.
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

    // Pass 2: 50×50 window centroid in [min_z, min_z + 150 mm].
    let z_max = min_z + 150.0;
    let (mut sx, mut sy, mut sz) = (0.0_f64, 0.0_f64, 0.0_f64);
    let mut count: u32 = 0;
    let u0 = (cu - WINDOW_HALF).max(0);
    let u1 = (cu + WINDOW_HALF).min(w - 1);
    let v0 = (cv - WINDOW_HALF).max(0);
    let v1 = (cv + WINDOW_HALF).min(h - 1);
    let (mut wsum_u, mut wsum_v) = (0.0_f64, 0.0_f64);
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

/// Linear scale from depth-frame pixel coordinates to display pixel coords.
/// The Kinect IR and color sensors don't share an optical axis, so this is
/// a few percent off the true RGB pixel (parallax) — fine for "is it
/// detecting roughly the right area?" debugging.
fn depth_pixel_to_display(u_d: u32, v_d: u32) -> (i32, i32) {
    let u = (u_d as f32 / DEPTH_WIDTH as f32 * DISPLAY_WIDTH as f32) as i32;
    let v = (v_d as f32 / DEPTH_HEIGHT as f32 * DISPLAY_HEIGHT as f32) as i32;
    (u, v)
}

/// Convert the BGRX 1920×1080 frame into 0xAARRGGBB pixels for minifb,
/// downscaled 2× by skipping every other pixel and row.
fn paint_bgrx_downscaled(out: &mut [u32], frame: &RgbFrame) {
    debug_assert_eq!(frame.width as usize, RGB_WIDTH);
    debug_assert_eq!(frame.height as usize, RGB_HEIGHT);
    debug_assert_eq!(out.len(), DISPLAY_WIDTH * DISPLAY_HEIGHT);
    for y in 0..DISPLAY_HEIGHT {
        let src_row = y * 2 * RGB_WIDTH * 4;
        let dst_row = y * DISPLAY_WIDTH;
        for x in 0..DISPLAY_WIDTH {
            let i = src_row + x * 2 * 4;
            let b = frame.data[i];
            let g = frame.data[i + 1];
            let r = frame.data[i + 2];
            out[dst_row + x] =
                0xff00_0000 | (u32::from(r) << 16) | (u32::from(g) << 8) | u32::from(b);
        }
    }
}

const CROSS_RADIUS: i32 = 18;
const RING_RADIUS: i32 = 28;
const RING_OUTER: i32 = 30;

fn draw_crosshair(buf: &mut [u32], cx: i32, cy: i32) {
    // Yellow cross + green ring so it's visible on any background.
    let yellow = 0xffff_ee00u32;
    let green = 0xff00_ff66u32;
    for d in -CROSS_RADIUS..=CROSS_RADIUS {
        plot(buf, cx + d, cy, yellow);
        plot(buf, cx, cy + d, yellow);
    }
    for theta in 0..360 {
        let rad = (theta as f32).to_radians();
        let x = cx + (rad.cos() * RING_RADIUS as f32) as i32;
        let y = cy + (rad.sin() * RING_RADIUS as f32) as i32;
        plot(buf, x, y, green);
        let x = cx + (rad.cos() * RING_OUTER as f32) as i32;
        let y = cy + (rad.sin() * RING_OUTER as f32) as i32;
        plot(buf, x, y, green);
    }
}

#[inline]
fn plot(buf: &mut [u32], x: i32, y: i32, color: u32) {
    if x < 0 || y < 0 {
        return;
    }
    let (x, y) = (x as usize, y as usize);
    if x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT {
        return;
    }
    buf[y * DISPLAY_WIDTH + x] = color;
}
