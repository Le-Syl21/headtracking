//! Validation harness for the pure-Rust U-seg inference: run the tract
//! pipeline on a folder of frames and write PNG overlays (mask tint +
//! box + confidence), so we can eyeball them against Ultralytics'
//! `val_batch*_pred.jpg`.
//!
//! Usage:
//!   cargo run -p u-onnx --example dump_masks -- <img_dir> <out_dir>
//!
//! Defaults to the val split / a sibling output dir when args are
//! omitted.

use std::path::{Path, PathBuf};

use image::{Rgb, RgbImage};
use u_onnx::UDetector;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let img_dir = args
        .next()
        .unwrap_or_else(|| "output/u-seg/dataset/images/val".to_string());
    let out_dir = args
        .next()
        .unwrap_or_else(|| "output/u-seg/rust_masks".to_string());
    std::fs::create_dir_all(&out_dir)?;

    let det = UDetector::new()?;
    let thr = det.mask_threshold();

    let mut entries: Vec<PathBuf> = std::fs::read_dir(&img_dir)?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| {
            matches!(
                p.extension().and_then(|s| s.to_str()),
                Some("jpg" | "jpeg" | "png" | "webp")
            )
        })
        .collect();
    entries.sort();
    println!("{} frames in {img_dir}", entries.len());

    let (mut n_with, mut n_without) = (0usize, 0usize);
    for path in &entries {
        let img = image::open(path)?.to_rgb8();
        let (w, h) = img.dimensions();
        let dets = det.detect(img.as_raw(), w, h);
        if dets.is_empty() {
            n_without += 1;
        } else {
            n_with += 1;
        }

        let mut canvas = img.clone();
        for d in &dets {
            tint_mask(&mut canvas, d, thr);
            draw_box(&mut canvas, d.bbox, Rgb([0, 200, 255]));
        }
        let confs: Vec<String> = dets
            .iter()
            .map(|d| format!("{:.2}", d.confidence))
            .collect();
        let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("frame");
        let out = Path::new(&out_dir).join(format!("{stem}.png"));
        canvas.save(&out)?;
        println!("{stem}: {} U  conf=[{}]", dets.len(), confs.join(", "));
    }
    println!(
        "\n{n_with} frames with a U, {n_without} without (of {})",
        entries.len()
    );
    Ok(())
}

/// Blend a cyan tint into the pixels the mask claims.
fn tint_mask(img: &mut RgbImage, d: &u_onnx::UDetection, thr: f32) {
    let (w, h) = img.dimensions();
    for y in 0..h {
        for x in 0..w {
            if d.mask_prob_at(x as f32, y as f32) >= thr {
                let p = img.get_pixel_mut(x, y);
                p.0[0] = (p.0[0] as u16 * 2 / 5) as u8;
                p.0[1] = (p.0[1] as u16 * 3 / 5 + 80) as u8;
                p.0[2] = (p.0[2] as u16 * 3 / 5 + 100) as u8;
            }
        }
    }
}

fn draw_box(img: &mut RgbImage, (x0, y0, x1, y1): (f32, f32, f32, f32), color: Rgb<u8>) {
    let (w, h) = img.dimensions();
    let clampx = |v: f32| v.round().clamp(0.0, (w - 1) as f32) as u32;
    let clampy = |v: f32| v.round().clamp(0.0, (h - 1) as f32) as u32;
    let (xa, xb) = (clampx(x0.min(x1)), clampx(x0.max(x1)));
    let (ya, yb) = (clampy(y0.min(y1)), clampy(y0.max(y1)));
    for x in xa..=xb {
        img.put_pixel(x, ya, color);
        img.put_pixel(x, yb, color);
    }
    for y in ya..=yb {
        img.put_pixel(xa, y, color);
        img.put_pixel(xb, y, color);
    }
}
