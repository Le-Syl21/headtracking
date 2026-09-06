//! Pre-fill the annotator with the current model's predictions.
//!
//! Runs the detector over a directory and writes `anchor-lines.json` in the
//! annotator's own format, so a human corrects lines rather than drawing them
//! from nothing. The 6 keypoints invert exactly back to the 4 lines: the four
//! corners are the line intersections, and the two bottom points are where
//! each rail meets the last image row.
//!
//! This is a draft, not an annotation. The model detects reliably but places
//! imprecisely -- which is the whole reason for re-annotating.
//!
//! `cargo run -p anchor --example predraft -- <dir> > anchor-lines.json`

use std::io::Write;

fn line_json(a: (f32, f32), b: (f32, f32), w: f32, h: f32) -> String {
    // Extend through the two points to the image border, as the tool does:
    // a long baseline is what makes the angle precise.
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let mut ts: Vec<f32> = Vec::new();
    if dx.abs() > 1e-6 {
        ts.push((0.0 - a.0) / dx);
        ts.push((w - a.0) / dx);
    }
    if dy.abs() > 1e-6 {
        ts.push((0.0 - a.1) / dy);
        ts.push((h - a.1) / dy);
    }
    ts.sort_by(|p, q| p.partial_cmp(q).unwrap());
    let inside = |t: f32| {
        let (x, y) = (a.0 + dx * t, a.1 + dy * t);
        x >= -0.5 && x <= w + 0.5 && y >= -0.5 && y <= h + 0.5
    };
    let hits: Vec<f32> = ts.into_iter().filter(|t| inside(*t)).collect();
    let (t0, t1) = (*hits.first().unwrap_or(&0.0), *hits.last().unwrap_or(&1.0));
    let p1 = (a.0 + dx * t0, a.1 + dy * t0);
    let p2 = (a.0 + dx * t1, a.1 + dy * t1);
    let angle = dy.atan2(dx).to_degrees();
    format!(
        "{{\"p1\":{{\"x\":{:.2},\"y\":{:.2}}},\"p2\":{{\"x\":{:.2},\"y\":{:.2}}},\
         \"pivot\":{{\"x\":{:.2},\"y\":{:.2}}},\"angle_deg\":{:.3}}}",
        p1.0, p1.1, p2.0, p2.1, a.0, a.1, angle
    )
}

fn main() {
    let dir = std::env::args().nth(1).expect("usage: predraft <dir>");
    let mut det = anchor::AnchorDetector::new(
        // second argument: "rgb" for the colour model, infrared otherwise
        match std::env::args().nth(2).as_deref() {
            Some("rgb") => anchor::Stream::Colour,
            _ => anchor::Stream::Infrared,
        },
    )
    .expect("load model");

    let mut files: Vec<_> = std::fs::read_dir(&dir)
        .expect("read dir")
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "png"))
        .collect();
    files.sort();

    let out = std::io::stdout();
    let mut out = out.lock();
    writeln!(out, "{{\"schema\":\"anchor-lines-v1\",\"images\":{{").unwrap();

    let mut first = true;
    let (mut done, mut missed) = (0u32, 0u32);
    for path in &files {
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        let Ok(img) = image::open(path) else { continue };
        let img = img.to_rgb8();
        let (w, h) = img.dimensions();
        let Some(d) = det.detect(img.as_raw(), w, h, anchor::PixelLayout::Rgb888) else {
            missed += 1;
            eprintln!("no detection: {name}");
            continue;
        };
        // 0 player_left  1 player_right  2 screen_right
        // 3 screen_left  4 bottom_left   5 bottom_right
        let k = d.keypoints;
        let (wf, hf) = (w as f32, h as f32);
        if !first {
            writeln!(out, ",").unwrap();
        }
        first = false;
        write!(
            out,
            "\"{name}\":{{\"width\":{w},\"height\":{h},\"score\":{:.3},\"annotations\":{{\
             \"sideleft\":{},\"sideright\":{},\"lockbar_player\":{},\"lockbar_screen\":{}}}}}",
            d.score,
            line_json(k[0], k[4], wf, hf),
            line_json(k[1], k[5], wf, hf),
            line_json(k[0], k[1], wf, hf),
            line_json(k[3], k[2], wf, hf),
        )
        .unwrap();
        done += 1;
    }
    writeln!(out, "\n}}}}").unwrap();
    eprintln!("{done} images pre-filled, {missed} without a detection");
}
