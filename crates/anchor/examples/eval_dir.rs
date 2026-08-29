//! Batch evaluation: run the embedded anchor model over every `*_raw.png`
//! in a directory and print one CSV line per image
//! (`file,score,lockbar_width_px,lateral_offset_px,vp`), `none` when the
//! model finds nothing. Summarise per backend prefix at the end.
//!
//! `cargo run -p anchor --example eval_dir -- <dir>`
//!
//! A/B different model weights by swapping `models/anchor.onnx` and
//! rebuilding — the detector embeds the file at compile time.

use std::collections::BTreeMap;

fn main() {
    let dir = std::env::args().nth(1).expect("usage: eval_dir <dir>");
    let mut det = anchor::AnchorDetector::new().expect("load model");

    // (detections, total, score sum) per backend prefix.
    let mut per_backend: BTreeMap<String, (u32, u32, f32)> = BTreeMap::new();

    let mut files: Vec<_> = std::fs::read_dir(&dir)
        .expect("read dir")
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.ends_with("_raw.png"))
        })
        .collect();
    files.sort();

    println!("file,score,lockbar_width_px,lateral_offset_px,vp");
    for path in files {
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        // `ht_<backend>_<stamp>_<id>_raw.png` → backend key.
        let backend = name
            .strip_prefix("ht_")
            .and_then(|r| r.split('_').next())
            .unwrap_or("?")
            .to_string();
        let img = match image::open(&path) {
            Ok(i) => i.to_rgb8(),
            Err(e) => {
                eprintln!("skip {name}: {e}");
                continue;
            }
        };
        let (w, h) = img.dimensions();
        let entry = per_backend.entry(backend).or_default();
        entry.1 += 1;
        match det.detect(img.as_raw(), w, h, anchor::PixelLayout::Rgb888) {
            Some(d) => {
                entry.0 += 1;
                entry.2 += d.score;
                let g = d.geometry(w, h);
                let vp = g
                    .depth_vp
                    .map_or_else(|| "none".to_string(), |(x, y)| format!("({x:.0};{y:.0})"));
                println!(
                    "{name},{:.3},{:.1},{:+.1},{vp}",
                    d.score, g.lockbar_width_px, g.lateral_offset_px
                );
            }
            None => println!("{name},none,,,"),
        }
    }

    eprintln!("--- summary (detections/total, mean score) ---");
    for (b, (hits, total, score_sum)) in &per_backend {
        let mean = if *hits > 0 {
            score_sum / *hits as f32
        } else {
            0.0
        };
        eprintln!("{b}: {hits}/{total} ({mean:.3})");
    }
}
