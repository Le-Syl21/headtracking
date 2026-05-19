//! Decode a PNG (typically a `headtracking-demo` screenshot) and run
//! the YOLOv11n-OBB lockbar detector against it, dumping the result
//! plus an optional debug overlay. The legacy HSV/gradient detector
//! that used to back this tool was retired in v0.0.21.
//!
//! `cargo run -p lockbar-replay -- /tmp/v18-v2.png --out /tmp/dbg.png`
//! Use `--score <f>` to override the default 0.25 confidence
//! threshold. Set `HEADTRACKING_LOG=info` to see init / inference logs.

use std::env;
use std::fs::File;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let env_filter = tracing_subscriber::EnvFilter::try_from_env("HEADTRACKING_LOG")
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));
    tracing_subscriber::fmt().with_env_filter(env_filter).init();

    let path = env::args()
        .nth(1)
        .ok_or("usage: lockbar-replay <PNG-PATH> [--out PNG] [--score F]")?;

    let decoder = png::Decoder::new(File::open(&path)?);
    let mut reader = decoder.read_info()?;
    let mut buf = vec![0; reader.output_buffer_size()];
    let info = reader.next_frame(&mut buf)?;
    let width = info.width;
    let height = info.height;

    // Demo screenshots are saved as 8-bit RGB. Reject anything else
    // explicitly so we don't silently produce garbage gradients.
    if info.color_type != png::ColorType::Rgb || info.bit_depth != png::BitDepth::Eight {
        return Err(format!(
            "expected 8-bit RGB PNG, got {:?} / {:?}",
            info.color_type, info.bit_depth
        )
        .into());
    }

    println!("loaded {path}: {width}×{height} RGB888");
    let mut out_path: Option<PathBuf> = None;
    let mut score_thresh: Option<f32> = None;
    let mut it = env::args().skip(2);
    while let Some(k) = it.next() {
        let v = it.next().ok_or_else(|| format!("missing value for {k}"))?;
        match k.as_str() {
            "--out" => out_path = Some(PathBuf::from(v)),
            "--score" => score_thresh = Some(v.parse()?),
            other => return Err(format!("unknown flag '{other}'").into()),
        }
    }

    let mut detector = lockbar_onnx::LockbarDetector::new()?;
    if let Some(t) = score_thresh {
        detector.set_score_threshold(t);
    }
    let start = std::time::Instant::now();
    let obb = detector.detect(&buf, width, height);
    let elapsed = start.elapsed();
    println!("inference took {:.1} ms", elapsed.as_secs_f32() * 1000.0);
    println!("result: {obb:#?}");
    // Machine-readable summary line for batch scripts. Parsed by
    // `research/scripts/batch_detect.py`.
    match obb.as_ref() {
        Some(o) => {
            println!(
                "SUMMARY conf={:.4} slope_deg={:.3} thickness_px={:.2} \
                 corners=[[{:.1},{:.1}],[{:.1},{:.1}],[{:.1},{:.1}],[{:.1},{:.1}]]",
                o.confidence,
                o.slope_deg,
                o.thickness_px,
                o.corners[0].0,
                o.corners[0].1,
                o.corners[1].0,
                o.corners[1].1,
                o.corners[2].0,
                o.corners[2].1,
                o.corners[3].0,
                o.corners[3].1
            );
        }
        None => println!("SUMMARY conf=0.0000 no_detection=true"),
    }

    if let Some(p) = out_path {
        let mut painted = buf.clone();
        if let Some(o) = obb.as_ref() {
            bake_obb(&mut painted, width, height, o);
        }
        write_png(&p, width, height, &painted)?;
        println!("wrote overlay → {}", p.display());
    }

    Ok(())
}

const CYAN: [u8; 3] = [0x00, 0xe5, 0xff];

fn put_pixel(buf: &mut [u8], w: u32, h: u32, x: i32, y: i32, c: [u8; 3]) {
    if x < 0 || y < 0 || (x as u32) >= w || (y as u32) >= h {
        return;
    }
    let i = ((y as u32 * w + x as u32) as usize) * 3;
    buf[i] = c[0];
    buf[i + 1] = c[1];
    buf[i + 2] = c[2];
}

fn draw_line(buf: &mut [u8], w: u32, h: u32, (mut x0, mut y0): (i32, i32), (x1, y1): (i32, i32)) {
    let dx = (x1 - x0).abs();
    let dy = -(y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    loop {
        for ox in -1..=1 {
            for oy in -1..=1 {
                put_pixel(buf, w, h, x0 + ox, y0 + oy, CYAN);
            }
        }
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

fn bake_obb(buf: &mut [u8], w: u32, h: u32, q: &lockbar_onnx::LockbarObb) {
    for i in 0..4 {
        let a = q.corners[i];
        let b = q.corners[(i + 1) % 4];
        draw_line(
            buf,
            w,
            h,
            (a.0 as i32, a.1 as i32),
            (b.0 as i32, b.1 as i32),
        );
    }
}

fn write_png(
    path: &std::path::Path,
    w: u32,
    h: u32,
    rgb: &[u8],
) -> Result<(), Box<dyn std::error::Error>> {
    let f = File::create(path)?;
    let writer = std::io::BufWriter::new(f);
    let mut enc = png::Encoder::new(writer, w, h);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    let mut wr = enc.write_header()?;
    wr.write_image_data(rgb)?;
    Ok(())
}
