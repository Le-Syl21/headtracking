//! Decode a PNG (typically a `headtracking-demo` screenshot) and run
//! `detect_lockbar_rgb` against it, dumping the result + debug logs.
//! With `--out <PNG>` it also rasterises the detected quad onto a
//! copy of the input so we can SEE where the algo locked on.
//!
//! `cargo run -p lockbar-replay -- /tmp/v18-v2.png --out /tmp/dbg.png`
//! Set `HEADTRACKING_LOG=lockbar=debug` to see every rejection gate.

use std::env;
use std::fs::File;
use std::path::PathBuf;

use headtracking::calibration::{LockbarQuadRgb, LockbarRgbParams, detect_lockbar_rgb};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let env_filter = tracing_subscriber::EnvFilter::try_from_env("HEADTRACKING_LOG")
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("lockbar=debug,info"));
    tracing_subscriber::fmt().with_env_filter(env_filter).init();

    let path = env::args()
        .nth(1)
        .ok_or("usage: lockbar-replay <PNG-PATH | --synthetic> [tweaks]")?;

    if path == "--synthetic" {
        let w = 200u32;
        let h = 200u32;
        let mut out = vec![240u8; (w * h * 3) as usize];
        for v in 170..=185u32 {
            for u in 30..=170u32 {
                let i = ((v * w + u) * 3) as usize;
                out[i] = 20;
                out[i + 1] = 20;
                out[i + 2] = 20;
            }
        }
        let params = LockbarRgbParams::default();
        println!("synthetic 200×200 band rows 170-185 cols 30-170");
        let q = detect_lockbar_rgb(&out, w, h, &params);
        println!("result: {q:#?}");
        return Ok(());
    }

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
    let mut params = LockbarRgbParams::default();
    let mut out_path: Option<PathBuf> = None;
    let mut it = env::args().skip(2);
    while let Some(k) = it.next() {
        let v = it.next().ok_or_else(|| format!("missing value for {k}"))?;
        match k.as_str() {
            "--bottom" => params.bottom_fraction = v.parse()?,
            "--strength" => params.min_edge_strength = v.parse()?,
            "--min-w" => params.min_width_fraction = v.parse()?,
            "--max-w" => params.max_width_fraction = v.parse()?,
            "--min-sep" => params.min_separation = v.parse()?,
            "--max-sep" => params.max_separation = v.parse()?,
            "--min-aspect" => params.min_aspect_ratio = v.parse()?,
            "--max-aspect" => params.max_aspect_ratio = v.parse()?,
            "--max-slope" => params.max_slope_diff_deg = v.parse()?,
            "--dev" => params.max_row_deviation = v.parse()?,
            "--min-top-row" => params.min_top_edge_row_fraction = v.parse()?,
            "--out" => out_path = Some(PathBuf::from(v)),
            other => return Err(format!("unknown tweak '{other}'").into()),
        }
    }
    println!("params: {params:?}");

    let q = detect_lockbar_rgb(&buf, width, height, &params);
    println!("\nresult: {q:#?}");

    if let Some(p) = out_path {
        let mut painted = buf.clone();
        // Scatter-plot all per-column max-gradient rows as orange dots
        // — see where the algo's votes actually land. This is the
        // diagnostic that lets me triage "wrong cluster selected" vs
        // "algo never saw the lockbar at all".
        bake_gradient_scatter(&mut painted, width, height, &buf, &params);
        if let Some(quad) = q.as_ref() {
            bake_quad(&mut painted, width, height, quad);
        }
        write_png(&p, width, height, &painted)?;
        println!("wrote debug overlay → {}", p.display());
    }

    Ok(())
}

/// Find ALL local-maximum vertical-gradient rows above the strength
/// threshold per column in the same ROI `detect_lockbar_rgb` uses,
/// keep the top-K by strength, and paint each as a colored dot
/// (orange = #1, yellow = #2, green = #3, blue = #4). Shows the
/// row distribution that `fit_all_lines` consumes.
fn bake_gradient_scatter(
    painted: &mut [u8],
    width: u32,
    height: u32,
    rgb: &[u8],
    params: &LockbarRgbParams,
) {
    let w = width as usize;
    let h = height as usize;
    if w == 0 || h < 4 {
        return;
    }
    let row_start = ((params.bottom_fraction * height as f32) as usize).min(h.saturating_sub(2));
    let luma = |c: usize, r: usize| -> i32 {
        let i = (r * w + c) * 3;
        let g = i32::from(rgb[i + 1]) * 183;
        let r_ = i32::from(rgb[i]) * 54;
        let b = i32::from(rgb[i + 2]) * 19;
        (r_ + g + b) >> 8
    };
    const COLORS: [[u8; 3]; 4] = [
        [0xff, 0x88, 0x00], // orange — strongest
        [0xff, 0xee, 0x00], // yellow — 2nd
        [0x88, 0xff, 0x00], // green  — 3rd
        [0x00, 0xaa, 0xff], // blue   — 4th
    ];
    for c in 0..w {
        let mut peaks: Vec<(i32, usize)> = Vec::new();
        let mut prev_g: i32 = 0;
        let mut rising = false;
        for r in row_start..(h - 1) {
            let g = (luma(c, r + 1) - luma(c, r)).abs();
            if rising && g < prev_g && prev_g >= params.min_edge_strength as i32 {
                peaks.push((prev_g, r - 1));
                rising = false;
            }
            if g > prev_g {
                rising = true;
            }
            prev_g = g;
        }
        if rising && prev_g >= params.min_edge_strength as i32 {
            peaks.push((prev_g, h - 2));
        }
        peaks.sort_unstable_by_key(|&(g, _)| std::cmp::Reverse(g));
        peaks.truncate(COLORS.len());
        for (rank, (_, row)) in peaks.iter().enumerate() {
            let color = COLORS[rank];
            for ox in -1..=1i32 {
                for oy in -1..=1i32 {
                    put_pixel(
                        painted,
                        width,
                        height,
                        c as i32 + ox,
                        *row as i32 + oy,
                        color,
                    );
                }
            }
        }
    }
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

fn bake_quad(buf: &mut [u8], w: u32, h: u32, q: &LockbarQuadRgb) {
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
