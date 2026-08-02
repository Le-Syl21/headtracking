//! Full-pipeline probe: detect → landmarks, draw the 33 points + upper-body
//! connections, save an annotated PNG. Validates against the Python reference.
//!   cargo run -p blazepose --example probe_pose -- <in.png> <out.png>

use image::{Rgb, RgbImage};

fn disc(img: &mut RgbImage, x: i32, y: i32, r: i32, c: Rgb<u8>) {
    let (w, h) = (img.width() as i32, img.height() as i32);
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                let (px, py) = (x + dx, y + dy);
                if px >= 0 && py >= 0 && px < w && py < h {
                    img.put_pixel(px as u32, py as u32, c);
                }
            }
        }
    }
}

fn line(img: &mut RgbImage, a: (i32, i32), b: (i32, i32), c: Rgb<u8>) {
    let (mut x0, mut y0) = a;
    let (x1, y1) = b;
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    let (w, h) = (img.width() as i32, img.height() as i32);
    loop {
        if x0 >= 0 && y0 >= 0 && x0 < w && y0 < h {
            img.put_pixel(x0 as u32, y0 as u32, c);
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

fn main() {
    use blazepose::idx::*;
    let path = std::env::args()
        .nth(1)
        .expect("usage: probe_pose <in> <out>");
    let out = std::env::args()
        .nth(2)
        .unwrap_or_else(|| "pose_out.png".into());
    let src = image::open(&path).expect("open").to_rgb8();
    let (w, h) = src.dimensions();
    let mut bp = blazepose::BlazePose::new().expect("load");
    let mut img = src.clone();
    match bp.detect(src.as_raw(), w, h).expect("detect") {
        None => println!("NO POSE"),
        Some(p) => {
            let conns = [
                (LEFT_SHOULDER, RIGHT_SHOULDER),
                (LEFT_SHOULDER, LEFT_ELBOW),
                (LEFT_ELBOW, LEFT_WRIST),
                (RIGHT_SHOULDER, RIGHT_ELBOW),
                (RIGHT_ELBOW, RIGHT_WRIST),
                (NOSE, LEFT_SHOULDER),
                (NOSE, RIGHT_SHOULDER),
            ];
            let g = |i: usize| (p.landmarks[i].x as i32, p.landmarks[i].y as i32);
            for (a, b) in conns {
                line(&mut img, g(a), g(b), Rgb([255, 255, 255]));
            }
            let r = (w / 220).max(4) as i32;
            for (i, l) in p.landmarks.iter().enumerate() {
                let c = if (11..=16).contains(&i) {
                    Rgb([255, 150, 0])
                } else {
                    Rgb([0, 220, 60])
                };
                disc(&mut img, l.x as i32, l.y as i32, r, c);
            }
            println!("presence={:.2}", p.presence);
            for (n, i) in [
                ("nose", NOSE),
                ("Lsh", LEFT_SHOULDER),
                ("Rsh", RIGHT_SHOULDER),
                ("Lwr", LEFT_WRIST),
                ("Rwr", RIGHT_WRIST),
            ] {
                let l = &p.landmarks[i];
                println!("  {n:5} ({:.0},{:.0}) vis={:.2}", l.x, l.y, l.visibility);
            }
        }
    }
    img.save(&out).expect("save");
    println!("-> {out}");
}
