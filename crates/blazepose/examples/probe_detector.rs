//! Scaffolding probe: run the pose detector and print the best person
//! detection (box + keypoints, in frame pixels) to validate the decode.
//!   cargo run -p blazepose --example probe_detector -- <image>

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: probe_detector <image>");
    let img = image::open(&path).expect("open image").to_rgb8();
    let (w, h) = img.dimensions();
    let mut bp = blazepose::BlazePose::new().expect("load models");
    println!("image {w}x{h}");
    match bp.detect_person(img.as_raw(), w, h, 0.5).expect("run") {
        None => println!("NO PERSON"),
        Some(d) => {
            println!(
                "person score={:.3}  box=({:.0},{:.0}) {:.0}x{:.0}  (frac cx={:.2} cy={:.2})",
                d.score,
                d.cx,
                d.cy,
                d.w,
                d.h,
                d.cx / w as f32,
                d.cy / h as f32
            );
            for (k, p) in d.keypoints.iter().enumerate() {
                println!("  kp{k}: ({:.0},{:.0})", p[0], p[1]);
            }
        }
    }
}
