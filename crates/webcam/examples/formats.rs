//! List every camera SDL sees and, for each, all capture modes it advertises
//! (native pixel format, resolution, fps) plus the max fps per resolution.
//!
//! Run on the machine that has the cameras attached:
//!   cargo run -p webcam --example formats --release

fn main() {
    let cams = match webcam::list() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("webcam::list failed: {e}");
            std::process::exit(1);
        }
    };
    if cams.is_empty() {
        println!("no cameras enumerated by SDL");
        return;
    }
    for cam in &cams {
        println!("==================================================");
        println!("id {}  \"{}\"", cam.id, cam.name);
        match webcam::supported_formats(cam.id) {
            Ok(fmts) if !fmts.is_empty() => {
                // Group by resolution to surface the max fps at a glance.
                let mut best: std::collections::BTreeMap<(u32, u32), f32> =
                    std::collections::BTreeMap::new();
                for f in &fmts {
                    let e = best.entry((f.width, f.height)).or_insert(0.0);
                    if f.fps > *e {
                        *e = f.fps;
                    }
                    println!(
                        "   {:>5}x{:<5} {:>7.2} fps   {}",
                        f.width, f.height, f.fps, f.pixel_format
                    );
                }
                println!("   --- max fps per resolution ---");
                for ((w, h), fps) in best {
                    println!("   {w:>5}x{h:<5} -> {fps:.2} fps");
                }
            }
            Ok(_) => println!("   (SDL advertised no formats for this camera)"),
            Err(e) => println!("   supported_formats failed: {e}"),
        }
    }
}
