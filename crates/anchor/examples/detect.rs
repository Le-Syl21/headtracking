//! Quick check: run the anchor model on an image and print the keypoints +
//! derived geometry. `cargo run -p anchor --example detect -- <image>`

fn main() {
    let path = std::env::args().nth(1).expect("usage: detect <image>");
    let img = image::open(&path).expect("open image").to_rgb8();
    let (w, h) = img.dimensions();

    let mut det = anchor::AnchorDetector::new().expect("load model");
    let Some(d) = det.detect(img.as_raw(), w, h, anchor::PixelLayout::Rgb888) else {
        println!("no detection");
        return;
    };
    println!("score {:.3}", d.score);
    let names = [
        "player_L", "player_R", "screen_R", "screen_L", "bottom_L", "bottom_R",
    ];
    for (i, name) in names.iter().enumerate() {
        let (x, y) = d.keypoints[i];
        println!("  {name:9} ({x:7.1}, {y:7.1})  conf {:.2}", d.kconf[i]);
    }
    let g = d.geometry(w, h);
    println!("lockbar width  = {:.1} px", g.lockbar_width_px);
    println!(
        "lateral offset = {:+.1} px (from image centre)",
        g.lateral_offset_px
    );
    match g.depth_vp {
        Some((x, y)) => println!("depth vanishing point (sidebars→∞) = ({x:.0}, {y:.0})"),
        None => println!("depth vanishing point: parallel sidebars (none)"),
    }
}
