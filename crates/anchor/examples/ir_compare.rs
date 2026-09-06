//! Compare the Rust IR preparation against the corpus rendered with OpenCV.
//!
//! The model is trained on images produced by the Python pipeline; the runtime
//! produces them with `anchor::prepare_ir`. If the two drift, the model is fed a
//! distribution it never saw -- the failure the whole split exists to avoid. So
//! this measures the difference directly.
//!
//! `cargo run -p anchor --example ir_compare -- <raw16 dir> <opencv-rendered dir>`

fn main() {
    let mut args = std::env::args().skip(1);
    let raw_dir = args
        .next()
        .expect("usage: ir_compare <raw dir> <rendered dir>");
    let ref_dir = args
        .next()
        .expect("usage: ir_compare <raw dir> <rendered dir>");

    let mut files: Vec<_> = std::fs::read_dir(&raw_dir)
        .expect("read raw dir")
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .is_some_and(|n| n.to_string_lossy().ends_with("_ir.png"))
        })
        .collect();
    files.sort();

    let (mut n, mut sum_mae, mut worst) = (0u32, 0.0f64, 0.0f64);
    for path in files {
        let stem = path
            .file_name()
            .unwrap()
            .to_string_lossy()
            .replace("_ir.png", "");
        let refp = std::path::Path::new(&ref_dir).join(format!("{stem}_irview.png"));
        let (Ok(raw), Ok(rf)) = (image::open(&path), image::open(&refp)) else {
            continue;
        };
        let raw = raw.to_luma16();
        let rf = rf.to_luma8();
        let (w, h) = (raw.width(), raw.height());
        let sensor = if stem.contains("kinect-v1") {
            anchor::IrSensor::KinectV1
        } else {
            anchor::IrSensor::KinectV2
        };
        let mine = anchor::prepare_ir(raw.as_raw(), w, h, sensor);
        if mine.len() != rf.as_raw().len() {
            continue;
        }
        let mae: f64 = mine
            .iter()
            .zip(rf.as_raw())
            .map(|(a, b)| f64::from(a.abs_diff(*b)))
            .sum::<f64>()
            / mine.len() as f64;
        sum_mae += mae;
        worst = worst.max(mae);
        n += 1;
    }
    println!("{n} images compared");
    println!(
        "mean absolute difference : {:.2} / 255",
        sum_mae / f64::from(n.max(1))
    );
    println!("worst image              : {worst:.2} / 255");
}
