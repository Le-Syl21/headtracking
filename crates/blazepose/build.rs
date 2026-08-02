//! Fetch the BlazePose ONNX models at build time (kept out of git — they're
//! ~28 MB). Skips download if already present, so local rebuilds and
//! pre-seeded checkouts don't hit the network.
use std::path::Path;

const MODELS: &[(&str, &str)] = &[
    ("pose_detection.onnx",
     "https://huggingface.co/unity/inference-engine-blaze-pose/resolve/main/models/pose_detection.onnx?download=true"),
    ("pose_landmark_full.onnx",
     "https://huggingface.co/unity/inference-engine-blaze-pose/resolve/main/models/pose_landmarks_detector_full.onnx?download=true"),
];

fn main() {
    let dir = Path::new("models");
    std::fs::create_dir_all(dir).expect("create models dir");
    for (name, url) in MODELS {
        let path = dir.join(name);
        if path.exists() {
            continue;
        }
        println!("cargo:warning=blazepose: downloading {name} …");
        let resp = ureq::get(url)
            .call()
            .unwrap_or_else(|e| panic!("download {name}: {e}"));
        let mut reader = resp.into_reader();
        let mut f = std::fs::File::create(&path).expect("create model file");
        std::io::copy(&mut reader, &mut f).expect("write model");
    }
    println!("cargo:rerun-if-changed=build.rs");
}
