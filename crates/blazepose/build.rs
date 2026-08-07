//! Fetch the BlazePose ONNX models at build time (kept out of git — they're
//! ~28 MB). Skips download if already present, so local rebuilds and
//! pre-seeded checkouts don't hit the network.
use std::path::Path;

// URLs pin an exact HF revision, not `main`: upstream can force-push
// `main` and silently swap the model weights under us, while a revision
// URL is immutable — every build (and the CI model cache keyed on this
// file) gets byte-identical models. Verified sha256 at this revision:
//   pose_detection.onnx      72081da8481170bc6d8fafa716455ee210b61a8cefed84c67fcbbf889a4c38cf
//   pose_landmark_full.onnx  ae17ee8f076a5bbc28f65b939f46139c10f10c51ec4392a011e56d06d3f76c5d
const MODELS: &[(&str, &str)] = &[
    ("pose_detection.onnx",
     "https://huggingface.co/unity/inference-engine-blaze-pose/resolve/d96e13766db93eaaac6ad398d7b01f1c94c7ff6e/models/pose_detection.onnx?download=true"),
    ("pose_landmark_full.onnx",
     "https://huggingface.co/unity/inference-engine-blaze-pose/resolve/d96e13766db93eaaac6ad398d7b01f1c94c7ff6e/models/pose_landmarks_detector_full.onnx?download=true"),
];

/// Hugging Face rate-limits bursts (HTTP 429) — CI builds all matrix
/// targets in parallel and each starts from a fresh checkout, so a
/// single-shot download gets throttled regularly. Retry with a fixed
/// backoff instead of failing the whole build on the first 429.
fn download(name: &str, url: &str) -> Result<ureq::Response, String> {
    const BACKOFF_SECS: &[u64] = &[0, 10, 30, 60];
    let mut last_err = String::new();
    for (attempt, wait) in BACKOFF_SECS.iter().enumerate() {
        std::thread::sleep(std::time::Duration::from_secs(*wait));
        match ureq::get(url).call() {
            Ok(resp) => return Ok(resp),
            Err(e) => {
                last_err = e.to_string();
                println!(
                    "cargo:warning=blazepose: {name} attempt {} failed: {last_err}",
                    attempt + 1
                );
            }
        }
    }
    Err(last_err)
}

fn main() {
    let dir = Path::new("models");
    std::fs::create_dir_all(dir).expect("create models dir");
    for (name, url) in MODELS {
        let path = dir.join(name);
        if path.exists() {
            continue;
        }
        println!("cargo:warning=blazepose: downloading {name} …");
        let resp =
            download(name, url).unwrap_or_else(|e| panic!("download {name} after retries: {e}"));
        let mut reader = resp.into_reader();
        let mut f = std::fs::File::create(&path).expect("create model file");
        std::io::copy(&mut reader, &mut f).expect("write model");
    }
    println!("cargo:rerun-if-changed=build.rs");
}
