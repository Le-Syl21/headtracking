//! Throwaway probe: does a stock YOLOv8-detect ONNX load + run in `tract`?
//! This validates the architecture for the face→head switch: a head-only
//! detector is the SAME graph, only the output channel count changes
//! (`[1,84,8400]` for 80 COCO classes → `[1,5,8400]` for head-only).
//!   cargo run -p u-onnx --example yolov8_probe -- yolov8n.onnx
//! Set OPTIMIZE=1 to also exercise `into_optimized()` (the pass that blew up
//! on MoveNet's GatherND); we expect it to be cheap here.

use std::time::Instant;

use tract_onnx::prelude::*;

fn main() -> TractResult<()> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "yolov8n.onnx".into());
    let optimize = std::env::var("OPTIMIZE").as_deref() == Ok("1");
    eprintln!("loading {path} (optimize={optimize})");

    let t0 = Instant::now();
    let typed = tract_onnx::onnx()
        .model_for_path(&path)?
        .with_input_fact(0, f32::fact([1, 3, 640, 640]).into())?
        .into_typed()?;
    let model = if optimize {
        typed.into_optimized()?.into_runnable()?
    } else {
        typed.into_runnable()?
    };
    eprintln!("OK: loaded + runnable in {:?}", t0.elapsed());

    let input: Tensor = tract_ndarray::Array4::<f32>::zeros((1, 3, 640, 640)).into();
    let t1 = Instant::now();
    let outputs = model.run(tvec!(input.into()))?;
    eprintln!("OK: ran in {:?}", t1.elapsed());

    let view = outputs[0].to_plain_array_view::<f32>()?;
    eprintln!("output shape = {:?}", view.shape());
    // For a head-only model this would be [1, 5, 8400] and the decode is
    // u-onnx's, truncated to the first 5 channels (no mask planes).
    Ok(())
}
