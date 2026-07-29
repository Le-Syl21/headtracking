//! Throwaway micro-benchmark: how long does one forward pass of a head ONNX
//! take in tract, at a given square input size?
//!   cargo run --release -p head --example bench -- <model.onnx> <size> [n]
//! Prints mean ms/inference over `n` runs (default 30) on a zeroed frame.

use std::time::Instant;

use tract_onnx::prelude::*;

fn main() -> TractResult<()> {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("usage: bench <model.onnx> <size> [n]");
    let size: usize = args
        .next()
        .expect("need input size")
        .parse()
        .expect("bad size");
    let n: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(30);

    let model = tract_onnx::onnx()
        .model_for_path(&path)?
        .with_input_fact(0, f32::fact([1, 3, size, size]).into())?
        .into_optimized()?
        .into_runnable()?;

    let input: Tensor = tract_ndarray::Array4::<f32>::zeros((1, 3, size, size)).into();
    // Warm up (first run pays lazy allocations).
    let _ = model.run(tvec!(input.clone().into()))?;

    let t = Instant::now();
    for _ in 0..n {
        let _ = model.run(tvec!(input.clone().into()))?;
    }
    let ms = t.elapsed().as_secs_f64() * 1000.0 / n as f64;
    println!("{path} @ {size}: {ms:.1} ms/inference (mean of {n})");
    Ok(())
}
