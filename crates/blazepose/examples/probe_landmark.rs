//! Probe: dump the landmark model's output tensors (name/shape/range).
fn main() {
    let mut bp = blazepose::BlazePose::new().expect("load");
    for o in bp.debug_landmark().expect("run") {
        println!(
            "{:22} shape={:?}  min={:.3} max={:.3}  (n={})",
            o.name, o.shape, o.min, o.max, o.positive
        );
    }
}
