# personseg

Person silhouette segmentation for the webcam tracking path — MediaPipe Selfie
Segmentation run through pure-Rust `tract` (same paradigm as `head` / `u-onnx`,
no native ONNX runtime shipped).

It turns a plain RGB webcam frame into a **coarse binary person mask** on *any*
background, robust to lighting — the depth-less counterpart to the Kinect
near-slab silhouette. That mask feeds `skeleton-depth`'s `track_mask`, so webcam
and Kinect share one skeleton pipeline. We only need a rough silhouette (the
skeleton method tolerates ragged edges), so the smallest model wins.

## API

```rust
let seg = personseg::Segmenter::new()?;            // loads the embedded model
let sil = seg.silhouette(rgb888, w, h, personseg::DEFAULT_THRESHOLD);
// sil.data: 256×256 bytes, 255 = person / 0 = background
```

## Model

`models/selfie_seg.onnx` — MediaPipe Selfie Segmentation "general", from
[`onnx-community/mediapipe_selfie_segmentation`](https://huggingface.co/onnx-community/mediapipe_selfie_segmentation)
(**Apache-2.0**, GPL-compatible). Input `pixel_values [1,3,256,256]` RGB in
`[0,1]` (plain resize, ÷255, no mean/std); output `alphas [1,1,256,256]`, already
sigmoid-activated.

### Regenerating the embedded model

The upstream export has a dynamic batch dim and shape-computed `Resize` sizes
that tract can't analyse. Two offline fixes make it tract-loadable (no runtime
change):

```bash
pip install onnx onnxsim onnxruntime
python - <<'PY'
import onnx
from onnx import shape_inference
m = onnx.load('model.onnx'); g = m.graph
def fix(vi, dims):
    for d, v in zip(vi.type.tensor_type.shape.dim, dims):
        d.ClearField('dim_param'); d.dim_value = v
fix(g.input[0],  [1, 3, 256, 256])   # freeze the dynamic batch
fix(g.output[0], [1, 1, 256, 256])
onnx.save(shape_inference.infer_shapes(m), 'static.onnx')
PY
python -m onnxsim static.onnx selfie_seg.onnx   # const-fold Shape/Slice → static Resize
```
