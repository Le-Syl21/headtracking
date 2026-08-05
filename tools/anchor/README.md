# `anchor` — cabinet auto-calibration toolkit

This folder holds the home-grown pipeline that turns a handful of **hand-drawn
lines on cabinet photos** into an ONNX model that locates the pinball cabinet's
reference frame (lockbar + side rails) in any image. That frame is what lets
headtracking recover the camera's pose **without any manual calibration** — the
star feature of the project.

```
annotator.html  ──►  anchor-lines.json  ──►  lines_to_yolo.py  ──►  YOLO-pose dataset
                                                                          │
                                                              train.sh (yolo11n-pose)
                                                                          │
                                                                   models/anchor_rgb.onnx
                                                                          │
                                              Rust decoder: keypoints → lines → vanishing
                                              points + homography → focal + camera pose
```

## 1. Draw the lines — `annotator.html`

Open the file in any browser (no server, no install). Drop a folder of cabinet
photos in, and for each image draw **4 lines**:

| line | what to trace |
|------|---------------|
| `sideleft` / `sideright` | the two cabinet **side rails** |
| `lockbar_player` | the lockbar edge on the **player** side |
| `lockbar_screen` | the lockbar edge on the **playfield/screen** side |

Each line is placed with a **pivot + rotation**: click a first point on the
edge, the line pivots to follow the cursor, click again to lock the angle — the
two endpoints are snapped to the image border for a maximal, precise baseline.
Click the **2 extremities** of each edge; the further apart, the more precise the
line. Everything auto-saves to the browser; **Export JSON** gives you
`anchor-lines.json`.

Convention (see `../../` memory `anchor-labeling-convention`): everything is in
**camera / image space** (top-left = image top-left, never "player left"). The
camera↔player mirror is absorbed once, downstream.

## 2. Lines → dataset — `lines_to_yolo.py`

A neural net cannot learn the *extrapolated* line endpoints (there is no pixel
where an extended line hits the ceiling). So the converter derives **6 keypoints
that sit on real features**, all from your 4 lines:

```
0 player_left   1 player_right   2 screen_right
3 screen_left   4 bottom_left    5 bottom_right
```

The 4 corners are line intersections (real bar corners); the 2 bottom points are
where each rail meets the last image row (the visible start of the rail). It also
writes a fixed **full-width × bottom-third bounding box** (one cabinet per image).

```bash
python lines_to_yolo.py \
  --json anchor-lines.json \
  --images /path/to/photos \
  --out dataset
```

## 3. Train + export — `train.sh`

```bash
./train.sh dataset            # epochs=200 imgsz=1280 batch=16
# -> models/anchor_rgb.onnx
```

Uses `yolo11n-pose`, geometric augmentation off, and **batch ≥ 16**.

## What we learned the hard way

Three non-obvious traps, all now baked into the scripts:

1. **Border/extrapolated keypoints are unlearnable.** Points where a line hits
   the frame edge have no visual evidence → the model guesses (200–1000 px
   error). Only points on real features (corners, visible rail) are learnable.
2. **Small batch kills BatchNorm.** With `batch < 16` the pose head never
   converges (`pose_loss` stays flat, ~11) — even overfitting a single image
   fails. `batch ≥ 16` fixes it instantly. A real dataset (dozens of images)
   makes this automatic.
3. **Never trust the lockbar *thickness* (70 mm).** It is thin and
   near-fronto-parallel → focal estimation from it is wildly ill-conditioned.
   The metric reference is the **610 mm width between the two sidebars**.

## Metric / geometry notes for the decoder

- Reference width = **610 mm** between the two sidebars (`LOCKBAR_WIDTH_MM`).
- Distance cam↔bar `= focal × 610 / pixel_width` — validated to **±0–3 %**
  against tape-measured ground truth on Kinect v1/v2 and a webcam.
- Focal: Kinect uses its **factory colour focal**; the webcam focal comes from
  the sidebar/lockbar geometry (`src/calibration/autocalib.rs`,
  `calibrate_homography`). Vanishing-point-only focal (`calibrate_from_lockbar`)
  is degenerate for a centred camera — use the homography.

## Status

`models/anchor_rgb_proof.onnx` (if present) is a **proof trained on 3 images** —
it validates the full pipeline end-to-end but does **not** generalize. The
production model, trained on a real annotated set, will live in
`crates/anchor/models/`. Contributions of annotated cabinet photos are the single
biggest thing that moves this forward — see the main README.
