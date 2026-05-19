#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "ultralytics>=8.4",
#     "opencv-python>=4.8",
# ]
# ///
"""Run the trained best.pt on the original real-pincab captures and
draw the detection quad. Sanity check that the model handles the 3
backends (Kinect v2, Kinect v1, webcam) before we touch the Rust code.
"""
from pathlib import Path
import sys

import cv2
import numpy as np
from ultralytics import YOLO

HERE = Path(__file__).parent
RESEARCH = HERE.parent
WEIGHTS = RESEARCH / "runs/obb/training/lockbar_v1/weights/best.pt"

TARGETS = [
    "b-03.png",     # Kinect v2
    "v1b-01.png",   # Kinect v1
    "wb-01.png",    # webcam
]


def render(model: YOLO, name: str) -> None:
    src = RESEARCH / name
    if not src.exists():
        print(f"  ! missing {src}")
        return
    bgr = cv2.imread(str(src))
    results = model.predict(source=str(src), conf=0.05, verbose=False)
    res = results[0]
    n = 0 if res.obb is None else len(res.obb)
    out = bgr.copy()
    for i in range(n):
        # xyxyxyxy: 4×2 array of corners in image coords
        pts = res.obb.xyxyxyxy[i].cpu().numpy().astype(np.int32).reshape(-1, 2)
        conf = float(res.obb.conf[i])
        cv2.polylines(out, [pts], True, (255, 229, 0), thickness=3)
        cx, cy = pts.mean(axis=0).astype(int)
        cv2.putText(out, f"{conf:.2f}", (cx - 30, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 229, 0), 2)
    out_path = RESEARCH / f"yolo_pred_{Path(name).stem}.png"
    cv2.imwrite(str(out_path), out)
    print(f"  {name}: {n} detection(s) → {out_path.name}")


def main() -> int:
    if not WEIGHTS.exists():
        print(f"weights not found: {WEIGHTS}")
        return 1
    model = YOLO(str(WEIGHTS))
    for t in TARGETS:
        render(model, t)
    return 0


if __name__ == "__main__":
    sys.exit(main())
