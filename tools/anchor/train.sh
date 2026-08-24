#!/usr/bin/env bash
# Train the anchor keypoint model (yolo11n-pose) and export it to ONNX.
#
# The settled recipe (see README.md → "What we learned the hard way"):
#   - 6 anchored keypoints (produced by lines_to_yolo.py)
#   - fixed full-width x bottom-third bbox
#   - geometric augmentation OFF (border keypoints would leave the frame)
#   - batch >= 16 — CRITICAL: a smaller batch degenerates BatchNorm and the
#     pose head never converges (pose_loss stays flat). With a real dataset of
#     dozens of images this is automatic.
#
# Usage:  ./train.sh <dataset_dir> [epochs] [imgsz] [batch]
set -euo pipefail

DATA="${1:?usage: train.sh <dataset_dir> [epochs] [imgsz] [batch]}/data.yaml"
EPOCHS="${2:-200}"
IMGSZ="${3:-1280}"
BATCH="${4:-16}"

HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/runs"

# Pick a python/yolo: prefer a project venv if there is one, else system yolo.
# `output/u-seg/` is the historical name — the segmentation experiment that
# seeded this tooling is gone, but the venv it created is still a working
# ultralytics install, so keep finding it rather than making people rebuild one.
YOLO="yolo"
for candidate in "$HERE/../../output/anchor/.venv" "$HERE/../../output/u-seg/.venv"; do
  if [ -x "$candidate/bin/yolo" ]; then
    YOLO="$candidate/bin/yolo"
    break
  fi
done

echo "▶ training  data=$DATA  epochs=$EPOCHS  imgsz=$IMGSZ  batch=$BATCH"
[ "$BATCH" -lt 16 ] && echo "⚠  batch < 16 : BatchNorm may not converge (see README)."

"$YOLO" pose train model=yolo11n-pose.pt data="$DATA" \
  epochs="$EPOCHS" imgsz="$IMGSZ" batch="$BATCH" device=0 \
  fliplr=0 mosaic=0 translate=0 scale=0 degrees=0 shear=0 perspective=0 \
  erasing=0 close_mosaic=0 \
  project="$OUT" name=anchor exist_ok=True

echo "▶ exporting ONNX"
"$YOLO" export model="$OUT/anchor/weights/best.pt" format=onnx imgsz="$IMGSZ" simplify=True
cp "$OUT/anchor/weights/best.onnx" "$HERE/models/anchor_rgb.onnx"
echo "✔ model -> $HERE/models/anchor_rgb.onnx"
