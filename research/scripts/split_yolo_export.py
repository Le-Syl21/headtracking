#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# ///
"""Take a Label Studio "YOLOv8 OBB with Images" export and rearrange
into the train/val layout Ultralytics expects, plus write the
lockbar.yaml that the trainer needs.

Input: a directory (or zip) with `images/` and `labels/` flat folders.
Output: dataset/yolo/{images,labels}/{train,val}/ + lockbar.yaml.

Usage:
    uv run scripts/split_yolo_export.py <export_dir_or_zip>

If <export_dir_or_zip> is a zip, it gets unzipped to a temp dir first.
"""
from __future__ import annotations

import random
import shutil
import sys
import tempfile
import zipfile
from pathlib import Path

HERE = Path(__file__).parent
DATASET = HERE.parent / "dataset"
YOLO_DIR = DATASET / "yolo"
SPLIT_SEED = 42
VAL_FRACTION = 0.20


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("Usage: split_yolo_export.py <export_dir_or_zip>")
        return 1
    src = Path(argv[1]).expanduser().resolve()
    if not src.exists():
        print(f"Not found: {src}")
        return 1

    if src.is_file() and src.suffix == ".zip":
        tmp = Path(tempfile.mkdtemp(prefix="yolo_export_"))
        with zipfile.ZipFile(src) as z:
            z.extractall(tmp)
        src_dir = tmp
        cleanup = lambda: shutil.rmtree(tmp, ignore_errors=True)
    elif src.is_dir():
        src_dir = src
        cleanup = lambda: None
    else:
        print(f"Expected a directory or .zip, got: {src}")
        return 1

    images_dir = src_dir / "images"
    labels_dir = src_dir / "labels"
    if not images_dir.is_dir() or not labels_dir.is_dir():
        # Sometimes the zip has an extra top-level folder.
        sub = next((p for p in src_dir.iterdir() if p.is_dir()
                    and (p / "images").is_dir() and (p / "labels").is_dir()), None)
        if sub is None:
            print(f"No images/labels folders found under {src_dir}")
            cleanup()
            return 1
        images_dir = sub / "images"
        labels_dir = sub / "labels"

    pairs: list[tuple[Path, Path]] = []
    for img in sorted(images_dir.iterdir()):
        if img.suffix.lower() not in {".jpg", ".jpeg", ".png", ".webp"}:
            continue
        lbl = labels_dir / f"{img.stem}.txt"
        if not lbl.is_file():
            print(f"  ! no label for {img.name}; skipping")
            continue
        if lbl.stat().st_size == 0:
            print(f"  ! empty label for {img.name}; skipping")
            continue
        pairs.append((img, lbl))

    if not pairs:
        print("No (image, label) pairs found")
        cleanup()
        return 1

    print(f"Found {len(pairs)} annotated image(s)")

    if YOLO_DIR.exists():
        print(f"Wiping previous {YOLO_DIR}")
        shutil.rmtree(YOLO_DIR)
    for split in ("train", "val"):
        (YOLO_DIR / "images" / split).mkdir(parents=True)
        (YOLO_DIR / "labels" / split).mkdir(parents=True)

    random.seed(SPLIT_SEED)
    random.shuffle(pairs)
    n_val = max(1, int(len(pairs) * VAL_FRACTION))
    val_set = pairs[:n_val]
    train_set = pairs[n_val:]
    print(f"split → train={len(train_set)}  val={len(val_set)}")

    for split, items in (("train", train_set), ("val", val_set)):
        for img, lbl in items:
            shutil.copy2(img, YOLO_DIR / "images" / split / img.name)
            shutil.copy2(lbl, YOLO_DIR / "labels" / split / lbl.name)

    yaml_path = YOLO_DIR / "lockbar.yaml"
    yaml_path.write_text(
        f"# YOLO-OBB dataset for the headtracking lockbar detector.\n"
        f"path: {YOLO_DIR.resolve()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"names:\n"
        f"  0: lockbar\n"
    )
    print(f"\nWrote {yaml_path}")
    print("\nNext step (training):")
    print(f"  uvx --from ultralytics yolo obb train \\")
    print(f"    data={yaml_path.resolve()} \\")
    print(f"    model=yolo11n-obb.pt epochs=100 imgsz=640 batch=8")

    cleanup()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
