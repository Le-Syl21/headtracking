#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "pillow>=10.0",
# ]
# ///
"""Batch-run the Rust YOLO lockbar detector over every annotated image
in `research/dataset/yolo/images/{train,val}/`, save each overlay
(with the confidence rendered top-right) under `output/`.

The Rust binary (`lockbar-replay`) only reads 8-bit RGB PNGs, so we
transcode JPG/WEBP/etc to a tmp PNG first. Confidence is read from
the binary's `SUMMARY conf=...` stdout line.

Usage:
    cd research/
    uv run scripts/batch_detect.py            # train + val
    uv run scripts/batch_detect.py train      # train only
    uv run scripts/batch_detect.py val        # val only
"""
from __future__ import annotations

import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).parent
RESEARCH = HERE.parent
REPO = RESEARCH.parent
BINARY = REPO / "target/release/lockbar-replay"
OUTPUT_DIR = REPO / "output"
DATA_ROOT = RESEARCH / "dataset/yolo/images"

CONF_RE = re.compile(r"SUMMARY conf=(\d+\.\d+)")
LOW_CONF_THRESHOLD = 0.30


def find_font(size: int = 36) -> ImageFont.ImageFont:
    """Try a few common system fonts; fall back to Pillow's default
    (which is small but always present)."""
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf",
        "/Library/Fonts/Arial Bold.ttf",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size=size)
        except OSError:
            continue
    return ImageFont.load_default()


def to_png_8bit_rgb(src: Path, dst: Path) -> None:
    """Decode any format Pillow understands, save as 8-bit RGB PNG
    (matches what lockbar-replay expects)."""
    with Image.open(src) as im:
        im.convert("RGB").save(dst, "PNG")


def parse_confidence(stdout: str) -> float | None:
    m = CONF_RE.search(stdout)
    return float(m.group(1)) if m else None


def annotate(
    overlay_path: Path,
    out_path: Path,
    src_name: str,
    conf: float | None,
    font: ImageFont.ImageFont,
) -> None:
    """Draw the source filename + confidence in the top-right corner."""
    with Image.open(overlay_path) as im:
        im = im.convert("RGB")
        draw = ImageDraw.Draw(im)
        if conf is None:
            text = f"{src_name}\nNO DETECTION"
            color = (255, 64, 64)
        else:
            text = f"{src_name}\nconf {conf:.3f}"
            color = (64, 255, 96) if conf >= LOW_CONF_THRESHOLD else (255, 200, 32)
        # Compute text size; position at top-right with 16 px margin.
        bbox = draw.multiline_textbbox((0, 0), text, font=font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]
        margin = 16
        x = im.width - tw - margin
        y = margin
        # Black box behind text for readability against any background.
        pad = 8
        draw.rectangle(
            [x - pad, y - pad, x + tw + pad, y + th + pad],
            fill=(0, 0, 0),
        )
        draw.multiline_text((x, y), text, font=font, fill=color)
        im.save(out_path, "PNG")


def process(src: Path, font: ImageFont.ImageFont) -> tuple[str, float | None]:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        in_png = tmp / "input.png"
        overlay = tmp / "overlay.png"
        to_png_8bit_rgb(src, in_png)
        proc = subprocess.run(
            [str(BINARY), str(in_png), "--out", str(overlay), "--score", "0.05"],
            capture_output=True, text=True,
        )
        if proc.returncode != 0:
            print(f"  ! {src.name}: binary failed\n{proc.stderr}", file=sys.stderr)
            return src.name, None
        conf = parse_confidence(proc.stdout)
        # If the detector found nothing, lockbar-replay leaves the
        # overlay PNG identical to the input. Either way we annotate.
        if not overlay.exists():
            shutil.copy2(in_png, overlay)
        out_path = OUTPUT_DIR / f"{src.stem}.png"
        annotate(overlay, out_path, src.name, conf, font)
    return src.name, conf


def collect(split: str) -> list[Path]:
    root = DATA_ROOT / split
    if not root.is_dir():
        print(f"missing: {root}", file=sys.stderr)
        return []
    out: list[Path] = []
    for ext in ("*.jpg", "*.jpeg", "*.png", "*.webp"):
        out.extend(sorted(root.glob(ext)))
    return out


def main(argv: list[str]) -> int:
    if not BINARY.exists():
        print(f"binary not found: {BINARY}\nbuild first: cargo build -p lockbar-replay --release",
              file=sys.stderr)
        return 1
    splits = argv[1:] or ["train", "val"]
    font = find_font(36)
    rows: list[tuple[str, str, float | None]] = []
    for split in splits:
        imgs = collect(split)
        print(f"[{split}] {len(imgs)} image(s)")
        for i, img in enumerate(imgs, 1):
            name, conf = process(img, font)
            conf_str = f"{conf:.3f}" if conf is not None else "  -  "
            print(f"  [{i:3d}/{len(imgs)}] {conf_str}  {name}")
            rows.append((split, name, conf))

    # Summary stats — high/medium/low/none counts and the mean.
    print("\n========== summary ==========")
    by_split: dict[str, list[float | None]] = {}
    for split, _name, conf in rows:
        by_split.setdefault(split, []).append(conf)
    for split, confs in by_split.items():
        det = [c for c in confs if c is not None and c > 0.0]
        none_n = sum(1 for c in confs if c is None or c == 0.0)
        if det:
            mean = sum(det) / len(det)
            print(f"  {split:>5}: {len(det)}/{len(confs)} detected  "
                  f"(mean conf {mean:.3f}, no-detect {none_n})")
        else:
            print(f"  {split:>5}: 0/{len(confs)} detected  (no-detect {none_n})")
    print(f"\noutput → {OUTPUT_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
