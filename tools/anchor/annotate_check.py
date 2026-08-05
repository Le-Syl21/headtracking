#!/usr/bin/env python3
"""Render anchor-line annotations + the 6 derived keypoints onto raw captures.

Reads a lines JSON in the annotator schema (anchor-lines-v1), derives the six
keypoints exactly like lines_to_yolo.py, and writes `<stem>_check.png` overlays
into --out for visual review. Used to self-check hand/assisted annotations
before they enter the training set.
"""
import argparse
import json
import os

from PIL import Image, ImageDraw

# Same derivation as lines_to_yolo.py (kept in sync by importing it).
from lines_to_yolo import six_points

LINE_COLOURS = {
    "sideleft": (255, 80, 80),        # red
    "sideright": (80, 160, 255),      # blue
    "lockbar_player": (80, 220, 80),  # green
    "lockbar_screen": (255, 200, 40), # yellow
}
KPT_ORDER = ["player_left", "player_right", "screen_right",
             "screen_left", "bottom_left", "bottom_right"]


def draw_line(dr, a, colour, w, h):
    p1 = (a["p1"]["x"], a["p1"]["y"])
    p2 = (a["p2"]["x"], a["p2"]["y"])
    dr.line([p1, p2], fill=colour, width=2)


def render(img_path, rec, out_path):
    im = Image.open(img_path).convert("RGB")
    dr = ImageDraw.Draw(im)
    an = rec["annotations"]
    W, H = rec["width"], rec["height"]
    for name, colour in LINE_COLOURS.items():
        if name in an:
            draw_line(dr, an[name], colour, W, H)
    pts = six_points(an, W, H)
    for i, (x, y) in enumerate(pts):
        r = max(4, W // 300)
        dr.ellipse([x - r, y - r, x + r, y + r], outline=(255, 255, 255), width=2)
        dr.text((x + r + 2, y - r - 2), f"{i}", fill=(255, 255, 255))
    im.save(out_path)
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", required=True)
    ap.add_argument("--images", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    data = json.load(open(args.json))
    os.makedirs(args.out, exist_ok=True)
    for name, rec in data["images"].items():
        src = os.path.join(args.images, name)
        if not os.path.exists(src):
            print(f"skip {name}: image not found")
            continue
        stem = os.path.splitext(name)[0]
        out = os.path.join(args.out, f"{stem}_check.png")
        pts = render(src, rec, out)
        print(f"{name}")
        for label, (x, y) in zip(KPT_ORDER, pts):
            print(f"    {label:13s} ({x:7.1f}, {y:7.1f})")
        print(f"  -> {out}")


if __name__ == "__main__":
    main()
