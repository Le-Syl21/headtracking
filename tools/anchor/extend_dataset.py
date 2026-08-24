#!/usr/bin/env python3
"""Auto-label new cron captures by reusing the validated per-backend lines.

The cameras are bolted to the cab and the contribution cron captures the same
empty scene every 30 min under drifting daylight. As long as a camera has not
moved, the hand-validated annotation lines of its reference image stay true
for every new frame — free training data with lighting variety.

Safety gate per image, measuring exactly what the labels encode: the four
anchor corners (rail × lockbar-edge intersections) from the reference lines.
A patch around each corner is template-matched in the candidate frame within
a small search window; the image is accepted only if EVERY corner stays
within --max-shift pixels (default 1.5) of where the reference puts it.
A nudged camera, a person in frame, or an occluded anchor breaks at least
one corner and the image is skipped with the measured displacement.

Run with the project's ultralytics venv python (numpy + cv2) — see
tools/anchor/train.sh for where it is looked up:
  ../../output/anchor/.venv/bin/python extend_dataset.py \
      --merged dataset_stage/merged.json --images input \
      --out output/merged-extended.json [--threshold 0.5]
"""

import argparse
import json
import os
import re
import sys

import cv2
import numpy as np


def backend_of(name: str):
    m = re.match(r"ht_(kinect-v1|kinect-v2|webcam)[^_]*_", name)
    return m.group(1) if m else None


def line_xy(l):
    return ((l["p1"]["x"], l["p1"]["y"]), (l["p2"]["x"], l["p2"]["y"]))


def meet(a, b):
    (x1, y1), (x2, y2) = line_xy(a)
    (x3, y3), (x4, y4) = line_xy(b)
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-9:
        return None
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
    return (px, py)


def anchor_corners(ann):
    """The 4 rail × lockbar-edge intersections — the points the labels encode."""
    pts = [
        meet(ann["sideleft"], ann["lockbar_player"]),
        meet(ann["sideright"], ann["lockbar_player"]),
        meet(ann["sideright"], ann["lockbar_screen"]),
        meet(ann["sideleft"], ann["lockbar_screen"]),
    ]
    return [p for p in pts if p is not None]


def norm_local(img: np.ndarray) -> np.ndarray:
    """Remove low-frequency illumination so daylight drift doesn't move the
    match peak."""
    f = img.astype(np.float32)
    return f - cv2.GaussianBlur(f, (0, 0), 8)


def corner_shift(ref_n, img_n, pt, patch=20, search=28):
    """Displacement of the patch around `pt` between reference and candidate.
    Returns (dx, dy, score) or None when the window leaves the frame."""
    h, w = ref_n.shape
    x, y = int(round(pt[0])), int(round(pt[1]))
    if not (patch <= x < w - patch and patch <= y < h - patch):
        return None
    tpl = ref_n[y - patch : y + patch + 1, x - patch : x + patch + 1]
    x0, x1 = max(0, x - search - patch), min(w, x + search + patch + 1)
    y0, y1 = max(0, y - search - patch), min(h, y + search + patch + 1)
    res = cv2.matchTemplate(img_n[y0:y1, x0:x1], tpl, cv2.TM_CCOEFF_NORMED)
    _, score, _, (mx, my) = cv2.minMaxLoc(res)
    return (x0 + mx + patch - x, y0 + my + patch - y, float(score))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--merged", required=True, help="hand-validated annotations JSON")
    ap.add_argument("--images", required=True, help="folder with *_raw.png captures")
    ap.add_argument("--out", required=True, help="extended annotations JSON to write")
    ap.add_argument("--max-shift", type=float, default=1.5,
                    help="max per-corner displacement in px (default 1.5)")
    ap.add_argument("--min-score", type=float, default=0.35,
                    help="min template-match confidence per corner (default 0.35)")
    args = ap.parse_args()

    data = json.load(open(args.merged))
    images = data.get("images", data)

    # Per backend: ALL annotated images whose pixels are still available,
    # newest first (filenames embed the capture timestamp, so lexical order
    # is chronological). A candidate is accepted if ANY reference validates
    # it — newest first because after a camera nudge + realign_lines.py the
    # fresh annotation is the truth; older ones still serve when the newest
    # has a person occluding some corner (the 04/08 pose shots do).
    ref = {}
    for name, rec in sorted(images.items(), reverse=True):
        b = backend_of(name)
        if b and os.path.exists(os.path.join(args.images, name)):
            ref.setdefault(b, []).append((name, rec))
    if not ref:
        sys.exit("no reference image found in --images for any backend")

    ref_data = {}
    for b, entries in ref.items():
        lst = []
        for name, rec in entries:
            img = cv2.imread(os.path.join(args.images, name), cv2.IMREAD_GRAYSCALE)
            corners = anchor_corners(rec["annotations"])
            lst.append((name, norm_local(img), corners, rec))
        ref_data[b] = lst

    out = dict(images)  # keep every hand annotation as-is
    added, skipped = [], []
    for name in sorted(os.listdir(args.images)):
        if not name.endswith("_raw.png") or name in out:
            continue
        b = backend_of(name)
        if b not in ref_data:
            skipped.append((name, "no reference for backend"))
            continue
        img = cv2.imread(os.path.join(args.images, name), cv2.IMREAD_GRAYSCALE)
        if img is None:
            skipped.append((name, "unreadable"))
            continue
        img_n = norm_local(img.astype(np.float32))
        verdicts = []
        accepted = None
        for rname, ref_n, corners, rec in ref_data[b]:
            if img.shape != ref_n.shape:
                verdicts.append("dims")
                continue
            shifts = [corner_shift(ref_n, img_n, p) for p in corners]
            shifts = [s for s in shifts if s is not None]
            if len(shifts) < 3:
                verdicts.append("out-of-frame")
                continue
            worst = max(shifts, key=lambda s: abs(s[0]) + abs(s[1]))
            low = min(s[2] for s in shifts)
            if low < args.min_score:
                verdicts.append(f"score {low:.2f}")
                continue
            if max(abs(worst[0]), abs(worst[1])) > args.max_shift:
                verdicts.append(f"({worst[0]:+.0f},{worst[1]:+.0f})px")
                continue
            accepted = (rname, rec, worst, low)
            break
        if accepted is None:
            skipped.append((name, "no reference agrees: " + ", ".join(verdicts)))
            continue
        rname, rec, worst, low = accepted
        out[name] = {"width": rec["width"], "height": rec["height"],
                     "annotations": rec["annotations"]}
        added.append((name, worst, low, rname))

    json.dump({"schema": "anchor-lines-v1", "images": out}, open(args.out, "w"))
    print(f"kept {len(images)} hand-annotated, added {len(added)}, "
          f"skipped {len(skipped)} -> {args.out}")
    for name, worst, low, rname in added:
        print(f"  + {name}  worst=({worst[0]:+.1f},{worst[1]:+.1f})px "
              f"score>={low:.2f} ref={rname.split('_')[2]}")
    for name, why in skipped:
        print(f"  - {name}  {why}")


if __name__ == "__main__":
    main()
