#!/usr/bin/env python3
"""Re-derive a backend's annotation lines after a SMALL camera move.

When a camera gets nudged a few pixels (someone bumps the webcam), the old
hand-validated lines are stale but the scene is unchanged. This tool tracks
patches sampled ALONG each annotated line from the reference image into a
fresh frame (template matching on illumination-normalized crops), refits
each line through the matched points (least squares), and emits a new
annotation entry in the annotator's JSON format.

Rails get samples down to the image bottom — a long baseline, because the
vanishing-point maths downstream amplifies any direction error.

Always validate the output: run extend_dataset.py against the new reference
(every same-day frame must pass the ≤1.5 px corner gate) and eyeball the
annotate_check.py overlay.

Usage:
  ../../output/u-seg/.venv/bin/python realign_lines.py \
      --merged dataset_stage/merged.json --images input \
      --ref  ht_webcam-1_20260804-152629_411983_raw.png \
      --new  ht_webcam-1_20260805-133025_f1177b_raw.png \
      --out output/realigned-webcam.json
"""

import argparse
import json
import os

import cv2
import numpy as np


def norm_local(img):
    f = img.astype(np.float32)
    return f - cv2.GaussianBlur(f, (0, 0), 8)


def match_point(ref_n, img_n, pt, patch=20, search=30):
    h, w = ref_n.shape
    x, y = int(round(pt[0])), int(round(pt[1]))
    if not (patch <= x < w - patch and patch <= y < h - patch):
        return None
    tpl = ref_n[y - patch : y + patch + 1, x - patch : x + patch + 1]
    x0, x1 = max(0, x - search - patch), min(w, x + search + patch + 1)
    y0, y1 = max(0, y - search - patch), min(h, y + search + patch + 1)
    res = cv2.matchTemplate(img_n[y0:y1, x0:x1], tpl, cv2.TM_CCOEFF_NORMED)
    _, score, _, (mx, my) = cv2.minMaxLoc(res)
    if score < 0.35:
        return None
    return (x0 + mx + patch, y0 + my + patch, score)


def clip_line_samples(l, w, h, n=7, margin=24):
    """`n` sample points spread over the segment's extent inside the frame."""
    p1 = np.array([l["p1"]["x"], l["p1"]["y"]])
    p2 = np.array([l["p2"]["x"], l["p2"]["y"]])
    ts = np.linspace(0.0, 1.0, 200)
    pts = p1[None, :] + ts[:, None] * (p2 - p1)[None, :]
    ok = (
        (pts[:, 0] >= margin) & (pts[:, 0] < w - margin)
        & (pts[:, 1] >= margin) & (pts[:, 1] < h - margin)
    )
    pts = pts[ok]
    if len(pts) < n:
        return pts
    idx = np.linspace(0, len(pts) - 1, n).round().astype(int)
    return pts[idx]


def fit_line(points):
    """Total-least-squares line through matched points → (p, direction)."""
    pts = np.array([(x, y) for x, y, _ in points], np.float64)
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c)
    return c, vt[0]


def to_border_segment(c, d, w, h):
    """Extend the fitted line to the frame borders, annotator-style."""
    if abs(d[1]) > abs(d[0]):  # mostly vertical → cut at y = 0 and y = h
        t0, t1 = (0 - c[1]) / d[1], (h - c[1]) / d[1]
    else:  # mostly horizontal → cut at x = 0 and x = w
        t0, t1 = (0 - c[0]) / d[0], (w - c[0]) / d[0]
    a, b = c + t0 * d, c + t1 * d
    return {
        "p1": {"x": round(float(a[0]), 2), "y": round(float(a[1]), 2)},
        "p2": {"x": round(float(b[0]), 2), "y": round(float(b[1]), 2)},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--merged", required=True)
    ap.add_argument("--images", required=True)
    ap.add_argument("--ref", required=True, help="annotated reference image name")
    ap.add_argument("--new", required=True, help="fresh image to realign onto")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    images = json.load(open(args.merged))["images"]
    rec = images[args.ref]
    ref = cv2.imread(os.path.join(args.images, args.ref), cv2.IMREAD_GRAYSCALE)
    new = cv2.imread(os.path.join(args.images, args.new), cv2.IMREAD_GRAYSCALE)
    assert ref is not None and new is not None and ref.shape == new.shape
    h, w = ref.shape
    ref_n, new_n = norm_local(ref), norm_local(new)

    # A nudge is a rigid camera move: estimate ONE similarity transform
    # (rotation + translation + scale, RANSAC) from patch matches collected
    # along every annotated line, then map the old lines through it. Exact
    # for camera roll; the linear extrapolation error of a small pan stays
    # sub-pixel at this displacement scale.
    src, dst = [], []
    for lname, l in rec["annotations"].items():
        for p in clip_line_samples(l, w, h, n=13):
            m = match_point(ref_n, new_n, p)
            if m is None:
                continue
            dx, dy = m[0] - p[0], m[1] - p[1]
            if max(abs(dx), abs(dy)) > 12.0:
                continue  # false match: screen content / glare changed
            src.append((p[0], p[1]))
            dst.append((m[0], m[1]))
    src, dst = np.array(src, np.float32), np.array(dst, np.float32)
    if len(src) < 8:
        raise SystemExit(f"only {len(src)} trusted matches — annotate by hand")
    tf, inl = cv2.estimateAffinePartial2D(
        src, dst, method=cv2.RANSAC, ransacReprojThreshold=1.5
    )
    if tf is None or inl.sum() < 8:
        raise SystemExit("no consistent rigid move found — annotate by hand")
    angle = np.degrees(np.arctan2(tf[1, 0], tf[0, 0]))
    scale = float(np.hypot(tf[0, 0], tf[1, 0]))
    resid = np.abs((src @ tf[:, :2].T + tf[:, 2]) - dst)[inl.ravel() == 1].max()
    print(f"rigid move: rot {angle:+.3f} deg, scale {scale:.4f}, "
          f"t=({tf[0, 2]:+.2f},{tf[1, 2]:+.2f})px, "
          f"{int(inl.sum())}/{len(src)} inliers, max residual {resid:.2f}px")

    def warp(x, y):
        v = tf @ np.array([x, y, 1.0])
        return float(v[0]), float(v[1])

    out_ann = {}
    for lname, l in rec["annotations"].items():
        a = warp(l["p1"]["x"], l["p1"]["y"])
        b = warp(l["p2"]["x"], l["p2"]["y"])
        c = np.array(a)
        d = np.array(b) - np.array(a)
        d = d / np.linalg.norm(d)
        seg = to_border_segment(c, d, w, h)
        seg["pivot"] = seg["p1"]
        seg["angle_deg"] = float(np.degrees(np.arctan2(
            seg["p2"]["y"] - seg["p1"]["y"], seg["p2"]["x"] - seg["p1"]["x"])))
        out_ann[lname] = seg

    entry = {args.new: {"width": rec["width"], "height": rec["height"],
                        "annotations": out_ann}}
    json.dump({"schema": "anchor-lines-v1", "images": entry}, open(args.out, "w"))
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
