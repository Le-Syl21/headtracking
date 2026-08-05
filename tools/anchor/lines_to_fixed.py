#!/usr/bin/env python3
"""Turn annotator line JSON into the demo's `anchor_fixed.json`.

The demo (headtracking-demo) looks for `anchor_fixed.json` next to its binary:
when an entry matches the active backend and stream dimensions, the anchor
geometry is built from these hand-placed lines and the (still weak) anchor
model never runs — the "hand-fixed calibration" bridge until the model is
trained on enough contributed captures.

Input : the annotator's export (same schema `lines_to_yolo.py` consumes):
        {"images": {"<name>": {"width": W, "height": H,
                               "annotations": {"sideleft": {"p1": {...}, "p2": {...}}, ...}}}}
Output: {"kinect-v2": {"img_w": .., "img_h": .., "lines": {"sideleft": [[x,y],[x,y]], ...}}, ...}

Backend keys come from the capture stems (`ht_<backend>_...`). Webcams are
written under the plain key "webcam" — the demo matches any `webcam-<N>` slug
to it, because the SDL index shifts with USB enumeration order.

Usage:
    python3 lines_to_fixed.py <lines.json> [--pick STEM ...] [--out anchor_fixed.json]

With several annotated captures for one backend, the first (sorted) wins;
use --pick to select specific stems (substring match) instead.
"""
import argparse
import json
import sys

LINE_NAMES = ("sideleft", "sideright", "lockbar_player", "lockbar_screen")


def backend_key(image_name):
    """`ht_kinect-v1_20260804-..._raw.png` -> `kinect-v1`; webcams -> `webcam`."""
    stem = image_name
    if stem.startswith("ht_"):
        stem = stem[3:]
    backend = stem.split("_")[0]
    if backend.startswith("webcam"):
        return "webcam"
    return backend


def to_seg(line):
    return [[line["p1"]["x"], line["p1"]["y"]], [line["p2"]["x"], line["p2"]["y"]]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lines_json", help="annotator export (anchor-lines*.json)")
    ap.add_argument("--pick", nargs="*", default=[],
                    help="substring(s) selecting which capture to use per backend")
    ap.add_argument("--out", default="anchor_fixed.json")
    args = ap.parse_args()

    data = json.load(open(args.lines_json))
    images = data.get("images", data)

    out = {}
    chosen = {}
    for name in sorted(images):
        rec = images[name]
        an = rec.get("annotations", {})
        if not all(k in an for k in LINE_NAMES):
            print(f"skip {name}: missing a line", file=sys.stderr)
            continue
        if "width" not in rec or "height" not in rec:
            sys.exit(f"error: {name} carries no width/height — re-export from the annotator")
        if args.pick and not any(p in name for p in args.pick):
            continue
        key = backend_key(name)
        if key in out:
            continue  # first (sorted) capture per backend wins
        out[key] = {
            "img_w": rec["width"],
            "img_h": rec["height"],
            "lines": {n: to_seg(an[n]) for n in LINE_NAMES},
        }
        chosen[key] = name

    if not out:
        sys.exit("error: no usable annotation found (check --pick filters)")

    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"wrote {args.out}:")
    for key, name in sorted(chosen.items()):
        e = out[key]
        print(f"  {key:<10} {e['img_w']}x{e['img_h']}  from {name}")


if __name__ == "__main__":
    main()
