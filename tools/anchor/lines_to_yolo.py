#!/usr/bin/env python3
"""Convert anchor-line annotations into a YOLO-pose dataset.

The annotator (`annotator.html`) exports **4 lines** per image
(`sideleft`, `sideright`, `lockbar_player`, `lockbar_screen`). A neural net
cannot regress the extrapolated line/border endpoints — there is no pixel
evidence where an extended line crosses the frame edge (ceiling, wall). So this
converter turns the lines into **6 anchored keypoints** that *do* sit on real
image features, all derived from the lines:

  0 player_left   = sideleft  ∩ lockbar_player      (real bar corner)
  1 player_right  = sideright ∩ lockbar_player
  2 screen_right  = sideright ∩ lockbar_screen
  3 screen_left   = sideleft  ∩ lockbar_screen
  4 bottom_left   = sideleft  ∩ (y = H-1)           (visible start of the rail)
  5 bottom_right  = sideright ∩ (y = H-1)

The decoder (Rust) rebuilds the full lines from these points and does the
vanishing-point / homography maths. The metric reference is the **610 mm width
between the two sidebars** — never the thin lockbar thickness (ill-conditioned).

Bounding box: YOLO-pose requires one box per instance. There is exactly one
cabinet per image, so we use a fixed **full-width x bottom-third** box. It
guarantees all 6 keypoints fall inside (positive-anchor assignment works) and is
robust to the lockbar sitting low/left in the frame.

Usage:
    python lines_to_yolo.py --json anchor-lines.json --images <dir> --out <dataset_dir>
"""
import argparse, json, os, shutil

KPT_ORDER = ["player_left", "player_right", "screen_right",
             "screen_left", "bottom_left", "bottom_right"]


def line_pts(a):
    return (a["p1"]["x"], a["p1"]["y"]), (a["p2"]["x"], a["p2"]["y"])


def intersect(la, lb):
    (x1, y1), (x2, y2) = line_pts(la)
    (x3, y3), (x4, y4) = line_pts(lb)
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-9:
        raise ValueError("parallel lines")
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
    return px, py


def at_y(line, y):
    (x1, y1), (x2, y2) = line_pts(line)
    if abs(y2 - y1) < 1e-9:
        return x1, y
    t = (y - y1) / (y2 - y1)
    return x1 + t * (x2 - x1), y


def six_points(an, W, H):
    return [
        intersect(an["sideleft"], an["lockbar_player"]),
        intersect(an["sideright"], an["lockbar_player"]),
        intersect(an["sideright"], an["lockbar_screen"]),
        intersect(an["sideleft"], an["lockbar_screen"]),
        at_y(an["sideleft"], H - 1),
        at_y(an["sideright"], H - 1),
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", required=True, help="anchor-lines.json from the annotator")
    ap.add_argument("--images", required=True, help="folder holding the source images")
    ap.add_argument("--out", required=True, help="output YOLO dataset dir")
    ap.add_argument("--val-split", type=float, default=0.0,
                    help="fraction of images held out for val (0 = train==val)")
    args = ap.parse_args()

    data = json.load(open(args.json))
    images = data.get("images", data)
    for sub in ("images/train", "labels/train", "images/val", "labels/val"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)

    names = [n for n in images if os.path.exists(os.path.join(args.images, n))]
    names.sort()
    n_val = int(len(names) * args.val_split)
    val = set(names[:n_val])

    n = 0
    for name in names:
        rec = images[name]
        an = rec["annotations"]
        W, H = rec["width"], rec["height"]
        if not all(k in an for k in ("sideleft", "sideright", "lockbar_player", "lockbar_screen")):
            print(f"skip {name}: missing a line")
            continue
        split = "val" if name in val else "train"
        shutil.copy(os.path.join(args.images, name),
                    os.path.join(args.out, "images", split, name))
        pts = six_points(an, W, H)
        # fixed bbox: full width, bottom third
        parts = ["0 0.5 0.833333 1.0 0.333333"]
        for x, y in pts:
            parts.append(f"{min(1, max(0, x / W)):.6f} {min(1, max(0, y / H)):.6f} 2")
        stem = os.path.splitext(name)[0]
        with open(os.path.join(args.out, "labels", split, stem + ".txt"), "w") as f:
            f.write(" ".join(parts) + "\n")
        n += 1

    # duplicate val==train when no split (BatchNorm needs a real batch; a real
    # dataset with dozens of images makes this moot)
    val_line = "images/train" if n_val == 0 else "images/val"
    with open(os.path.join(args.out, "data.yaml"), "w") as f:
        f.write(
            f"path: {os.path.abspath(args.out)}\n"
            f"train: images/train\nval: {val_line}\n"
            f"kpt_shape: [6, 3]\nflip_idx: [1, 0, 3, 2, 5, 4]\n"
            f"names:\n  0: anchor\n")
    print(f"wrote {n} labelled images -> {args.out}")
    print("keypoint order:", KPT_ORDER)


if __name__ == "__main__":
    main()
