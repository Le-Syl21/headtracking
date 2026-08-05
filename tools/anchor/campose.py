#!/usr/bin/env python3
"""Offline camera-pose validation from HAND-ANNOTATED anchor lines.

Consumes the annotator's `anchor-lines.json` (4 lines per image: sideleft,
sideright, lockbar_player, lockbar_screen) and derives, per image:

  * the 6 anchor keypoints (same derivation as lines_to_yolo.py),
  * the two vanishing points (depth VP from the rails, width VP from the
    two lockbar edges),
  * the focal length from the orthogonal-VP constraint, with a conditioning
    estimate (+-0.2 deg line-angle noise propagated by finite differences),
  * the full camera pose relative to the cab (pitch vs playfield, yaw, roll,
    distance to the lockbar, lateral offset, height above the playfield),
  * a ground-truth cross-check of the lockbar distance against the captured
    16-bit depth PNG (Kinects only).

Stated assumptions (all approximations are explicit in the report):
  * pinhole camera, square pixels, principal point at the image centre;
  * the playfield is a planar rectangle; the two rails are parallel and
    exactly `--lockbar-mm` apart (610 mm default, the metric reference —
    never the thin lockbar depth);
  * the annotation lives in the RAW COLOUR frame of each capture;
  * Z = f*W/w_px treats the lockbar segment as fronto-parallel (small error
    when the camera yaw is small);
  * "height above playfield" measures to the plane through the lockbar TOP
    (the physical playfield glass sits ~a few cm below it);
  * colour->depth mapping for the ground-truth check is an intrinsics-only
    angular remap (parallel axes, zero baseline). The real lenses sit
    2.5 cm (v1) / ~5 cm (v2) apart, mostly horizontally; the lockbar spans
    the full width, so a horizontal error stays ON the bar and the median
    window absorbs it. This is a sanity check, not a registration.

Stdlib only (the linear algebra is tiny). Usage:
    python3 campose.py --json anchor-lines.json --images <dir> [--lockbar-mm 610]
"""
import argparse
import json
import math
import os
import struct
import zlib

# ----------------------------------------------------------- tiny linear algebra


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return tuple(x / n for x in a)


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(a, s):
    return tuple(x * s for x in a)


# ------------------------------------------------------------------- line helpers


def line_h(p1, p2):
    """Homogeneous line through two euclidean image points."""
    return cross((p1[0], p1[1], 1.0), (p2[0], p2[1], 1.0))


def meet(l1, l2):
    """Homogeneous intersection of two lines (may be at infinity, w ~ 0)."""
    return cross(l1, l2)


def line_pts(a):
    return (a["p1"]["x"], a["p1"]["y"]), (a["p2"]["x"], a["p2"]["y"])


def rot_line(a, deg):
    """Rotate an annotated line by `deg` about its midpoint (noise model)."""
    (x1, y1), (x2, y2) = line_pts(a)
    mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))

    def rot(x, y):
        dx, dy = x - mx, y - my
        return (mx + c * dx - s * dy, my + s * dx + c * dy)

    return rot(x1, y1), rot(x2, y2)


# --------------------------------------------------------------- six anchor points


def intersect_e(la, lb):
    p = meet(line_h(*line_pts(la)), line_h(*line_pts(lb)))
    if abs(p[2]) < 1e-12:
        raise ValueError("parallel lines")
    return (p[0] / p[2], p[1] / p[2])


def at_y(a, y):
    (x1, y1), (x2, y2) = line_pts(a)
    if abs(y2 - y1) < 1e-9:
        return (x1, y)
    t = (y - y1) / (y2 - y1)
    return (x1 + t * (x2 - x1), y)


def six_points(an, H):
    return {
        "player_left": intersect_e(an["sideleft"], an["lockbar_player"]),
        "player_right": intersect_e(an["sideright"], an["lockbar_player"]),
        "screen_right": intersect_e(an["sideright"], an["lockbar_screen"]),
        "screen_left": intersect_e(an["sideleft"], an["lockbar_screen"]),
        "bottom_left": at_y(an["sideleft"], H - 1),
        "bottom_right": at_y(an["sideright"], H - 1),
    }


# ----------------------------------------------------- focal from orthogonal VPs


def focal_from_vps(vd, vw, cx, cy):
    """f^2 = -[(v1-p).(v2-p)] for two finite orthogonal-direction VPs.

    Returns None when either VP is (numerically) at infinity or f^2 <= 0.
    A VP is "at infinity" when |w| is tiny relative to the point scale.
    """
    for v in (vd, vw):
        s = max(abs(v[0]), abs(v[1]), 1.0)
        if abs(v[2]) < 1e-9 * s:
            return None
    v1 = (vd[0] / vd[2] - cx, vd[1] / vd[2] - cy)
    v2 = (vw[0] / vw[2] - cx, vw[1] / vw[2] - cy)
    f2 = -(v1[0] * v2[0] + v1[1] * v2[1])
    if f2 <= 0:
        return None
    return math.sqrt(f2)


def focal_with_sigma(an, cx, cy, noise_deg=0.2):
    """Nominal VP focal + spread when the two lockbar lines wiggle +-noise."""
    ld = line_h(*line_pts(an["sideleft"])), line_h(*line_pts(an["sideright"]))
    vd = meet(*ld)
    lw_p, lw_s = an["lockbar_player"], an["lockbar_screen"]
    f0 = focal_from_vps(vd, meet(line_h(*line_pts(lw_p)), line_h(*line_pts(lw_s))), cx, cy)
    spread = []
    for dp in (-noise_deg, noise_deg):
        for ds in (-noise_deg, noise_deg):
            vw = meet(line_h(*rot_line(lw_p, dp)), line_h(*rot_line(lw_s, ds)))
            f = focal_from_vps(vd, vw, cx, cy)
            spread.append(f)
    if f0 is None or any(f is None for f in spread):
        return f0, None  # some perturbation went degenerate -> unusable
    sigma = max(abs(f - f0) for f in spread)
    return f0, sigma


# ------------------------------------------------------------- camera pose given f


def pose_given_f(an, pts, f, cx, cy, W_mm):
    """Full camera pose relative to the cab, given a focal length."""
    vd = meet(line_h(*line_pts(an["sideleft"])), line_h(*line_pts(an["sideright"])))
    vw = meet(
        line_h(*line_pts(an["lockbar_player"])), line_h(*line_pts(an["lockbar_screen"]))
    )

    # Rail (depth) direction: toward the VP = receding toward the player.
    r = unit((vd[0] / vd[2] - cx, vd[1] / vd[2] - cy, f))

    # Width direction: from the width VP if finite, else parallel to the
    # image plane along the lockbar's common image direction.
    s = max(abs(vw[0]), abs(vw[1]), 1.0)
    if abs(vw[2]) < 1e-9 * s:
        w3 = unit((vw[0], vw[1], 0.0))  # line at infinity: pure image direction
    else:
        w3 = unit((vw[0] / vw[2] - cx, vw[1] / vw[2] - cy, f))
    if w3[0] < 0:
        w3 = scale(w3, -1.0)  # +X_cab = image left -> right

    # 3D angle between rails and lockbar — should be 90 deg if f is right.
    ortho_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot(r, w3)))))

    # Playfield normal (X_cab x D_cab, right-handed), signed to point "up"
    # out of the playfield toward the camera side (negative image-y-ish).
    n = unit(cross(w3, r))
    if n[1] > 0:
        n = scale(n, -1.0)

    z_axis = (0.0, 0.0, 1.0)
    # Camera pitch vs the playfield plane: 0 = grazing, 90 = looking straight
    # down at the playfield.
    pitch = 90.0 - math.degrees(math.acos(max(-1.0, min(1.0, abs(dot(z_axis, n))))))
    # Yaw: optical axis projected into the playfield plane vs the rail dir.
    zp = sub(z_axis, scale(n, dot(z_axis, n)))
    zp = unit(zp)
    yaw = math.degrees(math.atan2(dot(cross(r, zp), n), dot(zp, r)))
    # Roll: angle of the horizon (line joining both VPs) vs the image x-axis.
    h = cross(vd, vw)
    roll = math.degrees(math.atan2(-h[0], h[1]))
    if roll > 90.0:
        roll -= 180.0
    if roll < -90.0:
        roll += 180.0

    # Distance to the lockbar (screen edge, between the rails).
    pl, pr = pts["screen_left"], pts["screen_right"]
    w_px = math.hypot(pr[0] - pl[0], pr[1] - pl[1])
    z_lock = f * W_mm / w_px  # camera-Z depth, fronto-parallel approximation

    # Lockbar centre ray -> 3D, then decompose in the cab basis.
    m = ((pl[0] + pr[0]) / 2.0, (pl[1] + pr[1]) / 2.0)
    m3 = (z_lock * (m[0] - cx) / f, z_lock * (m[1] - cy) / f, z_lock)
    lateral = dot(m3, w3)  # cab centreline appears at +lateral along X_cab
    height = abs(dot(m3, n))  # distance to the plane through the lockbar top

    return {
        "ortho_deg": ortho_deg,
        "pitch_deg": pitch,
        "yaw_deg": yaw,
        "roll_deg": roll,
        "w_px": w_px,
        "z_lock_mm": z_lock,
        "cam_lateral_mm": -lateral,  # camera position vs cab centreline
        "cam_height_mm": height,
        "lockbar_mid_px": m,
        "screen_left": pl,
        "screen_right": pr,
    }


# ------------------------------------------------------ minimal 16-bit PNG reader


def read_png_gray16(path):
    """Non-interlaced 16-bit grayscale PNG -> (w, h, [row][col] u16)."""
    d = open(path, "rb").read()
    assert d[:8] == b"\x89PNG\r\n\x1a\n", "not a PNG"
    off, idat, w, h = 8, b"", 0, 0
    while off < len(d):
        ln, typ = struct.unpack(">I4s", d[off : off + 8])
        chunk = d[off + 8 : off + 8 + ln]
        if typ == b"IHDR":
            w, h, bd, ct, _, _, il = struct.unpack(">IIBBBBB", chunk)
            assert bd == 16 and ct == 0 and il == 0, f"not gray16: bd={bd} ct={ct}"
        elif typ == b"IDAT":
            idat += chunk
        elif typ == b"IEND":
            break
        off += 12 + ln
    raw = zlib.decompress(idat)
    stride = w * 2
    rows, prev = [], bytearray(stride)
    p = 0
    for _ in range(h):
        filt = raw[p]
        line = bytearray(raw[p + 1 : p + 1 + stride])
        p += 1 + stride
        if filt == 1:  # Sub
            for i in range(2, stride):
                line[i] = (line[i] + line[i - 2]) & 0xFF
        elif filt == 2:  # Up
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif filt == 3:  # Average
            for i in range(stride):
                a = line[i - 2] if i >= 2 else 0
                line[i] = (line[i] + ((a + prev[i]) >> 1)) & 0xFF
        elif filt == 4:  # Paeth
            for i in range(stride):
                a = line[i - 2] if i >= 2 else 0
                b = prev[i]
                c = prev[i - 2] if i >= 2 else 0
                pa, pb, pc = abs(b - c), abs(a - c), abs(a + b - 2 * c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[i] = (line[i] + pr) & 0xFF
        rows.append([(line[2 * i] << 8) | line[2 * i + 1] for i in range(w)])
        prev = line
    return w, h, rows


def median_depth_mm(rows, w, h, x, y, half=10):
    """Median of the valid (>0) u16 mm values in a (2*half+1)^2 window."""
    xs, ys = int(round(x)), int(round(y))
    vals = []
    for v in range(max(0, ys - half), min(h, ys + half + 1)):
        for u in range(max(0, xs - half), min(w, xs + half + 1)):
            z = rows[v][u]
            if z > 0:
                vals.append(z)
    if not vals:
        return None, 0
    vals.sort()
    return vals[len(vals) // 2], len(vals)


# ------------------------------------------------------------------ capture table

# Per-capture colour intrinsics (annotation frame) and colour->depth remap
# parameters. Values read from the repo, not guessed:
#   v1 colour ~525 px @640x480 (demo color_focal_px); depth fx=580, c=(320,240)
#     (crates/freenect consts).
#   v2 colour ~1081 px @1920x1080 (demo color_focal_px); depth/IR intrinsics as
#     logged by the device: fx=366.5636, cx=261.2549, cy=204.3954 @512x424.
#   webcam: no factory focal (VP estimate vs the demo's 0.9*width nominal).
CAPTURES = {
    "kinect-v1": {
        "color_f": 525.0,
        "depth": {"f": 580.0, "cx": 320.0, "cy": 240.0},
    },
    "kinect-v2": {
        "color_f": 1081.0,
        "depth": {"f": 366.5636, "cx": 261.2549, "cy": 204.3954},
    },
    "webcam": {"color_f": None, "depth": None},
}


def backend_of(name):
    for key in CAPTURES:
        if key in name:
            return key
    return "webcam"


def color_to_depth(pt, cx, cy, f_col, dp):
    """Angular remap colour px -> depth px (parallel axes, zero baseline)."""
    return (
        dp["cx"] + dp["f"] * (pt[0] - cx) / f_col,
        dp["cy"] + dp["f"] * (pt[1] - cy) / f_col,
    )


# --------------------------------------------------------------------------- main


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", required=True)
    ap.add_argument("--images", required=True)
    ap.add_argument("--lockbar-mm", type=float, default=610.0)
    args = ap.parse_args()

    data = json.load(open(args.json))
    for name, rec in data["images"].items():
        an = rec["annotations"]
        W, H = rec["width"], rec["height"]
        cx, cy = W / 2.0, H / 2.0
        backend = backend_of(name)
        cap = CAPTURES[backend]
        pts = six_points(an, H)

        print("=" * 78)
        print(f"{name}  ({W}x{H}, {backend})")
        for k, p in pts.items():
            print(f"    {k:13s} ({p[0]:7.1f}, {p[1]:7.1f})")

        # --- focal from VPs, with conditioning
        f_vp, sigma = focal_with_sigma(an, cx, cy)
        if f_vp is None:
            print("  VP focal: DEGENERATE (width VP at/near infinity)")
        elif sigma is None:
            print(
                f"  VP focal: nominal {f_vp:.0f} px but +-0.2 deg line noise makes it"
                " DEGENERATE -> unusable (near-parallel lockbar lines)"
            )
            f_vp = None  # do not use it for a pose run
        else:
            rel = sigma / f_vp * 100.0
            verdict = "usable" if rel < 15.0 else "ill-conditioned"
            print(f"  VP focal: {f_vp:7.1f} px +- {sigma:.0f} ({rel:.0f}%) -> {verdict}")

        # --- pose with the best focal available per backend
        runs = []
        if cap["color_f"]:
            runs.append((f"factory f={cap['color_f']:.0f}", cap["color_f"]))
        if f_vp is not None and sigma is not None:
            runs.append((f"VP f={f_vp:.0f}", f_vp))
        if backend == "webcam":
            runs.append((f"nominal f={0.9 * W:.0f} (0.9*width)", 0.9 * W))

        best = None
        for label, f in runs:
            po = pose_given_f(an, pts, f, cx, cy, args.lockbar_mm)
            if best is None:
                best = po
            print(
                f"  [{label:24s}] rail/lockbar 3D angle {po['ortho_deg']:6.1f} deg "
                f"(90 = f is right)"
            )
            print(
                f"      pitch {po['pitch_deg']:5.1f}  yaw {po['yaw_deg']:+6.1f}  "
                f"roll {po['roll_deg']:+5.1f} deg"
            )
            print(
                f"      lockbar {po['w_px']:.0f} px -> Z {po['z_lock_mm']:.0f} mm | "
                f"cam lateral {po['cam_lateral_mm']:+.0f} mm | "
                f"height above lockbar-top plane {po['cam_height_mm']:.0f} mm"
            )

        # --- ground-truth depth check (Kinects)
        if cap["depth"]:
            stem = name.replace("_raw.png", "_depth.png")
            dpath = os.path.join(args.images, stem)
            if os.path.exists(dpath):
                dw, dh, rows = read_png_gray16(dpath)
                zs = []
                for corner in ("screen_left", "screen_right"):
                    du, dv = color_to_depth(best[corner], cx, cy, runs[0][1], cap["depth"])
                    z, nvals = median_depth_mm(rows, dw, dh, du, dv)
                    zs.append(z)
                    zt = f"{z} mm ({nvals} px)" if z else "no valid depth"
                    print(
                        f"  depth@{corner:12s} colour({best[corner][0]:6.1f},"
                        f"{best[corner][1]:6.1f}) -> depth({du:5.1f},{dv:5.1f}) = {zt}"
                    )
                valid = [z for z in zs if z]
                if valid:
                    z_meas = sum(valid) / len(valid)
                    z_est = best["z_lock_mm"]
                    err = (z_est - z_meas) / z_meas * 100.0
                    print(
                        f"  lockbar Z: W-derived {z_est:.0f} mm vs depth-measured "
                        f"{z_meas:.0f} mm -> error {err:+.1f}%"
                    )
                    if len(valid) == 2 and abs(valid[0] - valid[1]) > 0.05 * min(valid):
                        errs = [(z_est - z) / z * 100.0 for z in valid]
                        print(
                            f"      WARNING corners disagree ({valid[0]} vs {valid[1]} mm)"
                            f" -> per-corner error {errs[0]:+.1f}% / {errs[1]:+.1f}%;"
                            " the zero-baseline remap likely pushed one window off the"
                            " bar — trust the closer corner"
                        )
                    print(
                        "      (depth remap ignores the colour/depth lens baseline"
                        " — sanity check, not registration)"
                    )
            else:
                print(f"  depth check skipped: {stem} not found")

        # Note: the collinearity residual of the 3 points per rail is zero BY
        # CONSTRUCTION for hand-annotated lines (all derived from one line);
        # it only becomes a quality metric on model-predicted points.


if __name__ == "__main__":
    main()
