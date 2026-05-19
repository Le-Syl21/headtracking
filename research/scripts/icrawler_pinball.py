#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "icrawler>=0.6.7",
#     "pillow>=10.0",
# ]
# ///
"""Bulk-fetch pinball photos via icrawler (Bing) into dataset/raw/.

Multiple keyword queries cover the angles we want:
  * 3/4 cabinet view (player perspective, lockbar slope visible)
  * top-down playfield (lockbar as a near-horizontal bar at bottom)
  * front view (lockbar centered)
  * DIY / pincab builds (varied finishes)
  * lockbar close-ups (positive examples for the class)

Output goes to dataset/raw/<sha1>.jpg. The icrawler default filenames
are renamed to content-hash so re-running de-duplicates.

Usage:
    uv run scripts/icrawler_pinball.py            # default: 10/keyword
    uv run scripts/icrawler_pinball.py 50         # 50 per keyword
"""
from __future__ import annotations

import hashlib
import shutil
import sys
import tempfile
from pathlib import Path

from icrawler.builtin import BingImageCrawler
from PIL import Image, UnidentifiedImageError

HERE = Path(__file__).parent
DATASET_RAW = HERE.parent / "dataset" / "raw"

# (keyword, weight) — weight controls per-keyword scaling. Final count
# per keyword = base_count × weight.
QUERIES: list[tuple[str, float]] = [
    ("pinball cabinet front view", 1.5),
    ("pinball machine lockbar", 2.0),
    ("pincab homemade lockbar", 1.5),
    ("virtual pinball cabinet build", 1.0),
    ("pinball machine playfield top down", 0.7),
    ("pinball flipper machine arcade", 1.0),
    ("stern pinball cabinet", 1.0),
    ("williams pinball cabinet", 1.0),
    ("custom pincab DIY", 1.0),
]

MIN_DIM = 400  # discard avatars/icons/sprites


def hash_rename(src: Path) -> Path | None:
    """Validate image + rename to content-hash. Return final path or
    None if rejected."""
    try:
        with Image.open(src) as img:
            img.verify()
        with Image.open(src) as img:
            if min(img.size) < MIN_DIM:
                return None
            fmt = img.format
    except (UnidentifiedImageError, OSError):
        return None
    suffix = {"JPEG": ".jpg", "PNG": ".png", "WEBP": ".webp"}.get(fmt)
    if suffix is None:
        return None
    data = src.read_bytes()
    h = hashlib.sha1(data).hexdigest()[:12]
    dest = DATASET_RAW / f"{h}{suffix}"
    if dest.exists():
        return None  # dedup
    shutil.move(str(src), str(dest))
    return dest


def crawl_one(keyword: str, max_num: int) -> int:
    """Crawl Bing for `keyword`, drop into dataset/raw/."""
    with tempfile.TemporaryDirectory(prefix="icrawler_") as tmp:
        crawler = BingImageCrawler(
            downloader_threads=2,
            storage={"root_dir": tmp},
            log_level="WARNING",
        )
        crawler.crawl(
            keyword=keyword,
            max_num=max_num,
            min_size=(MIN_DIM, MIN_DIM),
        )
        saved = 0
        for src in Path(tmp).iterdir():
            if hash_rename(src) is not None:
                saved += 1
        return saved


def main(argv: list[str]) -> int:
    base = int(argv[1]) if len(argv) > 1 else 10
    DATASET_RAW.mkdir(parents=True, exist_ok=True)
    print(f"Target ≈ {sum(int(base * w) for _, w in QUERIES)} candidate images "
          f"({base} × weighted, post-dedup may be less)\n")
    total = 0
    for kw, weight in QUERIES:
        n = max(1, int(base * weight))
        print(f"[{kw}] n={n}")
        try:
            saved = crawl_one(kw, n)
        except Exception as e:  # icrawler can throw on transient net errors
            print(f"  failed: {e}")
            continue
        total += saved
        print(f"  saved {saved} new images\n")
    print(f"Done. {total} new image(s) in {DATASET_RAW}")
    print(f"Total in dataset/raw/: {len(list(DATASET_RAW.iterdir()))} file(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
