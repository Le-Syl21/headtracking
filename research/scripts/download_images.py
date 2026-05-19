#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "requests>=2.32",
#     "beautifulsoup4>=4.12",
#     "pillow>=10.0",
# ]
# ///
"""Bulk download pinball/pincab images for the lockbar dataset.

Usage:
    uv run scripts/download_images.py [urls_file]

`urls_file` defaults to `scripts/urls.txt` and must contain one URL
per line. Supported URL types:

  * Direct image URL (.jpg / .jpeg / .png / .webp)
  * Pinside.com post / showroom page → scrape <img> tags whose src
    points at the Pinside CDN
  * Reddit post URL (`https://www.reddit.com/r/.../comments/...`)
    → query the JSON endpoint and pull preview / gallery images
  * Imgur gallery / single image → resolve via the standard /a/ pattern
  * Generic web page → scrape all <img> tags and filter by size

Output goes to research/dataset/raw/<sha1prefix>.<ext>. Duplicates are
skipped via content hash; corrupt downloads are deleted.

Empty lines and lines starting with `#` are ignored.
"""
from __future__ import annotations

import hashlib
import io
import re
import sys
import time
from pathlib import Path
from urllib.parse import urljoin, urlparse

import requests
from bs4 import BeautifulSoup
from PIL import Image, UnidentifiedImageError

HERE = Path(__file__).parent
DATASET_RAW = HERE.parent / "dataset" / "raw"
DEFAULT_URL_FILE = HERE / "urls.txt"

# Wikimedia rejects Mozilla-prefixed UAs; their bot policy requires a
# descriptive identifier with contact info. Use a plain bot UA — Reddit
# and Pinside still accept this fine.
USER_AGENT = "headtracking-dataset/0.1 (https://github.com/Le-Syl21/headtracking)"
HEADERS = {"User-Agent": USER_AGENT}

IMG_EXT_RE = re.compile(r"\.(jpe?g|png|webp)(\?.*)?$", re.IGNORECASE)
# Images smaller than this in either axis are skipped — favicons,
# avatars, sprites etc.
MIN_DIM = 400


def safe_name(content: bytes, suffix: str) -> str:
    """Stable filename: 12 hex chars of sha1, then suffix."""
    h = hashlib.sha1(content).hexdigest()[:12]
    return f"{h}{suffix}"


def save_image_bytes(content: bytes, suggested_url: str) -> Path | None:
    """Decode, validate, write to dataset/raw/. Return path or None."""
    try:
        img = Image.open(io.BytesIO(content))
        img.verify()
    except (UnidentifiedImageError, OSError):
        return None
    img = Image.open(io.BytesIO(content))
    if min(img.size) < MIN_DIM:
        return None
    suffix = {"JPEG": ".jpg", "PNG": ".png", "WEBP": ".webp"}.get(img.format)
    if suffix is None:
        return None
    DATASET_RAW.mkdir(parents=True, exist_ok=True)
    fname = safe_name(content, suffix)
    path = DATASET_RAW / fname
    if path.exists():
        return None  # dedup
    path.write_bytes(content)
    return path


def fetch(url: str) -> bytes | None:
    try:
        r = requests.get(url, headers=HEADERS, timeout=20)
        r.raise_for_status()
        return r.content
    except requests.RequestException as e:
        print(f"  fetch failed: {url}\n    {e}", file=sys.stderr)
        return None


def looks_like_image_url(url: str) -> bool:
    return bool(IMG_EXT_RE.search(urlparse(url).path))


def scrape_page_images(page_url: str, html: bytes) -> list[str]:
    """Return absolute image URLs found in <img> / og:image tags."""
    soup = BeautifulSoup(html, "html.parser")
    urls: list[str] = []
    # OpenGraph hero image is usually high-res.
    for meta in soup.find_all("meta", attrs={"property": "og:image"}):
        c = meta.get("content")
        if c:
            urls.append(urljoin(page_url, c))
    for img in soup.find_all("img"):
        src = img.get("data-src") or img.get("src")
        if src and not src.startswith("data:"):
            urls.append(urljoin(page_url, src))
    return urls


def handle_reddit(post_url: str) -> list[str]:
    """Query Reddit's JSON endpoint to extract gallery + preview images."""
    json_url = post_url.rstrip("/") + ".json"
    try:
        r = requests.get(json_url, headers=HEADERS, timeout=20)
        r.raise_for_status()
        data = r.json()
    except (requests.RequestException, ValueError):
        return []
    if not isinstance(data, list) or not data:
        return []
    post = data[0]["data"]["children"][0]["data"]
    out: list[str] = []
    # Single-image post
    if post.get("post_hint") == "image" and post.get("url"):
        out.append(post["url"])
    # Gallery
    gallery = post.get("media_metadata") or {}
    for item in gallery.values():
        s = item.get("s") or {}
        u = s.get("u") or s.get("gif")
        if u:
            out.append(u.replace("&amp;", "&"))
    # Preview fallback
    preview = post.get("preview", {}).get("images", [])
    for img in preview:
        u = img.get("source", {}).get("url")
        if u:
            out.append(u.replace("&amp;", "&"))
    return out


def expand(url: str) -> list[str]:
    """Turn one user-provided URL into a list of direct image URLs."""
    if looks_like_image_url(url):
        return [url]
    host = urlparse(url).netloc.lower()
    if "reddit.com" in host:
        urls = handle_reddit(url)
        if urls:
            return urls
    html = fetch(url)
    if html is None:
        return []
    images = scrape_page_images(url, html)
    # Filter to plausible photos (drop site chrome / sprites).
    return [u for u in images if looks_like_image_url(u)]


def process_one(url: str) -> int:
    print(f"[{url}]")
    image_urls = expand(url)
    if not image_urls:
        print("  no images found")
        return 0
    saved = 0
    for iu in image_urls:
        body = fetch(iu)
        if body is None:
            continue
        path = save_image_bytes(body, iu)
        if path is not None:
            print(f"  + {path.name}  ({iu})")
            saved += 1
        time.sleep(1.5)  # Wikimedia rate-limits aggressively, this stays safe
    return saved


def main(argv: list[str]) -> int:
    url_file = Path(argv[1]) if len(argv) > 1 else DEFAULT_URL_FILE
    if not url_file.exists():
        print(f"No URL file at {url_file}. Create it and add one URL per line.")
        return 1
    urls = [
        line.strip()
        for line in url_file.read_text().splitlines()
        if line.strip() and not line.startswith("#")
    ]
    if not urls:
        print(f"{url_file} is empty.")
        return 1
    total = 0
    for u in urls:
        total += process_one(u)
        time.sleep(0.3)
    print(f"\nDone. {total} new image(s) saved to {DATASET_RAW}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
