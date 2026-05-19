#!/usr/bin/env bash
# Launch Label Studio locally on http://localhost:8080
# DB and uploads live under research/label-studio/data/ so we can
# wipe and restart cleanly. Sign-up is free on first visit (local
# only).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LABEL_STUDIO_BASE_DATA_DIR="${HERE}/data"
export LABEL_STUDIO_DISABLE_SIGNUP_WITHOUT_LINK=false
# Disable telemetry & version checks for a quieter start.
export LABEL_STUDIO_DISABLE_METRICS=true

mkdir -p "${LABEL_STUDIO_BASE_DATA_DIR}"

# Allow Label Studio to read images from anywhere under research/
# without copying them into its internal upload area.
export LABEL_STUDIO_LOCAL_FILES_SERVING_ENABLED=true
export LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT="$(cd "${HERE}/.." && pwd)"

echo "Label Studio data dir: ${LABEL_STUDIO_BASE_DATA_DIR}"
echo "Serving local files from: ${LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT}"
echo "Open http://localhost:8080 — create an account on first run."
echo "Use the XML config in $(realpath "${HERE}/labeling_config.xml")"

# Label Studio's deps (django-environ) use pkgutil.find_loader which
# was removed in Python 3.12+. Pin to 3.11 — uv will download it
# transparently on first run.
exec uvx --python 3.11 --from label-studio label-studio start
