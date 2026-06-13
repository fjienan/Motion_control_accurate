#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR="${1:-output/pid_debug}"

if [ ! -d "${OUTPUT_DIR}" ]; then
  echo "No debug output directory found: ${OUTPUT_DIR}"
  echo "Run a MoveToPose action first, then check again."
  exit 1
fi

echo "Latest PID debug plots in ${OUTPUT_DIR}:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.png' -printf '%T@ %p\n' \
  | sort -nr \
  | head -n 10 \
  | cut -d' ' -f2-
