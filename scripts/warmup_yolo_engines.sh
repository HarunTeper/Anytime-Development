#!/bin/bash
# Pre-build all TensorRT engines for YOLO experiments.
# Run once after downloading weights or changing GPU/TensorRT version.
#
# Usage: ./scripts/warmup_yolo_engines.sh [--force-rebuild]
#
# Options:
#   --force-rebuild   Delete existing .engine files before building

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
WEIGHTS_DIR="${WORKSPACE_DIR}/packages/src/anytime_yolo/weights_32"

FORCE_REBUILD=false
for arg in "$@"; do
    case $arg in
        --force-rebuild) FORCE_REBUILD=true ;;
        *) echo "Unknown option: $arg"; exit 1 ;;
    esac
done

echo "Warming up YOLO TensorRT engines..."
if [ "${FORCE_REBUILD}" = true ]; then
    ros2 run anytime_yolo warmup_engines "${WEIGHTS_DIR}" --force-rebuild
else
    ros2 run anytime_yolo warmup_engines "${WEIGHTS_DIR}"
fi
echo "Engine warmup complete."
