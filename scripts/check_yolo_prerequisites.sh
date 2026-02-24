#!/bin/bash
# Check YOLO prerequisites (weights + images) before running experiments.
# Exits 1 with download instructions if any prerequisites are missing.
#
# Usage: ./scripts/check_yolo_prerequisites.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
WEIGHTS_DIR="${WORKSPACE_DIR}/packages/src/anytime_yolo/weights_32"
IMAGES_DIR="${WORKSPACE_DIR}/packages/src/video_publisher/images"

errors=0

# Check weights
if [ ! -f "${WEIGHTS_DIR}/model.json" ]; then
    echo "ERROR: YOLO weights not found at ${WEIGHTS_DIR}/"
    echo "       Expected file: model.json"
    echo ""
    echo "Download weights:"
    echo "  cd packages/src/anytime_yolo"
    echo "  wget https://tu-dortmund.sciebo.de/s/gmGSJEsFgwKb6MY/download -O weights.zip"
    echo "  unzip -o weights.zip"
    echo "  rm weights.zip"
    echo ""
    errors=1
fi

# Check images
image_count=$(ls "${IMAGES_DIR}"/image_*.jpg 2>/dev/null | wc -l)
if [ "${image_count}" -eq 0 ]; then
    echo "ERROR: Test images not found at ${IMAGES_DIR}/"
    echo "       Expected files: image_*.jpg"
    echo ""
    echo "Download images:"
    echo "  cd packages/src/video_publisher"
    echo "  wget https://tu-dortmund.sciebo.de/s/BQRaiztJkmx33tt/download -O images.zip"
    echo "  unzip -o images.zip"
    echo "  rm images.zip"
    echo ""
    errors=1
fi

if [ "${errors}" -ne 0 ]; then
    echo "See ARTIFACT_EVALUATION.md section 'YOLO Prerequisites' for details."
    exit 1
fi

echo "YOLO prerequisites OK (weights: ${WEIGHTS_DIR}, images: ${image_count} files)"
