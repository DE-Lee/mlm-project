#!/bin/bash
# 맵 PGM → PNG 변환 스크립트
# 웹 브라우저는 PGM 포맷을 렌더링할 수 없으므로 PNG로 변환 필요

set -e

# 기본 경로
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPS_DIR="${SCRIPT_DIR}/ros2_ws/src/slam/maps"
WEB_MAPS_DIR="${SCRIPT_DIR}/../web/public/maps"

# 인자 체크
if [ $# -eq 0 ]; then
    MAP_NAME="mlm_map"
    echo "Usage: $0 [map_name]"
    echo "No map name provided, using default: ${MAP_NAME}"
else
    MAP_NAME="$1"
fi

PGM_FILE="${MAPS_DIR}/${MAP_NAME}.pgm"
PNG_FILE="${MAPS_DIR}/${MAP_NAME}.png"

# PGM 파일 존재 확인
if [ ! -f "${PGM_FILE}" ]; then
    echo "Error: ${PGM_FILE} not found!"
    exit 1
fi

# ImageMagick 설치 확인
if ! command -v convert &> /dev/null; then
    echo "ImageMagick is not installed. Installing..."
    sudo apt update
    sudo apt install -y imagemagick
fi

# PGM → PNG 변환
echo "Converting ${PGM_FILE} to PNG..."
convert "${PGM_FILE}" "${PNG_FILE}"

# 웹 public/maps 디렉토리 생성
mkdir -p "${WEB_MAPS_DIR}"

# PNG 파일을 웹 디렉토리로 복사
echo "Copying ${PNG_FILE} to ${WEB_MAPS_DIR}/"
cp "${PNG_FILE}" "${WEB_MAPS_DIR}/"

echo "✓ Map conversion complete!"
echo "  - PNG file: ${PNG_FILE}"
echo "  - Web copy: ${WEB_MAPS_DIR}/${MAP_NAME}.png"
echo ""
echo "Note: Make sure web/src/config/robots.ts has the correct map path:"
echo "  image: '/maps/${MAP_NAME}.png'"
