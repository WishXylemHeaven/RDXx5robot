#!/usr/bin/env bash
set -euo pipefail

WS="${DIGUA_WS:-/home/sunrise/digua_ws}"
CURRENT_MAP_FILE="$WS/digua_maps/current_map_name.txt"

if [ ! -f "$CURRENT_MAP_FILE" ]; then
  echo "[ERROR] current_map_name.txt not found: $CURRENT_MAP_FILE"
  exit 1
fi

MAP_NAME="$(cat "$CURRENT_MAP_FILE" | tr -d ' \r\n\t')"

echo "MAP_NAME=$MAP_NAME"
echo
echo "RTABMAP_DB:"
echo "$WS/digua_maps/rtabmap/${MAP_NAME}.db"
ls -lh "$WS/digua_maps/rtabmap/${MAP_NAME}.db" 2>/dev/null || true

echo
echo "NAV2_MAP:"
echo "$WS/digua_maps/nav2/${MAP_NAME}_map.yaml"
echo "$WS/digua_maps/nav2/${MAP_NAME}_map.pgm"
ls -lh "$WS/digua_maps/nav2/${MAP_NAME}_map.yaml" 2>/dev/null || true
ls -lh "$WS/digua_maps/nav2/${MAP_NAME}_map.pgm" 2>/dev/null || true

echo
echo "SEMANTIC_MAP:"
echo "$WS/digua_maps/semantic_slam/$MAP_NAME/semantic_map.json"
ls -lh "$WS/digua_maps/semantic_slam/$MAP_NAME/semantic_map.json" 2>/dev/null || true

echo
echo "MAP_SET:"
echo "$WS/digua_maps/map_sets/$MAP_NAME"
find "$WS/digua_maps/map_sets/$MAP_NAME" -maxdepth 1 -type f 2>/dev/null | sort || true
