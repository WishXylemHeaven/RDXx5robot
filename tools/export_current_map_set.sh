#!/usr/bin/env bash
set -eo pipefail

WS="${DIGUA_WS:-/home/sunrise/digua_ws}"
CURRENT_MAP_FILE="$WS/digua_maps/current_map_name.txt"

if [ ! -f "$CURRENT_MAP_FILE" ]; then
  echo "[ERROR] current_map_name.txt not found: $CURRENT_MAP_FILE"
  echo "Please start rtabmap_online.launch.py first."
  exit 1
fi

MAP_NAME="$(cat "$CURRENT_MAP_FILE" | tr -d ' \r\n\t')"

if [ -z "$MAP_NAME" ]; then
  echo "[ERROR] current_map_name.txt is empty."
  exit 1
fi

SEMANTIC_MAP="$WS/digua_maps/semantic_slam/$MAP_NAME/semantic_map.json"
MAP_YAML="$WS/digua_maps/nav2/${MAP_NAME}_map.yaml"
RTABMAP_DB="$WS/digua_maps/rtabmap/${MAP_NAME}.db"
NAMED_POSES="$WS/digua_navigation_data/named_poses.yaml"

echo "==== export current map set ===="
echo "MAP_NAME     = $MAP_NAME"
echo "SEMANTIC_MAP = $SEMANTIC_MAP"
echo "MAP_YAML     = $MAP_YAML"
echo "RTABMAP_DB   = $RTABMAP_DB"
echo "NAMED_POSES  = $NAMED_POSES"
echo "==============================="

source /opt/tros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 run digua_semantic_mapping semantic_map_tool \
  --map "$SEMANTIC_MAP" \
  --export "$MAP_NAME" \
  --map-yaml "$MAP_YAML" \
  --rtabmap-db "$RTABMAP_DB" \
  --named-poses "$NAMED_POSES"
