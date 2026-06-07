#!/usr/bin/env python3
from pathlib import Path
import os


DEFAULT_WS = Path(os.environ.get("DIGUA_WS", "/home/sunrise/digua_ws"))

CURRENT_MAP_FILE = DEFAULT_WS / "digua_maps/current_map_name.txt"
SEMANTIC_SLAM_ROOT = DEFAULT_WS / "digua_maps/semantic_slam"
LEGACY_SEMANTIC_MAP = DEFAULT_WS / "digua_maps/semantic/semantic_map.json"


def read_current_map_name():
    if not CURRENT_MAP_FILE.exists():
        raise FileNotFoundError(
            f"current_map_name.txt not found: {CURRENT_MAP_FILE}. "
            "Please run rtabmap_online.launch.py first, or pass an explicit semantic_map_path."
        )

    name = CURRENT_MAP_FILE.read_text(encoding="utf-8").strip()

    if not name:
        raise RuntimeError(
            f"current_map_name.txt is empty: {CURRENT_MAP_FILE}. "
            "Please run rtabmap_online.launch.py first, or pass an explicit semantic_map_path."
        )

    return name


def current_semantic_map_path():
    map_name = read_current_map_name()
    return SEMANTIC_SLAM_ROOT / map_name / "semantic_map.json"


def resolve_semantic_map_path(value=None):
    """
    Resolve semantic map path.

    Supported values:
      current / auto / empty / None:
        digua_maps/semantic_slam/<current_map_name>/semantic_map.json

      legacy:
        digua_maps/semantic/semantic_map.json

      any other value:
        treated as explicit path
    """
    if value is None:
        return current_semantic_map_path()

    text = str(value).strip()

    if text == "" or text.lower() in ("current", "auto", "__current__"):
        return current_semantic_map_path()

    if text.lower() in ("legacy", "old", "default"):
        return LEGACY_SEMANTIC_MAP

    return Path(os.path.expanduser(text))
