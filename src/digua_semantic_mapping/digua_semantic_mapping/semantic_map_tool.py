#!/usr/bin/env python3
import argparse
import json
import shutil
import time
from collections import Counter, defaultdict
from pathlib import Path
from digua_semantic_mapping.semantic_paths import resolve_semantic_map_path


DEFAULT_WS = Path("/home/sunrise/digua_ws")
DEFAULT_MAP_PATH = "current"
DEFAULT_BACKUP_DIR = DEFAULT_WS / "digua_maps/semantic/backups"
DEFAULT_EXPORT_DIR = DEFAULT_WS / "digua_maps/map_sets"
DEFAULT_NAMED_POSES = DEFAULT_WS / "digua_navigation_data/named_poses.yaml"


def load_map(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"semantic map not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if "objects" not in data:
        data["objects"] = []

    return data


def save_map(path: Path, data: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    data["updated_at"] = time.time()

    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=4)
        f.write("\n")


def backup_map(path: Path, backup_dir: Path):
    if not path.exists():
        raise FileNotFoundError(f"semantic map not found: {path}")

    backup_dir.mkdir(parents=True, exist_ok=True)

    ts = time.strftime("%Y%m%d_%H%M%S")
    backup_path = backup_dir / f"semantic_map_backup_{ts}.json"

    shutil.copy2(path, backup_path)
    return backup_path


def print_object(obj, prefix=""):
    obj_id = obj.get("id")
    label = obj.get("label", "")
    status = obj.get("status", "")
    obs = int(obj.get("observations", 0))
    conf = float(obj.get("confidence", 0.0))
    x = float(obj.get("x", 0.0))
    y = float(obj.get("y", 0.0))
    z = float(obj.get("z", 0.0))
    first_seen = obj.get("first_seen", "")
    last_seen = obj.get("last_seen", "")

    print(
        f"{prefix}"
        f"id={obj_id}  "
        f"label={label}  "
        f"status={status}  "
        f"obs={obs}  "
        f"conf={conf:.3f}  "
        f"xyz=({x:.3f}, {y:.3f}, {z:.3f})  "
        f"first_seen={first_seen}  "
        f"last_seen={last_seen}"
    )


def show_summary(data: dict):
    objects = data.get("objects", [])
    print("==== semantic map summary ====")
    print(f"map_frame : {data.get('map_frame', '')}")
    print(f"version   : {data.get('version', '')}")
    print(f"updated_at: {data.get('updated_at', '')}")
    print(f"next_id   : {data.get('next_id', '')}")
    print(f"objects   : {len(objects)}")

    by_status = Counter(str(o.get("status", "unknown")) for o in objects)
    by_label = Counter(str(o.get("label", "unknown")) for o in objects)

    print("\n-- by status --")
    for k, v in sorted(by_status.items()):
        print(f"{k}: {v}")

    print("\n-- by label --")
    for k, v in sorted(by_label.items()):
        print(f"{k}: {v}")

    confirmed = [o for o in objects if str(o.get("status", "")).lower() == "confirmed"]
    candidates = [o for o in objects if str(o.get("status", "")).lower() == "candidate"]

    print(f"\nconfirmed : {len(confirmed)}")
    print(f"candidate : {len(candidates)}")
    print("==============================\n")


def list_objects(data: dict, label_filter=None, include_candidates=True):
    objects = data.get("objects", [])

    if label_filter:
        label_filter = str(label_filter).strip().lower()
        objects = [
            o for o in objects
            if str(o.get("label", "")).strip().lower() == label_filter
        ]

    if not include_candidates:
        objects = [
            o for o in objects
            if str(o.get("status", "")).strip().lower() == "confirmed"
        ]

    objects = sorted(
        objects,
        key=lambda o: (
            str(o.get("label", "")),
            int(o.get("id", 999999))
        )
    )

    print("==== semantic objects ====")
    if not objects:
        print("No objects matched.")
        print("==========================")
        return

    for obj in objects:
        print_object(obj)

    print("==========================\n")


def group_objects(data: dict):
    objects = data.get("objects", [])
    groups = defaultdict(list)

    for obj in objects:
        groups[str(obj.get("label", "unknown"))].append(obj)

    print("==== semantic objects grouped by label ====")
    for label in sorted(groups.keys()):
        arr = sorted(groups[label], key=lambda o: int(o.get("id", 999999)))
        print(f"\n[{label}] count={len(arr)}")
        for obj in arr:
            print_object(obj, prefix="  ")
    print("\n===========================================\n")


def filter_objects(data: dict, remove_func):
    old_objects = data.get("objects", [])
    kept = []
    removed = []

    for obj in old_objects:
        if remove_func(obj):
            removed.append(obj)
        else:
            kept.append(obj)

    new_data = dict(data)
    new_data["objects"] = kept
    return new_data, removed


def copy_file_if_exists(src: Path, dst_dir: Path, dst_name: str = None):
    if not src:
        return None

    src = Path(src)
    if not src.exists():
        return None

    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / (dst_name if dst_name else src.name)
    shutil.copy2(src, dst)
    return dst


def parse_map_yaml_image(map_yaml: Path):
    """
    轻量解析 ROS map.yaml 里的 image 字段。
    支持：
      image: map.pgm
      image: ./map.pgm
      image: /absolute/path/map.pgm
    """
    if not map_yaml or not map_yaml.exists():
        return None

    for raw in map_yaml.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw.strip()
        if not line.startswith("image:"):
            continue

        value = line.split(":", 1)[1].strip().strip('"').strip("'")
        if not value:
            return None

        image_path = Path(value)
        if image_path.is_absolute():
            return image_path

        return map_yaml.parent / image_path

    return None


def export_map(
    data_path: Path,
    export_root: Path,
    export_name: str,
    named_poses_path: Path = None,
    map_yaml_path: Path = None,
    rtabmap_db_path: Path = None,
    extra_paths=None,
):
    ts = time.strftime("%Y%m%d_%H%M%S")

    if export_name:
        folder_name = export_name
    else:
        folder_name = f"map_set_{ts}"

    export_dir = export_root / folder_name
    export_dir.mkdir(parents=True, exist_ok=True)

    copied = []
    missing = []

    def record_copy(src, dst_name=None):
        if src is None:
            return None

        src = Path(src)
        dst = copy_file_if_exists(src, export_dir, dst_name=dst_name)
        if dst is None:
            missing.append(str(src))
            return None

        copied.append(dst)
        return dst

    semantic_dst = record_copy(data_path, "semantic_map.json")
    named_poses_dst = record_copy(named_poses_path, "named_poses.yaml") if named_poses_path else None

    map_yaml_dst = None
    map_image_dst = None
    if map_yaml_path:
        map_yaml_path = Path(map_yaml_path)
        map_yaml_dst = record_copy(map_yaml_path, "map.yaml")

        image_path = parse_map_yaml_image(map_yaml_path)
        if image_path:
            map_image_dst = record_copy(image_path, image_path.name)
        else:
            missing.append(f"image referenced by map yaml: {map_yaml_path}")

    rtabmap_dst = None
    if rtabmap_db_path:
        rtabmap_dst = record_copy(rtabmap_db_path, "rtabmap.db")

    extra_copied = []
    if extra_paths:
        for p in extra_paths:
            dst = record_copy(Path(p))
            if dst:
                extra_copied.append(dst)

    manifest = {
        "exported_at": time.time(),
        "exported_at_text": ts,
        "export_name": folder_name,
        "export_dir": str(export_dir),

        "source_files": {
            "semantic_map": str(data_path),
            "named_poses": str(named_poses_path) if named_poses_path else None,
            "map_yaml": str(map_yaml_path) if map_yaml_path else None,
            "map_image": str(parse_map_yaml_image(Path(map_yaml_path))) if map_yaml_path else None,
            "rtabmap_db": str(rtabmap_db_path) if rtabmap_db_path else None,
            "extra_paths": [str(p) for p in extra_paths] if extra_paths else [],
        },

        "copied_files": {
            "semantic_map": str(semantic_dst) if semantic_dst else None,
            "named_poses": str(named_poses_dst) if named_poses_dst else None,
            "map_yaml": str(map_yaml_dst) if map_yaml_dst else None,
            "map_image": str(map_image_dst) if map_image_dst else None,
            "rtabmap_db": str(rtabmap_dst) if rtabmap_dst else None,
            "extra": [str(p) for p in extra_copied],
            "all": [str(p) for p in copied],
        },

        "missing_files": missing,

        "note": (
            "This folder is a map set. semantic_map.json coordinates are meaningful "
            "only when used with the matching geometry map / RTAB-Map database."
        )
    }

    manifest_path = export_dir / "manifest.json"
    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=4)
        f.write("\n")

    copied.append(manifest_path)

    return export_dir, copied, missing

def main():
    parser = argparse.ArgumentParser(
        description="Maintain Digua robot semantic_map.json"
    )

    parser.add_argument(
        "--map",
        default=str(DEFAULT_MAP_PATH),
        help="Path to semantic_map.json"
    )

    parser.add_argument("--summary", action="store_true", help="Show map summary")
    parser.add_argument("--list", action="store_true", help="List semantic objects")
    parser.add_argument("--group", action="store_true", help="Group objects by label")
    parser.add_argument("--label", default=None, help="Filter list by label")
    parser.add_argument(
        "--confirmed-only",
        action="store_true",
        help="When listing, only show confirmed objects"
    )

    parser.add_argument("--backup", action="store_true", help="Backup semantic_map.json")
    parser.add_argument(
        "--backup-dir",
        default=str(DEFAULT_BACKUP_DIR),
        help="Backup directory"
    )

    parser.add_argument(
        "--clean-candidates",
        action="store_true",
        help="Remove candidate objects"
    )
    parser.add_argument(
        "--clean-low-observations",
        type=int,
        default=None,
        help="Remove objects with observations lower than this value"
    )
    parser.add_argument(
        "--delete-id",
        type=int,
        action="append",
        default=[],
        help="Delete object by id. Can be used multiple times."
    )

    parser.add_argument(
        "--export",
        nargs="?",
        const="",
        default=None,
        help="Export semantic map to a named folder. Example: --export patrol_001"
    )
    parser.add_argument(
        "--export-dir",
        default=str(DEFAULT_EXPORT_DIR),
        help="Export root directory"
    )
    parser.add_argument(
        "--named-poses",
        default=str(DEFAULT_NAMED_POSES),
        help="Path to named_poses.yaml to include in export if it exists"
    )
    parser.add_argument(
        "--map-yaml",
        default=None,
        help="Path to ROS 2D map yaml. The image file referenced by image: will also be copied."
    )
    parser.add_argument(
        "--rtabmap-db",
        default=None,
        help="Path to RTAB-Map .db file."
    )
    parser.add_argument(
        "--copy-extra",
        action="append",
        default=[],
        help="Extra file to copy into the exported map set. Can be used multiple times."
    )

    parser.add_argument(
        "--yes",
        action="store_true",
        help="Actually apply destructive operations. Without --yes, clean/delete only previews."
    )

    args = parser.parse_args()

    map_path = resolve_semantic_map_path(args.map)
    backup_dir = Path(args.backup_dir)
    export_root = Path(args.export_dir)
    named_poses_path = Path(args.named_poses)

    data = load_map(map_path)

    did_anything = False

    if args.summary:
        show_summary(data)
        did_anything = True

    if args.list:
        list_objects(
            data,
            label_filter=args.label,
            include_candidates=not args.confirmed_only
        )
        did_anything = True

    if args.group:
        group_objects(data)
        did_anything = True

    if args.backup:
        backup_path = backup_map(map_path, backup_dir)
        print(f"[OK] backup written: {backup_path}")
        did_anything = True

    destructive_requested = (
        args.clean_candidates
        or args.clean_low_observations is not None
        or bool(args.delete_id)
    )

    if destructive_requested:
        # 破坏性操作前自动备份一份，防止误删
        backup_path = backup_map(map_path, backup_dir)
        print(f"[OK] backup before edit: {backup_path}")

        def should_remove(obj):
            status = str(obj.get("status", "")).strip().lower()
            obs = int(obj.get("observations", 0))
            obj_id = int(obj.get("id", -1))

            if args.clean_candidates and status == "candidate":
                return True

            if args.clean_low_observations is not None and obs < args.clean_low_observations:
                return True

            if obj_id in args.delete_id:
                return True

            return False

        new_data, removed = filter_objects(data, should_remove)

        print("\n==== objects to remove ====")
        if removed:
            for obj in removed:
                print_object(obj)
        else:
            print("No objects matched removal conditions.")
        print("===========================\n")

        if args.yes:
            save_map(map_path, new_data)
            print(f"[OK] semantic map updated: {map_path}")
            print(f"[OK] removed objects: {len(removed)}")
        else:
            print("[DRY-RUN] No file changed. Add --yes to apply changes.")

        did_anything = True

    if args.export is not None:
        export_name = args.export
        export_dir, copied, missing = export_map(
            data_path=map_path,
            export_root=export_root,
            export_name=export_name,
            named_poses_path=named_poses_path,
            map_yaml_path=Path(args.map_yaml) if args.map_yaml else None,
            rtabmap_db_path=Path(args.rtabmap_db) if args.rtabmap_db else None,
            extra_paths=args.copy_extra,
        )

        print(f"[OK] exported to: {export_dir}")
        for p in copied:
            print(f"  copied: {p}")

        if missing:
            print("\n[WARN] missing files:")
            for p in missing:
                print(f"  missing: {p}")

        did_anything = True

    if not did_anything:
        show_summary(data)
        list_objects(data)


if __name__ == "__main__":
    main()
