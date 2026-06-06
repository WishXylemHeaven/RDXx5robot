#!/usr/bin/env python3
import argparse
import difflib
import json
from pathlib import Path


DEFAULT_WS = Path("/home/sunrise/digua_ws")
DEFAULT_CLASSES = DEFAULT_WS / "src/digua_bpu_yolo/config/oiv7_classes.list"
DEFAULT_ALIASES = DEFAULT_WS / "src/digua_bpu_yolo/config/oiv7_aliases.json"
DEFAULT_SEMANTIC_YAML = DEFAULT_WS / "src/digua_semantic_mapping/config/semantic_mapping.yaml"


COMMON_ALIASES = {
    "footwear": ["footwear", "boot", "sandal", "high heels"],
    "shoe": ["footwear", "boot", "sandal", "high heels"],

    "bottle": ["bottle"],

    "cup": ["coffee cup", "mug", "measuring cup"],
    "mug": ["mug", "coffee cup"],

    "chair": ["chair", "stool"],
    "table": ["table", "coffee table", "kitchen & dining room table"],
    "desk": ["desk"],

    "couch": ["couch", "loveseat", "sofa bed", "studio couch"],
    "sofa": ["couch", "loveseat", "sofa bed", "studio couch"],

    "tv": ["television"],
    "television": ["television"],
    "monitor": ["computer monitor"],

    "laptop": ["laptop"],
    "keyboard": ["computer keyboard"],
    "mouse": ["computer mouse"],

    "refrigerator": ["refrigerator"],
    "fridge": ["refrigerator"],

    "microwave": ["microwave oven"],
    "sink": ["sink"],
    "toilet": ["toilet"],

    "trash_bin": ["waste container"],
    "trash can": ["waste container"],
    "trash_can": ["waste container"],
    "waste bin": ["waste container"],
    "waste_bin": ["waste container"],
    "garbage bin": ["waste container"],
    "garbage_bin": ["waste container"],

    "plant": ["plant", "houseplant", "flowerpot"],
    "door": ["door"],
    "window": ["window"],

    "book": ["book"],
    "remote": ["remote control"],
    "clock": ["clock", "digital clock", "wall clock", "alarm clock"],
    "lamp": ["lamp"],

    "backpack": ["backpack"],
    "suitcase": ["suitcase", "luggage and bags"],

    "person": ["person", "man", "woman", "boy", "girl"],
}


def norm(s: str) -> str:
    return str(s).strip().strip('"').strip("'").lower()


def load_classes(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"classes file not found: {path}")

    classes = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            name = line.strip()
            if not name:
                continue

            # 兼容 "0: Accordion" 这种格式
            if ":" in name:
                left, right = name.split(":", 1)
                if left.strip().isdigit():
                    name = right.strip()

            classes.append(name)

    return classes


def load_aliases(path: Path):
    if not path.exists():
        return {}

    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    aliases = {}
    for k, values in data.items():
        key = norm(k)
        if not key:
            continue

        if isinstance(values, str):
            values = [values]

        vals = []
        for v in values:
            v = norm(v)
            if v and v not in vals:
                vals.append(v)

        aliases[key] = vals

    return aliases


def save_aliases(path: Path, aliases: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(aliases, f, ensure_ascii=False, indent=2)
        f.write("\n")


def sync_aliases(classes, alias_path: Path, check_only: bool):
    aliases = load_aliases(alias_path)

    before = json.dumps(aliases, ensure_ascii=False, sort_keys=True)

    # 1. 加常用 alias，不覆盖你已经手动改过的 key
    for key, values in COMMON_ALIASES.items():
        key = norm(key)
        aliases.setdefault(key, [norm(v) for v in values])

    # 2. 601 类全部自映射：Bottle -> bottle, Television -> television
    for name in classes:
        key = norm(name)
        aliases.setdefault(key, [key])

    # 3. 清理重复
    cleaned = {}
    for key, values in aliases.items():
        key = norm(key)
        vals = []
        for v in values:
            v = norm(v)
            if v and v not in vals:
                vals.append(v)
        cleaned[key] = vals

    aliases = cleaned

    after = json.dumps(aliases, ensure_ascii=False, sort_keys=True)

    changed = before != after
    if changed and not check_only:
        save_aliases(alias_path, aliases)

    return aliases, changed


def parse_class_whitelist(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"semantic yaml not found: {path}")

    labels = []
    in_whitelist = False
    whitelist_indent = None

    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.rstrip("\n")
        stripped = line.strip()

        if not stripped or stripped.startswith("#"):
            continue

        if stripped.startswith("class_whitelist:"):
            in_whitelist = True
            whitelist_indent = len(line) - len(line.lstrip(" "))
            continue

        if in_whitelist:
            indent = len(line) - len(line.lstrip(" "))

            if stripped.startswith("- "):
                label = norm(stripped[2:])
                if label:
                    labels.append(label)
                continue

            # 离开 class_whitelist 区域
            if indent <= whitelist_indent and stripped.endswith(":"):
                break

    return labels


def unique_hits(label, aliases, class_to_ids):
    candidates = [label]
    candidates.extend(aliases.get(label, []))

    seen_candidates = []
    for c in candidates:
        c = norm(c)
        if c and c not in seen_candidates:
            seen_candidates.append(c)

    hits = []
    seen_ids = set()

    for c in seen_candidates:
        for cid in class_to_ids.get(c, []):
            if cid not in seen_ids:
                hits.append((cid, c))
                seen_ids.add(cid)

    return seen_candidates, hits


def suggest(label, class_names):
    label = norm(label)
    suggestions = []

    # 模糊匹配
    close = difflib.get_close_matches(label, class_names, n=8, cutoff=0.55)
    suggestions.extend(close)

    # 子串匹配
    for name in class_names:
        if label in name or name in label:
            if name not in suggestions:
                suggestions.append(name)

    return suggestions[:10]


def main():
    parser = argparse.ArgumentParser(
        description="Check semantic class_whitelist against OIV7 classes and alias mappings."
    )
    parser.add_argument("--classes", default=str(DEFAULT_CLASSES))
    parser.add_argument("--aliases", default=str(DEFAULT_ALIASES))
    parser.add_argument("--semantic-yaml", default=str(DEFAULT_SEMANTIC_YAML))
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Only check, do not auto-sync common aliases and 601-class self mappings.",
    )
    args = parser.parse_args()

    classes_path = Path(args.classes)
    aliases_path = Path(args.aliases)
    semantic_yaml_path = Path(args.semantic_yaml)

    classes = load_classes(classes_path)
    class_names = [norm(x) for x in classes]

    class_to_ids = {}
    for idx, name in enumerate(class_names):
        class_to_ids.setdefault(name, []).append(idx)

    aliases, changed = sync_aliases(classes, aliases_path, args.check_only)
    whitelist = parse_class_whitelist(semantic_yaml_path)

    print("==== semantic whitelist alias check ====")
    print(f"classes_file : {classes_path}")
    print(f"classes      : {len(classes)}")
    print(f"aliases_file : {aliases_path}")
    print(f"aliases      : {len(aliases)}")
    print(f"semantic_yaml: {semantic_yaml_path}")
    print(f"whitelist    : {len(whitelist)}")
    if changed and not args.check_only:
        print("[INFO] aliases file synced/updated")
    elif changed and args.check_only:
        print("[INFO] aliases file would be updated, but --check-only was used")
    print()

    miss = []

    for label in whitelist:
        candidates, hits = unique_hits(label, aliases, class_to_ids)

        if hits:
            hit_text = ", ".join([f"{cid}:{name}" for cid, name in hits])
            print(f"[OK]   {label:18s} -> {hit_text}")
        else:
            miss.append(label)
            print(f"[MISS] {label:18s} -> no class id matched")
            print(f"       candidates: {candidates}")
            sug = suggest(label, class_names)
            if sug:
                print(f"       suggestions: {sug}")

    print()
    if miss:
        print("==== result: MISS ====")
        print("These whitelist labels do not map to any OIV7 class:")
        for label in miss:
            print(f"  - {label}")
        print()
        print("Fix method:")
        print(f"  nano {aliases_path}")
        print("Add or edit one entry, for example:")
        print('  "your_label": ["real oiv7 class name"]')
        print()
        print("Then rerun:")
        print(f"  python3 {Path(__file__).resolve()}")
        raise SystemExit(2)

    print("==== result: OK ====")
    print("All whitelist labels can map to OIV7 class ids.")
    print()
    print("Reminder:")
    print("- If you only changed oiv7_aliases.json: restart realtime_bpu_yolo.")
    print("- If you changed semantic_mapping.yaml: rebuild digua_semantic_mapping and restart related nodes.")


if __name__ == "__main__":
    main()
