#!/usr/bin/env python3
import argparse
from pathlib import Path
import yaml

DEFAULT_POSE_FILE = Path.home() / 'digua_ws/digua_navigation_data/named_poses.yaml'


def main():
    parser = argparse.ArgumentParser(description='List saved named poses.')
    parser.add_argument('--file', type=str, default=str(DEFAULT_POSE_FILE), help='Named poses yaml file.')
    args = parser.parse_args()

    pose_file = Path(args.file)
    if not pose_file.exists():
        print(f'Pose file not found: {pose_file}')
        return

    with open(pose_file, 'r') as f:
        data = yaml.safe_load(f)

    poses = {} if data is None else data.get('poses', {})

    if not poses:
        print('No named poses saved.')
        return

    print(f'Named poses in {pose_file}:')
    for name, p in poses.items():
        print(
            f'  {name}: x={p["x"]:.3f}, y={p["y"]:.3f}, '
            f'yaw={p.get("yaw_deg", 0.0):.1f} deg, frame={p.get("frame_id", "map")}'
        )


if __name__ == '__main__':
    main()
