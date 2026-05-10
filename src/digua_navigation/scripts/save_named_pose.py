#!/usr/bin/env python3
import argparse
import math
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener


DEFAULT_POSE_FILE = Path.home() / 'digua_ws/digua_navigation_data/named_poses.yaml'


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_yaml(path):
    if not path.exists():
        return {'poses': {}}
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    if data is None:
        data = {'poses': {}}
    if 'poses' not in data:
        data['poses'] = {}
    return data


class TfPoseSaver(Node):
    def __init__(self):
        super().__init__('digua_save_named_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def wait_for_transform(self, target_frame, source_frame, timeout_sec):
        start = time.time()

        while rclpy.ok() and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

            try:
                trans = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )
                return trans
            except Exception as e:
                last_error = str(e)

        self.get_logger().error(
            f'No TF received: {target_frame} -> {source_frame}. '
            f'Last error: {last_error if "last_error" in locals() else "unknown"}'
        )
        return None


def main():
    parser = argparse.ArgumentParser(
        description='Save current robot pose from TF as a named navigation goal.'
    )
    parser.add_argument('name', type=str, help='Pose name, e.g. home, desk, water_dispenser.')
    parser.add_argument('--file', type=str, default=str(DEFAULT_POSE_FILE), help='Named poses yaml file.')
    parser.add_argument('--map-frame', type=str, default='map', help='Map/global frame.')
    parser.add_argument('--base-frame', type=str, default='base_footprint', help='Robot base frame.')
    parser.add_argument('--timeout', type=float, default=10.0, help='Wait timeout for TF.')
    args = parser.parse_args()

    pose_file = Path(args.file)
    pose_file.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = TfPoseSaver()

    try:
        trans = node.wait_for_transform(args.map_frame, args.base_frame, args.timeout)

        if trans is None:
            node.get_logger().error(
                'Failed to save pose. Make sure localization is running, '
                '2D Pose Estimate has been set, and map -> odom -> base_footprint TF exists.'
            )
            raise SystemExit(1)

        t = trans.transform.translation
        q = trans.transform.rotation
        yaw = quaternion_to_yaw(q)

        data = load_yaml(pose_file)
        data['poses'][args.name] = {
            'frame_id': args.map_frame,
            'base_frame': args.base_frame,
            'x': float(t.x),
            'y': float(t.y),
            'yaw': float(yaw),
            'yaw_deg': float(math.degrees(yaw)),
        }

        with open(pose_file, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

        node.get_logger().info(
            f'Saved pose "{args.name}" to {pose_file}: '
            f'x={t.x:.3f}, y={t.y:.3f}, yaw={math.degrees(yaw):.1f} deg, '
            f'from TF {args.map_frame}->{args.base_frame}'
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
