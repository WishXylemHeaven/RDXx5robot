#!/usr/bin/env python3
import argparse
import math
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


DEFAULT_POSE_FILE = Path.home() / 'digua_ws/digua_navigation_data/named_poses.yaml'


def yaw_to_quaternion(yaw_rad):
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return 0.0, 0.0, qz, qw


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('digua_auto_initial_pose')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def publish_initial_pose(self, pose_data, repeat=10, interval=0.2):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = pose_data.get('frame_id', 'map')
        msg.pose.pose.position.x = float(pose_data['x'])
        msg.pose.pose.position.y = float(pose_data['y'])
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(float(pose_data['yaw']))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # AMCL 初始位姿协方差。x/y/yaw 给一点不确定性。
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.get_logger().info(
            f'Publishing initial pose: x={msg.pose.pose.position.x:.3f}, '
            f'y={msg.pose.pose.position.y:.3f}, yaw={math.degrees(float(pose_data["yaw"])):.1f} deg'
        )

        for _ in range(repeat):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(description='Publish a saved named pose to /initialpose.')
    parser.add_argument('name', type=str, default='home', nargs='?', help='Pose name, default: home.')
    parser.add_argument('--file', type=str, default=str(DEFAULT_POSE_FILE), help='Named poses yaml file.')
    parser.add_argument('--repeat', type=int, default=10, help='Publish repeat count.')
    args = parser.parse_args()

    pose_file = Path(args.file)
    if not pose_file.exists():
        print(f'Pose file not found: {pose_file}')
        raise SystemExit(1)

    with open(pose_file, 'r') as f:
        data = yaml.safe_load(f)

    if data is None or 'poses' not in data or args.name not in data['poses']:
        print(f'Pose "{args.name}" not found in {pose_file}')
        raise SystemExit(1)

    rclpy.init()
    node = InitialPosePublisher()

    try:
        node.publish_initial_pose(data['poses'][args.name], repeat=args.repeat)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
