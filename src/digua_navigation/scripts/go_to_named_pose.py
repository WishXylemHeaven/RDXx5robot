#!/usr/bin/env python3
import argparse
import math
import sys
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


DEFAULT_POSE_FILE = Path.home() / 'digua_ws/digua_navigation_data/named_poses.yaml'


def yaw_to_quaternion(yaw_rad):
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return 0.0, 0.0, qz, qw


class NamedPoseNavigator(Node):
    def __init__(self):
        super().__init__('digua_go_to_named_pose')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.feedback = None

    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback

    def send_goal(self, pose_data, timeout_sec=120.0):
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/navigate_to_pose action server not available.')
            return False

        frame_id = pose_data.get('frame_id', 'map')
        x = float(pose_data['x'])
        y = float(pose_data['y'])
        yaw = float(pose_data['yaw'])

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending named goal: frame={frame_id}, x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg'
        )

        send_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()

        start_time = time.time()
        last_print = 0.0

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

            now = time.time()
            if self.feedback is not None and now - last_print > 1.0:
                last_print = now
                self.get_logger().info(
                    f'distance_remaining={self.feedback.distance_remaining:.3f} m, '
                    f'navigation_time={self.feedback.navigation_time.sec}s, '
                    f'recoveries={self.feedback.number_of_recoveries}'
                )

            if now - start_time > timeout_sec:
                self.get_logger().warn('Navigation timeout, canceling goal...')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return False

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded.')
            return True

        self.get_logger().error(f'Navigation failed, status={result.status}')
        return False


def main():
    parser = argparse.ArgumentParser(description='Navigate to a saved named pose.')
    parser.add_argument('name', type=str, help='Pose name in named_poses.yaml.')
    parser.add_argument('--file', type=str, default=str(DEFAULT_POSE_FILE), help='Named poses yaml file.')
    parser.add_argument('--timeout', type=float, default=120.0, help='Navigation timeout seconds.')
    args = parser.parse_args()

    pose_file = Path(args.file)
    if not pose_file.exists():
        print(f'Pose file not found: {pose_file}')
        sys.exit(1)

    with open(pose_file, 'r') as f:
        data = yaml.safe_load(f)

    if data is None or 'poses' not in data or args.name not in data['poses']:
        print(f'Pose "{args.name}" not found in {pose_file}')
        print('Available poses:')
        if data and 'poses' in data:
            for name in data['poses']:
                print(f'  - {name}')
        sys.exit(1)

    rclpy.init()
    node = NamedPoseNavigator()

    try:
        ok = node.send_goal(data['poses'][args.name], args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
