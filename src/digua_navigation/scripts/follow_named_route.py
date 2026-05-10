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


def load_named_poses(path):
    if not path.exists():
        raise FileNotFoundError(f'Pose file not found: {path}')

    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    if data is None or 'poses' not in data:
        return {}

    return data['poses']


class RouteNavigator(Node):
    def __init__(self):
        super().__init__('digua_follow_named_route')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.feedback = None

    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback

    def send_goal(self, name, pose_data, timeout_sec):
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

        self.feedback = None
        self.get_logger().info(
            f'Route goal "{name}": x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg'
        )

        send_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'Goal "{name}" rejected.')
            return False

        self.get_logger().info(f'Goal "{name}" accepted.')
        result_future = goal_handle.get_result_async()

        start_time = time.time()
        last_print = 0.0

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

            now = time.time()
            if self.feedback is not None and now - last_print > 1.0:
                last_print = now
                self.get_logger().info(
                    f'[{name}] distance_remaining={self.feedback.distance_remaining:.3f} m, '
                    f'navigation_time={self.feedback.navigation_time.sec}s, '
                    f'recoveries={self.feedback.number_of_recoveries}'
                )

            if now - start_time > timeout_sec:
                self.get_logger().warn(f'Goal "{name}" timeout, canceling...')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return False

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal "{name}" succeeded.')
            return True

        self.get_logger().error(f'Goal "{name}" failed, status={result.status}')
        return False


def main():
    parser = argparse.ArgumentParser(description='Follow a route made of saved named poses.')
    parser.add_argument('poses', nargs='+', help='Pose names, e.g. home test_front home.')
    parser.add_argument('--file', type=str, default=str(DEFAULT_POSE_FILE), help='Named poses yaml file.')
    parser.add_argument('--timeout-per-goal', type=float, default=180.0, help='Timeout for each goal.')
    parser.add_argument('--wait-between', type=float, default=1.0, help='Seconds to wait between goals.')
    parser.add_argument('--loops', type=int, default=1, help='How many times to repeat the route.')
    parser.add_argument('--continue-on-failure', action='store_true', help='Continue route even if one goal fails.')
    args = parser.parse_args()

    pose_file = Path(args.file)
    try:
        named_poses = load_named_poses(pose_file)
    except Exception as e:
        print(e)
        sys.exit(1)

    missing = [name for name in args.poses if name not in named_poses]
    if missing:
        print(f'Missing named poses: {missing}')
        print('Available poses:')
        for name in named_poses:
            print(f'  - {name}')
        sys.exit(1)

    rclpy.init()
    node = RouteNavigator()

    all_ok = True

    try:
        for loop_idx in range(args.loops):
            node.get_logger().info(f'Starting route loop {loop_idx + 1}/{args.loops}: {" -> ".join(args.poses)}')

            for name in args.poses:
                ok = node.send_goal(name, named_poses[name], args.timeout_per_goal)

                if not ok:
                    all_ok = False
                    if not args.continue_on_failure:
                        node.get_logger().error('Route aborted due to failed goal.')
                        raise SystemExit(1)

                if args.wait_between > 0:
                    time.sleep(args.wait_between)

        if all_ok:
            node.get_logger().info('Route completed successfully.')
        else:
            node.get_logger().warn('Route completed with some failed goals.')

    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
