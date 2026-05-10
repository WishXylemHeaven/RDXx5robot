#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def yaw_to_quaternion(yaw_rad):
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return 0.0, 0.0, qz, qw


class GoToPoseClient(Node):
    def __init__(self):
        super().__init__('digua_go_to_pose_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.feedback = None

    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback

    def send_goal(self, x, y, yaw_deg, frame_id='map', timeout_sec=120.0):
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/navigate_to_pose action server not available.')
            return False

        yaw_rad = math.radians(yaw_deg)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending goal: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f} deg, frame={frame_id}'
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
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded.')
            return True

        self.get_logger().error(f'Navigation failed, status={status}')
        return False


def main():
    parser = argparse.ArgumentParser(description='Send a Nav2 NavigateToPose goal.')
    parser.add_argument('--x', type=float, required=True, help='Goal x in map frame, meters.')
    parser.add_argument('--y', type=float, required=True, help='Goal y in map frame, meters.')
    parser.add_argument('--yaw', type=float, default=0.0, help='Goal yaw in degrees.')
    parser.add_argument('--frame', type=str, default='map', help='Goal frame id.')
    parser.add_argument('--timeout', type=float, default=120.0, help='Timeout in seconds.')
    args = parser.parse_args()

    rclpy.init()
    node = GoToPoseClient()

    try:
        ok = node.send_goal(args.x, args.y, args.yaw, args.frame, args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
