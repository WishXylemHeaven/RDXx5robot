#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class YawChecker(Node):
    def __init__(self):
        super().__init__('ekf_yaw_checker')
        self.sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.cb,
            10
        )

    def cb(self, msg):
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.get_logger().info(
            f"x={msg.pose.pose.position.x:.3f}, "
            f"y={msg.pose.pose.position.y:.3f}, "
            f"yaw={yaw:.3f} rad / {math.degrees(yaw):.1f} deg"
        )


def main():
    rclpy.init()
    node = YawChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
