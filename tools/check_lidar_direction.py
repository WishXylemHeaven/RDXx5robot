#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class LidarDirectionChecker(Node):
    def __init__(self):
        super().__init__('lidar_direction_checker')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.cb,
            qos_profile_sensor_data
        )

    def sector_min(self, msg, center_deg, width_deg=15.0):
        center = math.radians(center_deg)
        width = math.radians(width_deg)
        vals = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle_diff(angle, center)) <= width:
                vals.append(r)

        if not vals:
            return None
        return min(vals)

    def cb(self, msg):
        front = self.sector_min(msg, 0)
        left = self.sector_min(msg, 90)
        right = self.sector_min(msg, -90)
        back = self.sector_min(msg, 180)

        def fmt(v):
            return "None" if v is None else f"{v:.2f} m"

        self.get_logger().info(
            f"FRONT(+X): {fmt(front)} | "
            f"LEFT(+Y): {fmt(left)} | "
            f"RIGHT(-Y): {fmt(right)} | "
            f"BACK(-X): {fmt(back)}"
        )


def main():
    rclpy.init()
    node = LidarDirectionChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
