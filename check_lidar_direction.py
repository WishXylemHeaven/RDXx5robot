#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan


class LidarDirectionChecker(Node):
    def __init__(self):
        super().__init__('lidar_direction_checker')

        # LaserScan 常见 QoS 是 Best Effort。
        # 如果这里用默认 Reliable，就会出现：
        # offering incompatible QoS. Last incompatible policy: RELIABILITY
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            scan_qos
        )

    def get_min_range_near_angle(self, msg, target_deg, window_deg=8):
        target_rad = math.radians(target_deg)
        values = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # 计算当前角度和目标角度的最小差值，避免 180/-180 边界问题
            diff = math.atan2(
                math.sin(angle - target_rad),
                math.cos(angle - target_rad)
            )

            if abs(math.degrees(diff)) <= window_deg:
                if math.isfinite(r) and msg.range_min < r < msg.range_max:
                    values.append(r)

        if not values:
            return None

        return min(values)

    def callback(self, msg):
        front = self.get_min_range_near_angle(msg, 0)
        left = self.get_min_range_near_angle(msg, 90)
        back = self.get_min_range_near_angle(msg, 180)
        right = self.get_min_range_near_angle(msg, -90)

        def fmt(v):
            if v is None:
                return "无有效点"
            return f"{v:.3f} m"

        print("\033c", end="")
        print("========== YDLIDAR X2 方向检查 ==========")
        print("")
        print(f"前方   0°   : {fmt(front)}")
        print(f"左方  +90°  : {fmt(left)}")
        print(f"后方  180°  : {fmt(back)}")
        print(f"右方  -90°  : {fmt(right)}")
        print("")
        print("测试方法：")
        print("1. 把纸板/手放在雷达正前方，看“前方 0°”是否明显变小")
        print("2. 放在左边，看“左方 +90°”是否明显变小")
        print("3. 放在右边，看“右方 -90°”是否明显变小")
        print("4. 如果对应正确，雷达方向就是正确的")
        print("")
        print("按 Ctrl+C 退出")


def main():
    rclpy.init()
    node = LidarDirectionChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()