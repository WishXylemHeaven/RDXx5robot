#!/usr/bin/env python3
import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2


class PointCloud2Restamp(Node):
    def __init__(self):
        super().__init__('camera_points_restamp')

        self.declare_parameter('input_topic', '/camera/depth/points')
        self.declare_parameter('output_topic', '/camera/depth/points_now')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('publish_hz', 6.0)
        self.declare_parameter('cache_max_age', 1.2)

        # 关键参数：
        # RDK X5 上 odom/base TF 经常比当前时间慢 0.3~0.6s。
        # 点云 stamp 用 now 会导致 collision_monitor 查 TF 时出现 extrapolation into the future。
        # 所以把点云时间戳回退 0.55s。
        self.declare_parameter('stamp_delay', 0.55)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.cache_max_age = float(self.get_parameter('cache_max_age').value)
        self.stamp_delay = float(self.get_parameter('stamp_delay').value)

        self.last_msg = None
        self.last_receive_ns = 0
        self.last_input_age = float('nan')
        self.max_input_age = 0.0

        self.pub_count = 0
        self.recv_count = 0
        self.skip_stale_count = 0

        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile_sensor_data
        )

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.callback,
            qos_profile_sensor_data
        )

        period = 1.0 / self.publish_hz if self.publish_hz > 0.0 else 0.2
        self.pub_timer = self.create_timer(period, self.publish_cached)
        self.report_timer = self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'PointCloud2 adapter cached republish: {self.input_topic} -> {self.output_topic}, '
            f'publish_hz={self.publish_hz}, cache_max_age={self.cache_max_age}, '
            f'stamp_delay={self.stamp_delay}'
        )

    def callback(self, msg: PointCloud2):
        now = self.get_clock().now()
        self.last_msg = msg
        self.last_receive_ns = now.nanoseconds
        self.recv_count += 1

        try:
            age = (now - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
            self.last_input_age = age
            if math.isfinite(age):
                self.max_input_age = max(self.max_input_age, age)
        except Exception:
            self.last_input_age = float('nan')

    def publish_cached(self):
        if self.last_msg is None:
            return

        now = self.get_clock().now()
        age_since_receive = (now.nanoseconds - self.last_receive_ns) / 1e9

        if age_since_receive > self.cache_max_age:
            self.skip_stale_count += 1
            return

        msg = copy.deepcopy(self.last_msg)

        stamp_time = now - Duration(seconds=self.stamp_delay)
        msg.header.stamp = stamp_time.to_msg()

        if self.frame_id:
            msg.header.frame_id = self.frame_id

        self.pub.publish(msg)
        self.pub_count += 1

    def report(self):
        self.get_logger().info(
            f'received={self.recv_count}, published={self.pub_count}, stale_skips={self.skip_stale_count}, '
            f'last_input_age={self.last_input_age:.3f}s, max_input_age={self.max_input_age:.3f}s, '
            f'publish_hz={self.publish_hz}, cache_max_age={self.cache_max_age}, '
            f'stamp_delay={self.stamp_delay}'
        )
        self.recv_count = 0
        self.pub_count = 0
        self.skip_stale_count = 0
        self.max_input_age = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = PointCloud2Restamp()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
