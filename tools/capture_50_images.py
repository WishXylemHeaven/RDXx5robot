#!/usr/bin/env python3
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CaptureImages(Node):
    def __init__(self, topic, out_dir, count, interval):
        super().__init__('capture_50_images')
        self.topic = topic
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.count = count
        self.interval = interval
        self.saved = 0
        self.latest_msg = None
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            self.topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(self.interval, self.save_latest_image)

        self.get_logger().info(f'订阅图像话题: {self.topic}')
        self.get_logger().info(f'保存目录: {self.out_dir}')
        self.get_logger().info(f'计划保存: {self.count} 张，每 {self.interval} 秒 1 张')

    def image_callback(self, msg):
        self.latest_msg = msg

    def save_latest_image(self):
        if self.latest_msg is None:
            self.get_logger().warn('还没有收到图像，等待相机数据...')
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_msg, desired_encoding='bgr8')
            filename = self.out_dir / f'calib_{self.saved + 1:03d}.jpg'
            cv2.imwrite(str(filename), cv_img)

            self.saved += 1
            self.get_logger().info(f'已保存 {self.saved}/{self.count}: {filename}')

            if self.saved >= self.count:
                self.get_logger().info('拍摄完成。')
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'保存图像失败: {e}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/camera/color/image_raw')
    parser.add_argument('--out', required=True)
    parser.add_argument('--count', type=int, default=50)
    parser.add_argument('--interval', type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = CaptureImages(args.topic, args.out, args.count, args.interval)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
