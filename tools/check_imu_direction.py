#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu


class ImuDirectionChecker(Node):
    def __init__(self):
        super().__init__('imu_direction_checker')

        self.declare_parameter('topic', '/imu')
        self.declare_parameter('test_time', 6.0)

        self.topic = self.get_parameter('topic').value
        self.test_time = float(self.get_parameter('test_time').value)

        self.last_msg = None
        self.samples = []

        self.sub = self.create_subscription(
            Imu,
            self.topic,
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(f'正在订阅 IMU 话题: {self.topic}')
        self.get_logger().info('等待 IMU 数据...')

    def imu_callback(self, msg: Imu):
        self.last_msg = msg

    def wait_first_msg(self, timeout=10.0):
        start = time.time()
        while rclpy.ok() and self.last_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                return False
        return True

    def collect_samples(self, seconds):
        self.samples = []
        start = time.time()

        while rclpy.ok() and time.time() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_msg is not None:
                msg = self.last_msg
                self.samples.append((
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ))

        return self.samples

    def print_live_once(self):
        msg = self.last_msg
        if msg is None:
            return

        av = msg.angular_velocity
        la = msg.linear_acceleration

        print()
        print('当前 IMU 数据：')
        print(f'  angular_velocity: x={av.x:+.4f}, y={av.y:+.4f}, z={av.z:+.4f} rad/s')
        print(f'  linear_accel     : x={la.x:+.4f}, y={la.y:+.4f}, z={la.z:+.4f} m/s^2')

    def analyze_yaw(self, title):
        if not self.samples:
            print('没有采集到样本。')
            return

        z_values = [s[2] for s in self.samples]
        avg_z = sum(z_values) / len(z_values)
        max_z = max(z_values)
        min_z = min(z_values)

        print()
        print(f'[{title}] Z轴角速度统计：')
        print(f'  样本数: {len(z_values)}')
        print(f'  avg_z : {avg_z:+.4f} rad/s')
        print(f'  max_z : {max_z:+.4f} rad/s')
        print(f'  min_z : {min_z:+.4f} rad/s')

        return avg_z, max_z, min_z


def main():
    rclpy.init()
    node = ImuDirectionChecker()

    print()
    print('================ IMU 方向验证脚本 ================')
    print('ROS 坐标系约定：')
    print('  X 正方向：车头前方')
    print('  Y 正方向：车体左侧')
    print('  Z 正方向：车体上方')
    print()
    print('判断重点：')
    print('  从车体上方往下看，逆时针/向左转车，angular_velocity.z 应该为正。')
    print('  从车体上方往下看，顺时针/向右转车，angular_velocity.z 应该为负。')
    print('==================================================')
    print()

    ok = node.wait_first_msg(timeout=10.0)
    if not ok:
        print()
        print('[ERROR] 10 秒内没有收到 IMU 数据。')
        print('请检查：')
        print(f'  1. IMU 话题是否存在：ros2 topic list | grep imu')
        print(f'  2. 当前脚本订阅的话题是否正确：{node.topic}')
        print('  3. base_control 是否已启动，并且 pub_imu:=true')
        print()
        node.destroy_node()
        rclpy.shutdown()
        return

    print('[OK] 已收到 IMU 数据。')
    node.print_live_once()

    input('\n第一步：保持小车静止，按 Enter 开始采集静止数据...')
    node.collect_samples(3.0)
    node.analyze_yaw('静止测试')
    print('静止时 angular_velocity.z 应该接近 0，轻微漂移是正常的。')

    input('\n第二步：按 Enter 后，在 6 秒内手动让小车“向左转/逆时针旋转”...')
    node.collect_samples(node.test_time)
    left_avg, left_max, left_min = node.analyze_yaw('向左转 / 逆时针')

    input('\n第三步：按 Enter 后，在 6 秒内手动让小车“向右转/顺时针旋转”...')
    node.collect_samples(node.test_time)
    right_avg, right_max, right_min = node.analyze_yaw('向右转 / 顺时针')

    print()
    print('================ 判断结果 ================')

    left_ok = left_max > abs(left_min)
    right_ok = abs(right_min) > right_max

    if left_ok:
        print('[OK] 向左转时，Z 轴角速度主要为正。')
    else:
        print('[WARN] 向左转时，Z 轴角速度没有明显为正。')

    if right_ok:
        print('[OK] 向右转时，Z 轴角速度主要为负。')
    else:
        print('[WARN] 向右转时，Z 轴角速度没有明显为负。')

    print()
    if left_ok and right_ok:
        print('[结论] IMU Z 轴方向基本正确，可以继续用于 EKF / robot_localization。')
    else:
        print('[结论] IMU Z 轴方向可能反了，或者 IMU 安装方向 / 驱动坐标系需要检查。')
        print('建议检查 URDF 里 imu_link 的 rpy，或者底盘驱动发布 IMU 数据时的坐标变换。')

    print('==========================================')
    print()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
