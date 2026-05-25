#!/usr/bin/env python3
import math
from typing import List, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException

import tf2_ros
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2


def quat_to_rot(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0.0:
        return np.eye(3, dtype=np.float32)

    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    return np.array([
        [1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [    2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz,     2*qy*qz - 2*qx*qw],
        [    2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy],
    ], dtype=np.float32)


class LowObstacleCloudFilter(Node):
    def __init__(self):
        super().__init__('low_obstacle_cloud_filter')

        self.declare_parameter('input_topic', '/camera/depth/points_now')
        self.declare_parameter('output_topic', '/camera/low_obstacle_points')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('stamp_delay', 1.05)

        # base_footprint 坐标系下的低矮障碍 ROI
        self.declare_parameter('x_min', 0.30)
        self.declare_parameter('x_max', 1.05)
        self.declare_parameter('y_abs', 0.30)
        self.declare_parameter('z_min', 0.15)
        self.declare_parameter('z_max', 0.45)

        self.declare_parameter('sample_stride', 64)
        self.declare_parameter('max_publish_points', 2000)
        self.declare_parameter('min_obstacle_points', 120)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.stamp_delay = float(self.get_parameter('stamp_delay').value)

        self.x_min = float(self.get_parameter('x_min').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.y_abs = float(self.get_parameter('y_abs').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)

        self.sample_stride = max(1, int(self.get_parameter('sample_stride').value))
        self.max_publish_points = max(1, int(self.get_parameter('max_publish_points').value))
        self.min_obstacle_points = max(1, int(self.get_parameter('min_obstacle_points').value))

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.callback,
            qos_profile_sensor_data
        )

        self.recv_count = 0
        self.pub_count = 0
        self.last_input_points = 0
        self.last_sampled_points = 0
        self.last_candidate_points = 0
        self.last_out_points = 0
        self.last_error = ''

        self.report_timer = self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'Low obstacle mark-only filter: {self.input_topic} -> {self.output_topic}, '
            f'target_frame={self.target_frame}, '
            f'ROI x=[{self.x_min},{self.x_max}], y=±{self.y_abs}, z=[{self.z_min},{self.z_max}], '
            f'sample_stride={self.sample_stride}, min_obstacle_points={self.min_obstacle_points}, '
            f'stamp_delay={self.stamp_delay}'
        )

    def field_offset(self, msg: PointCloud2, name: str):
        for f in msg.fields:
            if f.name == name:
                return f.offset
        return None

    def extract_xyz_numpy(self, msg: PointCloud2):
        ox = self.field_offset(msg, 'x')
        oy = self.field_offset(msg, 'y')
        oz = self.field_offset(msg, 'z')

        if ox is None or oy is None or oz is None:
            raise RuntimeError('PointCloud2 missing x/y/z fields')

        if msg.point_step <= 0:
            raise RuntimeError('invalid point_step')

        n_from_data = len(msg.data) // msg.point_step
        n_from_wh = int(msg.width) * int(msg.height) if msg.width > 0 and msg.height > 0 else n_from_data
        n = min(n_from_data, n_from_wh)

        if n <= 0:
            return np.empty((0, 3), dtype=np.float32)

        endian = '>' if msg.is_bigendian else '<'
        dtype = np.dtype(endian + 'f4')

        x = np.ndarray(shape=(n,), dtype=dtype, buffer=msg.data, offset=ox, strides=(msg.point_step,))
        y = np.ndarray(shape=(n,), dtype=dtype, buffer=msg.data, offset=oy, strides=(msg.point_step,))
        z = np.ndarray(shape=(n,), dtype=dtype, buffer=msg.data, offset=oz, strides=(msg.point_step,))

        if self.sample_stride > 1:
            x = x[::self.sample_stride]
            y = y[::self.sample_stride]
            z = z[::self.sample_stride]

        return np.stack((x, y, z), axis=1).astype(np.float32, copy=False)

    def make_header(self):
        header = Header()
        header.frame_id = self.target_frame
        header.stamp = (self.get_clock().now() - Duration(seconds=self.stamp_delay)).to_msg()
        return header

    def publish_points(self, points: List[Tuple[float, float, float]]):
        self.pub.publish(point_cloud2.create_cloud_xyz32(self.make_header(), points))
        self.pub_count += 1
        self.last_out_points = len(points)

    def callback(self, msg: PointCloud2):
        self.recv_count += 1

        if not msg.header.frame_id:
            self.last_error = 'input cloud has empty frame_id'
            self.publish_points([])
            return

        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, Time())
        except Exception as e:
            self.last_error = f'tf lookup failed: {e}'
            self.publish_points([])
            return

        try:
            pts = self.extract_xyz_numpy(msg)
        except Exception as e:
            self.last_error = f'extract xyz failed: {e}'
            self.publish_points([])
            return

        self.last_input_points = len(msg.data) // msg.point_step if msg.point_step > 0 else 0
        self.last_sampled_points = int(pts.shape[0])

        if pts.shape[0] == 0:
            self.publish_points([])
            self.last_error = ''
            return

        finite = np.isfinite(pts).all(axis=1)
        pts = pts[finite]

        if pts.shape[0] == 0:
            self.publish_points([])
            self.last_error = ''
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=np.float32)

        bpts = pts @ R.T + trans

        mask = (
            (bpts[:, 0] >= self.x_min) &
            (bpts[:, 0] <= self.x_max) &
            (np.abs(bpts[:, 1]) <= self.y_abs) &
            (bpts[:, 2] >= self.z_min) &
            (bpts[:, 2] <= self.z_max)
        )

        candidates = bpts[mask]
        self.last_candidate_points = int(candidates.shape[0])

        if candidates.shape[0] < self.min_obstacle_points:
            self.publish_points([])
            self.last_error = ''
            return

        out_pts = candidates

        if out_pts.shape[0] > self.max_publish_points:
            step = max(1, out_pts.shape[0] // self.max_publish_points)
            out_pts = out_pts[::step][:self.max_publish_points]

        points = [(float(x), float(y), float(z)) for x, y, z in out_pts]
        self.publish_points(points)
        self.last_error = ''

    def report(self):
        msg = (
            f'received={self.recv_count}, published={self.pub_count}, '
            f'input_points={self.last_input_points}, sampled_points={self.last_sampled_points}, '
            f'candidate_points={self.last_candidate_points}, output_points={self.last_out_points}, '
            f'min_obstacle_points={self.min_obstacle_points}, '
            f'ROI x=[{self.x_min},{self.x_max}], y=±{self.y_abs}, z=[{self.z_min},{self.z_max}]'
        )
        if self.last_error:
            msg += f', last_error={self.last_error}'
        self.get_logger().info(msg)

        self.recv_count = 0
        self.pub_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = LowObstacleCloudFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
