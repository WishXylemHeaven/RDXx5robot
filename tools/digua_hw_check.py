#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import math
import argparse
import subprocess
import struct
from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message

try:
    from tf2_ros import Buffer, TransformListener
except Exception:
    Buffer = None
    TransformListener = None


# ============================================================
# 设备检查配置
# ============================================================

DEVICE_CHECKS = [
    {"name": "base serial", "path": "/dev/ttyS1", "required": True},
    {"name": "YDLIDAR X2 serial", "path": "/dev/ttyUSB0", "required": True},
]

USB_CAMERA_KEYWORDS = [
    "orbbec",
    "astra",
    "2bc5",
]


# ============================================================
# 分阶段话题检查配置
# 重点：不要把图像和 IMU/odom 放在同一阶段，否则 Python 会被图像拖慢
# ============================================================

CHECK_PHASES = [
    {
        "name": "Base sensors",
        "duration": 5.0,
        "topics": [
            {
                "name": "lidar scan",
                "topic": "/scan",
                "type": "sensor_msgs/msg/LaserScan",
                "min_hz": 4.0,
                "required": True,
                "validator": "scan",
                "max_samples": 40,
            },
            {
                "name": "imu",
                "topic": "/imu",
                "type": "sensor_msgs/msg/Imu",
                "min_hz": 20.0,
                "required": True,
                "validator": "imu",
                "max_samples": 160,
            },
            {
                "name": "raw odom",
                "topic": "/odom",
                "type": "nav_msgs/msg/Odometry",
                "min_hz": 20.0,
                "required": True,
                "validator": "odom",
                "max_samples": 160,
            },
            {
                "name": "ekf odom",
                "topic": "/odometry/filtered",
                "type": "nav_msgs/msg/Odometry",
                "min_hz": 10.0,
                "required": True,
                "validator": "odom",
                "max_samples": 160,
            },
        ],
    },
    {
        "name": "Color image",
        "duration": 3.0,
        "topics": [
            {
                "name": "color image",
                "topic": "/camera/color/image_raw",
                "type": "sensor_msgs/msg/Image",
                "min_hz": 5.0,
                "required": True,
                "validator": "image",
                "max_samples": 6,
            },
        ],
    },
    {
        "name": "Depth image",
        "duration": 3.0,
        "topics": [
            {
                "name": "depth image",
                "topic": "/camera/depth/image_raw",
                "type": "sensor_msgs/msg/Image",
                "min_hz": 5.0,
                "required": True,
                "validator": "depth_image",
                "max_samples": 6,
            },
        ],
    },
    {
        "name": "CameraInfo",
        "duration": 2.0,
        "topics": [
            {
                "name": "color camera_info",
                "topic": "/camera/color/camera_info",
                "type": "sensor_msgs/msg/CameraInfo",
                "min_hz": 0.0,
                "required": True,
                "validator": "camera_info",
                "max_samples": 10,
            },
            {
                "name": "depth camera_info",
                "topic": "/camera/depth/camera_info",
                "type": "sensor_msgs/msg/CameraInfo",
                "min_hz": 0.0,
                "required": True,
                "validator": "camera_info",
                "max_samples": 10,
            },
        ],
    },
    {
        "name": "RGBD sync",
        "duration": 3.0,
        "topics": [
            {
                "name": "rgbd sync",
                "topic": "/camera/rgbd_image",
                "type": "rtabmap_msgs/msg/RGBDImage",
                "min_hz": 5.0,
                "required": True,
                "validator": "rgbd",
                "max_samples": 6,
            },
        ],
    },
]


TF_CHECKS = [
    {"name": "odom -> base_footprint", "target": "odom", "source": "base_footprint", "required": True},
    {"name": "base_footprint -> base_link", "target": "base_footprint", "source": "base_link", "required": True},
    {"name": "base_link -> laser_frame", "target": "base_link", "source": "laser_frame", "required": True},
    {"name": "base_link -> camera_link", "target": "base_link", "source": "camera_link", "required": True},
    {"name": "camera_link -> camera_color_optical_frame", "target": "camera_link", "source": "camera_color_optical_frame", "required": True},
    {"name": "camera_link -> camera_depth_optical_frame", "target": "camera_link", "source": "camera_depth_optical_frame", "required": True},
]


# ============================================================
# 工具函数
# ============================================================

def is_finite_number(x: Any) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def near_zero(x: Any, eps: float = 1e-8) -> bool:
    try:
        return abs(float(x)) < eps
    except Exception:
        return False


def vector_norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def yaw_from_quat(q) -> Optional[float]:
    try:
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    except Exception:
        return None


def fmt_hz(hz: Optional[float]) -> str:
    if hz is None:
        return "N/A"
    return f"{hz:.2f} Hz"


def print_result(status: str, item: str, detail: str = ""):
    prefix = {
        "PASS": "[PASS]",
        "WARN": "[WARN]",
        "FAIL": "[FAIL]",
        "SKIP": "[SKIP]",
    }.get(status, "[INFO]")

    if detail:
        print(f"{prefix:<7} {item:<44} {detail}")
    else:
        print(f"{prefix:<7} {item}")


def get_header_frame_id(msg) -> str:
    try:
        return str(msg.header.frame_id)
    except Exception:
        return ""


@dataclass
class TopicCheck:
    name: str
    topic: str
    type_name: str
    min_hz: float
    required: bool
    validator_name: str
    max_samples: int = 50

    count: int = 0
    times: List[float] = field(default_factory=list)
    samples: List[Any] = field(default_factory=list)

    import_error: str = ""
    subscription_created: bool = False
    per_message_invalid_count: int = 0
    last_invalid_info: str = ""

    def hz(self) -> Optional[float]:
        if len(self.times) < 2:
            return None
        dt = self.times[-1] - self.times[0]
        if dt <= 0:
            return None
        return (len(self.times) - 1) / dt

    def add_sample(self, msg):
        self.count += 1
        self.times.append(time.monotonic())
        if len(self.samples) < self.max_samples:
            self.samples.append(msg)


# ============================================================
# 单条消息快速检查
# ============================================================

def validate_msg_basic(c: TopicCheck, msg) -> Tuple[bool, str]:
    if c.validator_name in ["scan", "imu", "odom", "image", "depth_image", "camera_info"]:
        frame_id = get_header_frame_id(msg)
        if frame_id == "":
            return False, "header.frame_id is empty"
    return True, "basic ok"


def validate_scan_msg(msg) -> Tuple[bool, str]:
    ranges = list(msg.ranges)
    if len(ranges) == 0:
        return False, "empty ranges"

    finite = [r for r in ranges if math.isfinite(float(r))]
    valid = [r for r in finite if msg.range_min <= r <= msg.range_max]

    if len(valid) < 10:
        return False, f"valid ranges too few: {len(valid)}/{len(ranges)}"

    rounded_unique = len(set(round(float(r), 3) for r in valid[:200]))
    if rounded_unique <= 3 and len(valid) > 50:
        return False, f"ranges look stuck or fake, unique_valid_values={rounded_unique}"

    return True, f"valid ranges {len(valid)}/{len(ranges)}, min={min(valid):.2f}m, max={max(valid):.2f}m"


def validate_imu_msg(msg) -> Tuple[bool, str]:
    q = msg.orientation
    av = msg.angular_velocity
    la = msg.linear_acceleration

    values = [
        q.x, q.y, q.z, q.w,
        av.x, av.y, av.z,
        la.x, la.y, la.z,
    ]

    if not all(is_finite_number(v) for v in values):
        return False, "imu contains NaN/Inf"

    if all(near_zero(v) for v in values):
        return False, "imu all fields are zero"

    if near_zero(q.x) and near_zero(q.y) and near_zero(q.z) and near_zero(q.w):
        return False, "orientation quaternion is all zero"

    q_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    ang_norm = vector_norm3(av.x, av.y, av.z)
    acc_norm = vector_norm3(la.x, la.y, la.z)

    if q_norm < 0.5 or q_norm > 1.5:
        return False, f"orientation quaternion norm abnormal: {q_norm:.3f}"

    return True, f"q_norm={q_norm:.3f}, ang_norm={ang_norm:.4f}, acc_norm={acc_norm:.2f}"


def validate_odom_msg(msg) -> Tuple[bool, str]:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

    if not all(is_finite_number(v) for v in values):
        return False, "odom pose contains NaN/Inf"

    q_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if q_norm < 0.5 or q_norm > 1.5:
        return False, f"odom quaternion norm abnormal: {q_norm:.3f}"

    return True, f"x={p.x:.3f}, y={p.y:.3f}"


def validate_image_msg(msg) -> Tuple[bool, str]:
    if msg.width <= 0 or msg.height <= 0:
        return False, f"invalid image size: {msg.width}x{msg.height}"

    if len(msg.data) == 0:
        return False, "image data is empty"

    if msg.step < msg.width:
        return False, f"image step too small: step={msg.step}, width={msg.width}"

    return True, f"{msg.width}x{msg.height}, encoding={msg.encoding}"


def validate_depth_image_msg(msg) -> Tuple[bool, str]:
    ok, detail = validate_image_msg(msg)
    if not ok:
        return ok, detail

    if msg.encoding not in ["16UC1", "32FC1"]:
        return False, f"unsupported depth encoding: {msg.encoding}"

    # 注意：这里不做全图像素扫描，避免拖慢回调。
    # 深度有效像素检查放在 final_check_depth_image() 里只做少量样本统计。
    return True, f"{msg.width}x{msg.height}, encoding={msg.encoding}"


def validate_camera_info_msg(msg) -> Tuple[bool, str]:
    if msg.width <= 0 or msg.height <= 0:
        return False, f"invalid camera_info size: {msg.width}x{msg.height}"

    if len(msg.k) != 9:
        return False, "camera_info K matrix invalid length"

    if len(msg.p) != 12:
        return False, "camera_info P matrix invalid length"

    if not all(is_finite_number(v) for v in msg.k):
        return False, "camera_info K matrix contains NaN/Inf"

    if not all(is_finite_number(v) for v in msg.p):
        return False, "camera_info P matrix contains NaN/Inf"

    if sum(abs(float(v)) for v in msg.k) < 1e-6:
        return False, "camera_info K matrix is all zero"

    if msg.k[0] <= 0 or msg.k[4] <= 0:
        return False, f"camera_info focal length invalid: fx={msg.k[0]}, fy={msg.k[4]}"

    if abs(msg.k[8] - 1.0) > 1e-6:
        return False, f"camera_info K[8] invalid: {msg.k[8]}"

    return True, f"{msg.width}x{msg.height}, fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}"


def validate_rgbd_msg(msg) -> Tuple[bool, str]:
    rgb = getattr(msg, "rgb", None)
    depth = getattr(msg, "depth", None)

    if rgb is None or depth is None:
        return True, "received"

    if rgb.width <= 0 or rgb.height <= 0:
        return False, "rgb image in RGBDImage has invalid size"

    if depth.width <= 0 or depth.height <= 0:
        return False, "depth image in RGBDImage has invalid size"

    if len(rgb.data) == 0:
        return False, "rgb data in RGBDImage is empty"

    if len(depth.data) == 0:
        return False, "depth data in RGBDImage is empty"

    return True, f"rgb={rgb.width}x{rgb.height}, depth={depth.width}x{depth.height}"


MESSAGE_VALIDATORS = {
    "scan": validate_scan_msg,
    "imu": validate_imu_msg,
    "odom": validate_odom_msg,
    "image": validate_image_msg,
    "depth_image": validate_depth_image_msg,
    "camera_info": validate_camera_info_msg,
    "rgbd": validate_rgbd_msg,
}


# ============================================================
# 深度图有效像素统计
# ============================================================

def depth_valid_pixel_stats(msg) -> Tuple[int, int, int, Optional[float], Optional[float], float]:
    data = bytes(msg.data)

    if msg.encoding == "16UC1":
        total_pixels = len(data) // 2
        if total_pixels == 0:
            return 0, 0, 0, None, None, 0.0

        sample_count = min(30000, total_pixels)
        step = max(1, total_pixels // sample_count)

        checked = 0
        valid = 0
        zero = 0
        min_v = None
        max_v = None

        for idx in range(0, total_pixels, step):
            off = idx * 2
            if off + 1 >= len(data):
                break

            v = data[off] | (data[off + 1] << 8)
            checked += 1

            if v == 0:
                zero += 1
                continue

            if 0 < v < 10000:
                valid += 1
                meters = v / 1000.0
                min_v = meters if min_v is None else min(min_v, meters)
                max_v = meters if max_v is None else max(max_v, meters)

        ratio = valid / max(1, checked)
        return checked, valid, zero, min_v, max_v, ratio

    if msg.encoding == "32FC1":
        total_pixels = len(data) // 4
        if total_pixels == 0:
            return 0, 0, 0, None, None, 0.0

        sample_count = min(30000, total_pixels)
        step = max(1, total_pixels // sample_count)

        checked = 0
        valid = 0
        zero = 0
        min_v = None
        max_v = None

        for idx in range(0, total_pixels, step):
            off = idx * 4
            if off + 3 >= len(data):
                break

            v = struct.unpack_from("<f", data, off)[0]
            checked += 1

            if v == 0.0:
                zero += 1
                continue

            if math.isfinite(v) and 0.0 < v < 10.0:
                valid += 1
                min_v = v if min_v is None else min(min_v, v)
                max_v = v if max_v is None else max(max_v, v)

        ratio = valid / max(1, checked)
        return checked, valid, zero, min_v, max_v, ratio

    return 0, 0, 0, None, None, 0.0


# ============================================================
# 多条消息最终检查
# ============================================================

def final_check_scan(c: TopicCheck) -> Tuple[str, str]:
    valid_counts = []
    min_ranges = []
    max_ranges = []

    for msg in c.samples:
        ranges = list(msg.ranges)
        valid = [r for r in ranges if math.isfinite(float(r)) and msg.range_min <= r <= msg.range_max]
        valid_counts.append(len(valid))
        if valid:
            min_ranges.append(min(valid))
            max_ranges.append(max(valid))

    if not valid_counts:
        return "FAIL", "no usable scan samples"

    avg_valid = sum(valid_counts) / len(valid_counts)

    if avg_valid < 30:
        return "FAIL", f"scan has too few valid ranges on average: {avg_valid:.1f}"

    return "PASS", f"avg_valid_ranges={avg_valid:.1f}"


def final_check_imu(c: TopicCheck) -> Tuple[str, str]:
    if not c.samples:
        return "FAIL", "no imu samples"

    q_norms = []
    ang_norms = []
    acc_norms = []
    all_zero_count = 0
    quat_zero_count = 0
    key_tuples = []

    for msg in c.samples:
        q = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration

        vals = [
            float(q.x), float(q.y), float(q.z), float(q.w),
            float(av.x), float(av.y), float(av.z),
            float(la.x), float(la.y), float(la.z),
        ]

        if all(abs(v) < 1e-8 for v in vals):
            all_zero_count += 1

        if abs(q.x) < 1e-8 and abs(q.y) < 1e-8 and abs(q.z) < 1e-8 and abs(q.w) < 1e-8:
            quat_zero_count += 1

        q_norms.append(math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w))
        ang_norms.append(vector_norm3(av.x, av.y, av.z))
        acc_norms.append(vector_norm3(la.x, la.y, la.z))
        key_tuples.append(tuple(round(v, 8) for v in vals))

    n = len(c.samples)

    if all_zero_count == n:
        return "FAIL", "IMU all sampled messages are completely zero"

    if quat_zero_count == n:
        return "FAIL", "IMU orientation quaternion is always all zero"

    avg_q = sum(q_norms) / len(q_norms)
    avg_ang = sum(ang_norms) / len(ang_norms)
    avg_acc = sum(acc_norms) / len(acc_norms)
    unique_samples = len(set(key_tuples))

    if avg_q < 0.5 or avg_q > 1.5:
        return "FAIL", f"IMU quaternion norm abnormal, avg_q_norm={avg_q:.3f}"

    if avg_acc < 0.5:
        return "FAIL", f"IMU acceleration almost zero, avg_acc_norm={avg_acc:.3f}; looks invalid"

    if n >= 20 and unique_samples <= 2:
        return "WARN", f"IMU data almost not changing, unique_samples={unique_samples}/{n}; check if driver is stuck"

    if avg_acc < 5.0 or avg_acc > 15.0:
        return "WARN", f"IMU received, but avg_acc_norm={avg_acc:.2f} is not near 9.8; check driver convention"

    return "PASS", f"avg_q_norm={avg_q:.3f}, avg_ang_norm={avg_ang:.4f}, avg_acc_norm={avg_acc:.2f}, unique={unique_samples}/{n}"


def final_check_odom(c: TopicCheck) -> Tuple[str, str]:
    if not c.samples:
        return "FAIL", "no odom samples"

    frames = set()
    child_frames = set()

    for msg in c.samples:
        frames.add(getattr(msg.header, "frame_id", ""))
        child_frames.add(getattr(msg, "child_frame_id", ""))

    if "" in frames:
        return "FAIL", "odom header.frame_id is empty"

    if "" in child_frames:
        return "FAIL", "odom child_frame_id is empty"

    return "PASS", f"frame_id={list(frames)[0]}, child_frame_id={list(child_frames)[0]}"


def final_check_image(c: TopicCheck) -> Tuple[str, str]:
    if not c.samples:
        return "FAIL", "no image samples"

    msg = c.samples[-1]
    return "PASS", f"{msg.width}x{msg.height}, encoding={msg.encoding}, frame_id={get_header_frame_id(msg)}"


def final_check_depth_image(c: TopicCheck) -> Tuple[str, str]:
    if not c.samples:
        return "FAIL", "no depth image samples"

    ratios = []
    min_values = []
    max_values = []

    for msg in c.samples:
        checked, valid, zero, min_v, max_v, ratio = depth_valid_pixel_stats(msg)
        ratios.append(ratio)
        if min_v is not None:
            min_values.append(min_v)
        if max_v is not None:
            max_values.append(max_v)

    avg_ratio = sum(ratios) / max(1, len(ratios))
    msg = c.samples[-1]

    if avg_ratio < 0.01:
        return "FAIL", f"too few valid depth pixels, avg_valid_ratio={avg_ratio:.3f}, encoding={msg.encoding}"

    if not min_values or not max_values:
        return "FAIL", "no valid depth range"

    return "PASS", f"{msg.width}x{msg.height}, encoding={msg.encoding}, avg_valid_ratio={avg_ratio:.3f}, range={min(min_values):.2f}-{max(max_values):.2f}m, frame_id={get_header_frame_id(msg)}"


def final_check_camera_info(c: TopicCheck) -> Tuple[str, str]:
    if not c.samples:
        return "FAIL", "no camera_info samples"

    msg = c.samples[-1]

    if not all(is_finite_number(v) for v in msg.k):
        return "FAIL", f"camera_info K matrix contains NaN/Inf, frame_id={get_header_frame_id(msg)}"

    if not all(is_finite_number(v) for v in msg.p):
        return "FAIL", f"camera_info P matrix contains NaN/Inf, frame_id={get_header_frame_id(msg)}"

    if msg.k[0] <= 0 or msg.k[4] <= 0:
        return "FAIL", f"camera_info focal length invalid: fx={msg.k[0]}, fy={msg.k[4]}, frame_id={get_header_frame_id(msg)}"

    return "PASS", f"{msg.width}x{msg.height}, fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}, frame_id={get_header_frame_id(msg)}"


def final_check_rgbd(c: TopicCheck) -> Tuple[str, str]:
    if not c.samples:
        return "FAIL", "no rgbd samples"

    msg = c.samples[-1]
    rgb = getattr(msg, "rgb", None)
    depth = getattr(msg, "depth", None)

    if rgb is not None and depth is not None:
        return "PASS", f"rgb={rgb.width}x{rgb.height}, depth={depth.width}x{depth.height}"

    return "PASS", "received"


FINAL_VALIDATORS = {
    "scan": final_check_scan,
    "imu": final_check_imu,
    "odom": final_check_odom,
    "image": final_check_image,
    "depth_image": final_check_depth_image,
    "camera_info": final_check_camera_info,
    "rgbd": final_check_rgbd,
}


# ============================================================
# ROS 检查节点
# ============================================================

class DiguaHwCheckNode(Node):
    def __init__(self, topic_checks: List[TopicCheck], node_name: str):
        super().__init__(node_name)

        self.topic_checks = topic_checks

        self.sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subscriptions_holder = []

        for check in self.topic_checks:
            try:
                msg_cls = get_message(check.type_name)
            except Exception as e:
                check.import_error = str(e)
                continue

            def make_cb(c: TopicCheck):
                def cb(msg):
                    c.add_sample(msg)

                    ok_basic, info_basic = validate_msg_basic(c, msg)
                    if not ok_basic:
                        c.per_message_invalid_count += 1
                        c.last_invalid_info = info_basic
                        return

                    validator = MESSAGE_VALIDATORS.get(c.validator_name)
                    if validator is None:
                        return

                    try:
                        ok, info = validator(msg)
                    except Exception as e:
                        ok, info = False, f"validator exception: {e}"

                    if not ok:
                        c.per_message_invalid_count += 1
                        c.last_invalid_info = info

                return cb

            sub = self.create_subscription(
                msg_cls,
                check.topic,
                make_cb(check),
                self.sensor_qos,
            )
            self.subscriptions_holder.append(sub)
            check.subscription_created = True


class DiguaTfCheckNode(Node):
    def __init__(self):
        super().__init__("digua_hw_check_tf")
        self.tf_buffer = None
        self.tf_listener = None

        if Buffer is not None and TransformListener is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)


# ============================================================
# 检查逻辑
# ============================================================

def check_devices() -> List[Tuple[str, str, str]]:
    results = []

    for item in DEVICE_CHECKS:
        name = item["name"]
        path = item["path"]
        required = item.get("required", True)
        label = f"device {path}"

        if os.path.exists(path):
            if os.access(path, os.R_OK | os.W_OK):
                results.append(("PASS", label, name))
            else:
                results.append(("WARN", label, f"{name}, exists but current user may not have read/write permission"))
        else:
            if required:
                results.append(("FAIL", label, f"{name}, not found"))
            else:
                results.append(("SKIP", label, f"{name}, not found"))

    try:
        output = subprocess.check_output(["lsusb"], stderr=subprocess.STDOUT, timeout=2.0)
        text = output.decode(errors="ignore").lower()
        matched = [kw for kw in USB_CAMERA_KEYWORDS if kw.lower() in text]

        if matched:
            results.append(("PASS", "usb camera", f"matched keywords: {', '.join(matched)}"))
        else:
            results.append(("WARN", "usb camera", "no Orbbec/Astra keyword found in lsusb"))
    except Exception as e:
        results.append(("WARN", "usb camera", f"lsusb check failed: {e}"))

    return results


def evaluate_topics(node: DiguaHwCheckNode) -> List[Tuple[str, str, str]]:
    results = []

    graph_topics = {}
    try:
        for name, types in node.get_topic_names_and_types():
            graph_topics[name] = types
    except Exception:
        graph_topics = {}

    for c in node.topic_checks:
        label = f"topic {c.topic}"

        if c.import_error:
            if c.required:
                results.append(("FAIL", label, f"cannot import {c.type_name}: {c.import_error}"))
            else:
                results.append(("SKIP", label, f"cannot import {c.type_name}: {c.import_error}"))
            continue

        if c.topic not in graph_topics and c.count == 0:
            if c.required:
                results.append(("FAIL", label, "topic not found and no message received"))
            else:
                results.append(("SKIP", label, "topic not found"))
            continue

        if c.count == 0:
            if c.required:
                results.append(("FAIL", label, "no message received"))
            else:
                results.append(("SKIP", label, "no message received"))
            continue

        expected_type = c.type_name
        actual_types = graph_topics.get(c.topic, [])
        type_warn = ""
        if actual_types and expected_type not in actual_types:
            type_warn = f"type mismatch? expected={expected_type}, actual={actual_types}; "

        hz = c.hz()

        final_validator = FINAL_VALIDATORS.get(c.validator_name)
        if final_validator is None:
            status, detail = "PASS", f"received {c.count} messages"
        else:
            status, detail = final_validator(c)

        hz_detail = ""
        if c.min_hz > 0:
            if hz is None:
                if status == "PASS":
                    status = "WARN"
                hz_detail = "received too few messages to estimate hz; "
            elif hz < c.min_hz:
                if status == "PASS":
                    status = "WARN"
                hz_detail = f"{fmt_hz(hz)} < {c.min_hz:.1f} Hz; "
            else:
                hz_detail = f"{fmt_hz(hz)}; "

        if c.per_message_invalid_count > 0:
            if status == "PASS":
                status = "WARN"
            detail += f", invalid_msg={c.per_message_invalid_count}, last_invalid={c.last_invalid_info}"

        results.append((status, label, type_warn + hz_detail + detail))

    return results


def evaluate_tf(node: DiguaTfCheckNode) -> List[Tuple[str, str, str]]:
    results = []

    if node.tf_buffer is None:
        return [("FAIL", "tf", "tf2_ros not available")]

    for item in TF_CHECKS:
        name = item["name"]
        target = item["target"]
        source = item["source"]
        required = item.get("required", True)
        label = f"tf {name}"

        try:
            trans = node.tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=0.2),
            )

            t = trans.transform.translation
            r = trans.transform.rotation
            yaw = yaw_from_quat(r)

            if yaw is None:
                detail = f"x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}"
            else:
                detail = f"x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}, yaw={math.degrees(yaw):.1f}deg"

            results.append(("PASS", label, detail))

        except Exception as e:
            if required:
                results.append(("FAIL", label, f"unavailable: {e}"))
            else:
                results.append(("WARN", label, f"unavailable: {e}"))

    return results


def run_topic_phase(phase: dict, duration_override: Optional[float], tf_node=None) -> List[Tuple[str, str, str]]:
    duration = float(duration_override) if duration_override is not None else float(phase["duration"])

    topic_checks = [
        TopicCheck(
            name=item["name"],
            topic=item["topic"],
            type_name=item["type"],
            min_hz=float(item.get("min_hz", 0.0)),
            required=bool(item.get("required", True)),
            validator_name=item.get("validator", ""),
            max_samples=int(item.get("max_samples", 50)),
        )
        for item in phase["topics"]
    ]

    safe_name = phase["name"].lower().replace(" ", "_").replace("-", "_")
    node = DiguaHwCheckNode(topic_checks, f"digua_hw_check_{safe_name}")

    print("")
    print(f"---- {phase['name']} checks ----")
    print(f"Collecting messages for {duration:.1f}s...")

    start = time.monotonic()
    while time.monotonic() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.03)
        if tf_node is not None:
            rclpy.spin_once(tf_node, timeout_sec=0.0)

    results = evaluate_topics(node)
    for status, item, detail in results:
        print_result(status, item, detail)

    node.destroy_node()
    return results


def run_tf_phase(node=None) -> List[Tuple[str, str, str]]:
    print("")
    print("---- TF checks ----")

    own_node = False
    if node is None:
        node = DiguaTfCheckNode()
        own_node = True

    start = time.monotonic()
    while time.monotonic() - start < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    results = evaluate_tf(node)
    for status, item, detail in results:
        print_result(status, item, detail)

    if own_node:
        node.destroy_node()

    return results


def main():
    parser = argparse.ArgumentParser(description="Digua robot hardware self-check")
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="override every topic phase duration, e.g. --duration 5",
    )
    parser.add_argument(
        "--skip-tf",
        action="store_true",
        help="skip TF checks",
    )
    parser.add_argument(
        "--skip-device",
        action="store_true",
        help="skip Linux device checks",
    )
    parser.add_argument(
        "--base-only",
        action="store_true",
        help="only check /scan, /imu, /odom and /odometry/filtered",
    )
    parser.add_argument(
        "--camera-only",
        action="store_true",
        help="only check camera image, camera_info and rgbd topics",
    )
    args = parser.parse_args()

    print("")
    print("========== Digua Robot Hardware Self Check ==========")
    print("Mode: phased topic check")
    print("")

    device_results = []
    if not args.skip_device:
        print("---- Device checks ----")
        device_results = check_devices()
        for status, item, detail in device_results:
            print_result(status, item, detail)

    rclpy.init()

    selected_phases = CHECK_PHASES

    if args.base_only:
        selected_phases = [CHECK_PHASES[0]]

    if args.camera_only:
        selected_phases = CHECK_PHASES[1:]

    tf_node = None
    if not args.skip_tf:
        tf_node = DiguaTfCheckNode()

    topic_results = []
    for phase in selected_phases:
        topic_results.extend(run_topic_phase(phase, args.duration, tf_node))

    tf_results = []
    if not args.skip_tf:
        tf_results = run_tf_phase(tf_node)
        tf_node.destroy_node()

    rclpy.shutdown()

    all_results = device_results + topic_results + tf_results
    fail_count = sum(1 for r in all_results if r[0] == "FAIL")
    warn_count = sum(1 for r in all_results if r[0] == "WARN")
    pass_count = sum(1 for r in all_results if r[0] == "PASS")
    skip_count = sum(1 for r in all_results if r[0] == "SKIP")

    print("")
    print("---- Summary ----")
    print(f"PASS: {pass_count}")
    print(f"WARN: {warn_count}")
    print(f"FAIL: {fail_count}")
    print(f"SKIP: {skip_count}")

    if fail_count > 0:
        print("")
        print("Result: FAIL")
        sys.exit(2)

    if warn_count > 0:
        print("")
        print("Result: WARN")
        sys.exit(1)

    print("")
    print("Result: PASS")
    sys.exit(0)


if __name__ == "__main__":
    main()
