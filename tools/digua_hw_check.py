#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import math
import argparse
import subprocess
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Any

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
# 配置区
# ============================================================

DEVICE_CHECKS = [
    {
        "name": "base serial",
        "path": "/dev/ttyS1",
        "required": True,
    },
    {
        "name": "YDLIDAR X2 serial",
        "path": "/dev/ttyUSB0",
        "required": True,
    },
]

USB_CAMERA_KEYWORDS = [
    "orbbec",
    "astra",
    "2bc5",
]

TOPIC_CHECKS_CONFIG = [
    {
        "name": "lidar scan",
        "topic": "/scan",
        "type": "sensor_msgs/msg/LaserScan",
        "min_hz": 4.0,
        "required": True,
        "validator": "scan",
    },
    {
        "name": "imu",
        "topic": "/imu",
        "type": "sensor_msgs/msg/Imu",
        "min_hz": 10.0,
        "required": True,
        "validator": "imu",
    },
    {
        "name": "raw odom",
        "topic": "/odom",
        "type": "nav_msgs/msg/Odometry",
        "min_hz": 5.0,
        "required": True,
        "validator": "odom",
    },
    {
        "name": "ekf odom",
        "topic": "/odometry/filtered",
        "type": "nav_msgs/msg/Odometry",
        "min_hz": 5.0,
        "required": True,
        "validator": "odom",
    },
    {
        "name": "color image",
        "topic": "/camera/color/image_raw",
        "type": "sensor_msgs/msg/Image",
        "min_hz": 5.0,
        "required": True,
        "validator": "image",
    },
    {
        "name": "depth image",
        "topic": "/camera/depth/image_raw",
        "type": "sensor_msgs/msg/Image",
        "min_hz": 5.0,
        "required": True,
        "validator": "image",
    },
    {
        "name": "color camera_info",
        "topic": "/camera/color/camera_info",
        "type": "sensor_msgs/msg/CameraInfo",
        "min_hz": 0.0,
        "required": True,
        "validator": "camera_info",
    },
    {
        "name": "depth camera_info",
        "topic": "/camera/depth/camera_info",
        "type": "sensor_msgs/msg/CameraInfo",
        "min_hz": 0.0,
        "required": True,
        "validator": "camera_info",
    },
    {
        "name": "rgbd sync",
        "topic": "/camera/rgbd_image",
        "type": "rtabmap_msgs/msg/RGBDImage",
        "min_hz": 5.0,
        "required": True,
        "validator": "rgbd",
    },
]

TF_CHECKS = [
    {
        "name": "odom -> base_footprint",
        "target": "odom",
        "source": "base_footprint",
        "required": True,
    },
    {
        "name": "base_footprint -> base_link",
        "target": "base_footprint",
        "source": "base_link",
        "required": True,
    },
    {
        "name": "base_link -> laser_frame",
        "target": "base_link",
        "source": "laser_frame",
        "required": True,
    },
    {
        "name": "base_link -> camera_link",
        "target": "base_link",
        "source": "camera_link",
        "required": True,
    },
    {
        "name": "camera_link -> camera_color_optical_frame",
        "target": "camera_link",
        "source": "camera_color_optical_frame",
        "required": True,
    },
    {
        "name": "camera_link -> camera_depth_optical_frame",
        "target": "camera_link",
        "source": "camera_depth_optical_frame",
        "required": True,
    },
]


# ============================================================
# 通用工具
# ============================================================

def near_zero(x: float, eps: float = 1e-6) -> bool:
    try:
        return abs(float(x)) < eps
    except Exception:
        return False


def is_finite_number(x: Any) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def safe_float(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except Exception:
        return default


def yaw_from_quat(q) -> Optional[float]:
    try:
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    except Exception:
        return None


def vector_norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


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


def get_header_stamp_sec(msg) -> float:
    try:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
    except Exception:
        return 0.0


@dataclass
class TopicCheck:
    name: str
    topic: str
    type_name: str
    min_hz: float
    required: bool
    validator_name: str

    count: int = 0
    times: List[float] = field(default_factory=list)
    samples: List[Any] = field(default_factory=list)
    max_samples: int = 80

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
# 单条消息基础检查
# ============================================================

def validate_msg_basic(c: TopicCheck, msg) -> Tuple[bool, str]:
    frame_id = get_header_frame_id(msg)
    stamp_sec = get_header_stamp_sec(msg)

    # 有些 /tf_static 或部分驱动可能 stamp 为 0，但传感器话题一般不该长期为 0。
    # 这里不直接失败，只作为后续提示。
    if frame_id == "" and c.validator_name in ["scan", "imu", "odom", "image", "camera_info"]:
        return False, "header.frame_id is empty"

    return True, "basic ok"


def validate_scan_msg(msg) -> Tuple[bool, str]:
    if not hasattr(msg, "ranges"):
        return False, "no ranges field"

    ranges = list(msg.ranges)
    if len(ranges) == 0:
        return False, "empty ranges"

    finite = [r for r in ranges if math.isfinite(r)]
    valid = [r for r in finite if msg.range_min <= r <= msg.range_max]

    if len(valid) < 10:
        return False, f"valid ranges too few: {len(valid)}/{len(ranges)}"

    # 检查是不是几乎全一样。雷达正常扫房间时不太可能所有有效距离完全一样。
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

    q_norm = vector_norm3(q.x, q.y, q.z) ** 2 + q.w * q.w
    q_norm = math.sqrt(q_norm)

    ang_norm = vector_norm3(av.x, av.y, av.z)
    acc_norm = vector_norm3(la.x, la.y, la.z)

    all_zero = all(near_zero(v, 1e-8) for v in values)
    if all_zero:
        return False, "imu all fields are zero"

    quat_all_zero = near_zero(q.x) and near_zero(q.y) and near_zero(q.z) and near_zero(q.w)
    if quat_all_zero:
        return False, "orientation quaternion is all zero"

    if q_norm < 0.5 or q_norm > 1.5:
        return False, f"orientation quaternion norm abnormal: {q_norm:.3f}"

    # 对你的底盘 IMU 来说，静止时 linear_acceleration 一般应该接近 9.8。
    # 但不同驱动可能会做重力补偿，所以这里单条消息不直接 fail，最终统计里再 WARN。
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

    yaw = yaw_from_quat(q)
    if yaw is None:
        return True, f"x={p.x:.3f}, y={p.y:.3f}"

    return True, f"x={p.x:.3f}, y={p.y:.3f}, yaw={math.degrees(yaw):.1f}deg"


def validate_image_msg(msg) -> Tuple[bool, str]:
    if msg.width <= 0 or msg.height <= 0:
        return False, f"invalid image size: {msg.width}x{msg.height}"

    if len(msg.data) == 0:
        return False, "image data is empty"

    expected_step_min = msg.width
    if msg.step < expected_step_min:
        return False, f"image step too small: step={msg.step}, width={msg.width}"

    return True, f"{msg.width}x{msg.height}, encoding={msg.encoding}"


def validate_camera_info_msg(msg) -> Tuple[bool, str]:
    if msg.width <= 0 or msg.height <= 0:
        return False, f"invalid camera_info size: {msg.width}x{msg.height}"

    if len(msg.k) != 9:
        return False, "camera_info K matrix invalid"

    k_abs_sum = sum(abs(float(v)) for v in msg.k)
    if k_abs_sum < 1e-6:
        return False, "camera_info K matrix is all zero"

    if msg.k[0] <= 0 or msg.k[4] <= 0:
        return False, f"camera_info focal length invalid: fx={msg.k[0]}, fy={msg.k[4]}"

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
    "camera_info": validate_camera_info_msg,
    "rgbd": validate_rgbd_msg,
}


# ============================================================
# 多条消息统计检查：检测“全 0”“卡死”“明显不合理”
# ============================================================

def final_check_scan(c: TopicCheck) -> Tuple[str, str]:
    valid_counts = []
    min_ranges = []
    max_ranges = []

    for msg in c.samples:
        ranges = list(msg.ranges)
        valid = [r for r in ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        valid_counts.append(len(valid))
        if valid:
            min_ranges.append(min(valid))
            max_ranges.append(max(valid))

    if not valid_counts:
        return "FAIL", "no usable scan samples"

    avg_valid = sum(valid_counts) / len(valid_counts)

    if avg_valid < 30:
        return "FAIL", f"scan has too few valid ranges on average: {avg_valid:.1f}"

    # 多帧 min/max 完全不变不一定是错，但值得提示。
    if len(set(round(v, 3) for v in min_ranges)) <= 1 and len(set(round(v, 3) for v in max_ranges)) <= 1 and len(c.samples) >= 10:
        return "WARN", f"scan valid, but min/max almost unchanged, avg_valid={avg_valid:.1f}"

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
            safe_float(q.x), safe_float(q.y), safe_float(q.z), safe_float(q.w),
            safe_float(av.x), safe_float(av.y), safe_float(av.z),
            safe_float(la.x), safe_float(la.y), safe_float(la.z),
        ]

        if all(abs(v) < 1e-8 for v in vals):
            all_zero_count += 1

        if abs(q.x) < 1e-8 and abs(q.y) < 1e-8 and abs(q.z) < 1e-8 and abs(q.w) < 1e-8:
            quat_zero_count += 1

        q_norms.append(math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w))
        ang_norms.append(vector_norm3(av.x, av.y, av.z))
        acc_norms.append(vector_norm3(la.x, la.y, la.z))

        # 用较高精度判断是否“每帧完全一样”
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
        return "FAIL", f"IMU acceleration almost zero, avg_acc_norm={avg_acc:.3f}; looks like invalid IMU data"

    # 静止时角速度接近 0 是正常的，所以不能因为 avg_ang 小就判错。
    # 但如果完整 5 秒内所有字段完全不变，通常说明数据卡死或驱动假数据。
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


def final_check_camera_info(c: TopicCheck) -> Tuple[str, str]:
    if not c.samples:
        return "FAIL", "no camera_info samples"

    msg = c.samples[-1]
    return "PASS", f"{msg.width}x{msg.height}, fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, frame_id={get_header_frame_id(msg)}"


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
    "camera_info": final_check_camera_info,
    "rgbd": final_check_rgbd,
}


# ============================================================
# ROS 节点
# ============================================================

class DiguaHwCheckNode(Node):
    def __init__(self, topic_checks: List[TopicCheck]):
        super().__init__("digua_hw_check")

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
        if actual_types and expected_type not in actual_types:
            results.append(("WARN", label, f"type mismatch? expected={expected_type}, actual={actual_types}"))

        hz = c.hz()

        if c.min_hz > 0:
            if hz is None:
                results.append(("WARN", label, f"received only {c.count} msg, cannot estimate hz"))
                continue
            if hz < c.min_hz:
                results.append(("WARN", label, f"{fmt_hz(hz)} < {c.min_hz:.1f} Hz"))

        final_validator = FINAL_VALIDATORS.get(c.validator_name)
        if final_validator is None:
            status, detail = "PASS", f"received {c.count} messages"
        else:
            status, detail = final_validator(c)

        if c.per_message_invalid_count > 0:
            invalid_detail = f", invalid_msg={c.per_message_invalid_count}, last_invalid={c.last_invalid_info}"
            if status == "PASS":
                status = "WARN"
            detail += invalid_detail

        hz_detail = ""
        if c.min_hz > 0:
            hz_detail = f"{fmt_hz(hz)}, "

        results.append((status, label, hz_detail + detail))

    return results


def evaluate_tf(node: DiguaHwCheckNode) -> List[Tuple[str, str, str]]:
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


def main():
    parser = argparse.ArgumentParser(description="Digua robot hardware self-check")
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="how many seconds to collect ROS messages, default: 5.0",
    )
    parser.add_argument(
        "--skip-tf",
        action="store_true",
        help="skip TF checks",
    )
    args = parser.parse_args()

    print("")
    print("========== Digua Robot Hardware Self Check ==========")
    print(f"Collect duration: {args.duration:.1f}s")
    print("")

    print("---- Device checks ----")
    device_results = check_devices()
    for status, item, detail in device_results:
        print_result(status, item, detail)

    rclpy.init()

    topic_checks = [
        TopicCheck(
            name=item["name"],
            topic=item["topic"],
            type_name=item["type"],
            min_hz=float(item.get("min_hz", 0.0)),
            required=bool(item.get("required", True)),
            validator_name=item.get("validator", ""),
        )
        for item in TOPIC_CHECKS_CONFIG
    ]

    node = DiguaHwCheckNode(topic_checks)

    print("")
    print("---- ROS topic checks ----")
    print("Collecting messages...")

    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    topic_results = evaluate_topics(node)
    for status, item, detail in topic_results:
        print_result(status, item, detail)

    tf_results = []
    if not args.skip_tf:
        print("")
        print("---- TF checks ----")

        extra_start = time.monotonic()
        while time.monotonic() - extra_start < 1.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        tf_results = evaluate_tf(node)
        for status, item, detail in tf_results:
            print_result(status, item, detail)

    node.destroy_node()
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
