#!/usr/bin/env python3
import json
import math
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import tf2_ros


class SemanticObserverNode(Node):
    def __init__(self):
        super().__init__("semantic_observer_node")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")

        self.declare_parameter("detections_topic", "/semantic/detections_json")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("observations_topic", "/semantic/observations_json")

        self.declare_parameter("depth_min", 0.45)
        self.declare_parameter("depth_max", 3.5)
        self.declare_parameter("roi_scale", 0.4)

        self.declare_parameter("class_whitelist", ["chair", "table", "desk", "couch", "bottle", "cup", "refrigerator", "microwave", "sink", "toilet", "tv", "laptop", "person"])
        self.declare_parameter("dynamic_classes", ["person", "dog", "cat"])

        self.map_frame = self.get_parameter("map_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value

        self.detections_topic = self.get_parameter("detections_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.observations_topic = self.get_parameter("observations_topic").value

        self.depth_min = float(self.get_parameter("depth_min").value)
        self.depth_max = float(self.get_parameter("depth_max").value)
        self.roi_scale = float(self.get_parameter("roi_scale").value)

        self.class_whitelist = [str(x).strip().lower() for x in self.get_parameter("class_whitelist").value]
        self.dynamic_classes = [str(x).strip().lower() for x in self.get_parameter("dynamic_classes").value]

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_encoding = None
        self.latest_camera_info: Optional[CameraInfo] = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        self.detections_sub = self.create_subscription(
            String,
            self.detections_topic,
            self.detections_callback,
            10
        )

        self.obs_pub = self.create_publisher(
            String,
            self.observations_topic,
            10
        )

        self.get_logger().info("semantic_observer_node started")
        self.get_logger().info(f"detections_topic: {self.detections_topic}")
        self.get_logger().info(f"depth_topic: {self.depth_topic}")
        self.get_logger().info(f"camera_info_topic: {self.camera_info_topic}")
        self.get_logger().info(f"observations_topic: {self.observations_topic}")
        self.get_logger().info(f"TF: {self.camera_frame} -> {self.map_frame}")

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth = np.array(depth)
            self.latest_depth_encoding = msg.encoding
        except Exception as e:
            self.get_logger().warn(f"failed to convert depth image: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg

    def detections_callback(self, msg: String):
        if self.latest_depth is None:
            self.get_logger().warn("no depth image received yet")
            return

        if self.latest_camera_info is None:
            self.get_logger().warn("no camera_info received yet")
            return

        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"invalid detections json: {e}")
            return

        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            self.get_logger().warn("detections field is not a list")
            return

        observations = []

        for det in detections:
            obs = self.handle_one_detection(det)
            if obs is not None:
                observations.append(obs)

        if not observations:
            return

        out = {
            "stamp": self.get_clock().now().nanoseconds / 1e9,
            "frame_id": self.map_frame,
            "source_frame": self.camera_frame,
            "observations": observations
        }

        out_msg = String()
        out_msg.data = json.dumps(out, ensure_ascii=False)
        self.obs_pub.publish(out_msg)

    def handle_one_detection(self, det: dict) -> Optional[dict]:
        label = str(det.get("label", "unknown")).strip().lower()
        confidence = float(det.get("confidence", 0.0))

        if self.class_whitelist and label not in self.class_whitelist:
            return None

        bbox = det.get("bbox", {})
        try:
            cx = float(bbox["cx"])
            cy = float(bbox["cy"])
            w = float(bbox["w"])
            h = float(bbox["h"])
        except Exception:
            self.get_logger().warn(f"invalid bbox in detection: {det}")
            return None

        depth = self.get_median_depth(cx, cy, w, h)
        if depth is None:
            self.get_logger().warn(f"no valid depth for label={label}, bbox={bbox}")
            return None

        point_camera = self.back_project(cx, cy, depth)
        if point_camera is None:
            return None

        point_map = self.transform_point_to_map(point_camera)
        if point_map is None:
            return None

        return {
            "label": label,
            "confidence": confidence,
            "depth": depth,
            "position_camera": {
                "x": point_camera[0],
                "y": point_camera[1],
                "z": point_camera[2]
            },
            "position_map": {
                "x": point_map[0],
                "y": point_map[1],
                "z": point_map[2]
            },
            "status_hint": "dynamic" if label in self.dynamic_classes else "candidate"
        }

    def get_median_depth(self, cx: float, cy: float, w: float, h: float) -> Optional[float]:
        depth_img = self.latest_depth
        if depth_img is None:
            return None

        img_h, img_w = depth_img.shape[:2]

        roi_w = max(2, int(w * self.roi_scale))
        roi_h = max(2, int(h * self.roi_scale))

        x1 = int(cx - roi_w / 2)
        x2 = int(cx + roi_w / 2)
        y1 = int(cy - roi_h / 2)
        y2 = int(cy + roi_h / 2)

        x1 = max(0, min(img_w - 1, x1))
        x2 = max(0, min(img_w, x2))
        y1 = max(0, min(img_h - 1, y1))
        y2 = max(0, min(img_h, y2))

        if x2 <= x1 or y2 <= y1:
            return None

        roi = depth_img[y1:y2, x1:x2].astype(np.float32)

        if self.latest_depth_encoding in ["16UC1", "mono16"]:
            roi = roi / 1000.0

        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > self.depth_min) & (valid < self.depth_max)]

        if valid.size < 5:
            return None

        return float(np.median(valid))

    def back_project(self, u: float, v: float, z: float) -> Optional[Tuple[float, float, float]]:
        info = self.latest_camera_info
        if info is None:
            return None

        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warn("invalid camera intrinsics")
            return None

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return (float(x), float(y), float(z))

    def transform_point_to_map(self, point_camera: Tuple[float, float, float]) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed {self.map_frame} <- {self.camera_frame}: {e}")
            return None

        t = tf.transform.translation
        q = tf.transform.rotation

        rotated = self.rotate_vector_by_quaternion(
            np.array(point_camera, dtype=np.float64),
            q.x, q.y, q.z, q.w
        )

        point_map = np.array([t.x, t.y, t.z], dtype=np.float64) + rotated

        return (float(point_map[0]), float(point_map[1]), float(point_map[2]))

    @staticmethod
    def rotate_vector_by_quaternion(v: np.ndarray, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
        q_vec = np.array([qx, qy, qz], dtype=np.float64)
        q_w = float(qw)

        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-12:
            return v

        q_vec = q_vec / norm
        q_w = q_w / norm

        t = 2.0 * np.cross(q_vec, v)
        return v + q_w * t + np.cross(q_vec, t)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
