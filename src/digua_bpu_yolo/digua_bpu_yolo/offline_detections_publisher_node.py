#!/usr/bin/env python3
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OfflineDetectionsPublisherNode(Node):
    def __init__(self):
        super().__init__("offline_detections_publisher_node")

        self.declare_parameter(
            "detections_json",
            "/home/sunrise/rdk_normal_yolo_test/result/digua_oiv7_offline_detections.json"
        )
        self.declare_parameter("topic", "/semantic/detections_json")
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        # 模型输出坐标是 640x640
        self.declare_parameter("model_width", 640.0)
        self.declare_parameter("model_height", 640.0)

        # Astra 彩色图 / 深度图是 640x480
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)

        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("min_confidence", 0.0)
        self.declare_parameter("lowercase_label", True)

        self.detections_json = self.get_parameter("detections_json").value
        self.topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.model_width = float(self.get_parameter("model_width").value)
        self.model_height = float(self.get_parameter("model_height").value)
        self.image_width = float(self.get_parameter("image_width").value)
        self.image_height = float(self.get_parameter("image_height").value)

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.lowercase_label = bool(self.get_parameter("lowercase_label").value)

        self.pub = self.create_publisher(String, self.topic, 10)

        self.detections = self.load_and_convert_detections()

        period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(period, self.publish_once)

        self.get_logger().info("offline_detections_publisher_node started")
        self.get_logger().info(f"detections_json: {self.detections_json}")
        self.get_logger().info(f"publish topic: {self.topic}")
        self.get_logger().info(f"frame_id: {self.frame_id}")
        self.get_logger().info(
            f"model size: {self.model_width}x{self.model_height}, "
            f"image size: {self.image_width}x{self.image_height}"
        )
        self.get_logger().info(f"loaded detections: {len(self.detections)}")

    def load_and_convert_detections(self):
        path = Path(self.detections_json)
        if not path.exists():
            raise FileNotFoundError(f"detections_json not found: {path}")

        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        raw_dets = data.get("detections", [])
        converted = []

        sx = self.image_width / self.model_width
        sy = self.image_height / self.model_height

        for det in raw_dets:
            label = str(det.get("label", "unknown")).strip()
            if self.lowercase_label:
                label = label.lower()

            conf = float(det.get("confidence", 0.0))
            if conf < self.min_confidence:
                continue

            bbox = det.get("bbox", {})
            try:
                cx_m = float(bbox["cx"])
                cy_m = float(bbox["cy"])
                w_m = float(bbox["w"])
                h_m = float(bbox["h"])
            except Exception:
                self.get_logger().warn(f"bad bbox, skip: {det}")
                continue

            cx = cx_m * sx
            cy = cy_m * sy
            w = w_m * sx
            h = h_m * sy

            # 限制在图像范围内，避免后续取深度越界
            cx = max(0.0, min(self.image_width - 1.0, cx))
            cy = max(0.0, min(self.image_height - 1.0, cy))
            w = max(1.0, min(self.image_width, w))
            h = max(1.0, min(self.image_height, h))

            converted.append({
                "label": label,
                "confidence": conf,
                "bbox": {
                    "cx": cx,
                    "cy": cy,
                    "w": w,
                    "h": h
                }
            })

        return converted

    def publish_once(self):
        msg = String()

        payload = {
            "stamp": self.get_clock().now().nanoseconds / 1e9,
            "frame_id": self.frame_id,
            "image_width": int(self.image_width),
            "image_height": int(self.image_height),
            "source": "offline_oiv7_postprocess",
            "detections": self.detections,
        }

        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)

        labels = [d["label"] for d in self.detections]
        self.get_logger().info(
            f"published {len(self.detections)} detections to {self.topic}: {labels}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OfflineDetectionsPublisherNode()
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
