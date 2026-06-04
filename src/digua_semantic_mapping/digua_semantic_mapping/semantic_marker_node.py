#!/usr/bin/env python3
import json
import os
import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray


class SemanticMarkerNode(Node):
    def __init__(self):
        super().__init__("semantic_marker_node")

        self.declare_parameter(
            "semantic_map_path",
            "/home/sunrise/digua_ws/digua_maps/semantic/semantic_map.json"
        )
        self.declare_parameter("marker_topic", "/semantic/markers")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("refresh_period", 1.0)

        self.semantic_map_path = self.get_parameter("semantic_map_path").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.refresh_period = float(self.get_parameter("refresh_period").value)

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.timer = self.create_timer(self.refresh_period, self.publish_markers)

        self.last_warn_time = 0.0

        self.get_logger().info("semantic_marker_node started")
        self.get_logger().info(f"semantic_map_path: {self.semantic_map_path}")
        self.get_logger().info(f"marker_topic: {self.marker_topic}")
        self.get_logger().info(f"map_frame: {self.map_frame}")

    def load_semantic_map(self) -> Dict[str, Any]:
        if not os.path.exists(self.semantic_map_path):
            now = time.time()
            if now - self.last_warn_time > 5.0:
                self.get_logger().warn(f"semantic map file not found: {self.semantic_map_path}")
                self.last_warn_time = now
            return {"objects": []}

        try:
            with open(self.semantic_map_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().warn(f"failed to load semantic map: {e}")
            return {"objects": []}

    def publish_markers(self):
        semantic_map = self.load_semantic_map()
        objects = semantic_map.get("objects", [])

        marker_array = MarkerArray()

        for obj in objects:
            try:
                obj_id = int(obj.get("id", 0))
                label = str(obj.get("label", "unknown"))
                x = float(obj.get("x", 0.0))
                y = float(obj.get("y", 0.0))
                z = float(obj.get("z", 0.0))
                confidence = float(obj.get("confidence", 0.0))
                observations = int(obj.get("observations", 0))
                status = str(obj.get("status", "candidate"))
            except Exception:
                continue

            point_marker = self.make_point_marker(
                obj_id=obj_id,
                label=label,
                x=x,
                y=y,
                z=z,
                status=status
            )

            text_marker = self.make_text_marker(
                obj_id=obj_id,
                label=label,
                x=x,
                y=y,
                z=z,
                confidence=confidence,
                observations=observations,
                status=status
            )

            marker_array.markers.append(point_marker)
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def make_point_marker(self, obj_id: int, label: str, x: float, y: float, z: float, status: str) -> Marker:
        marker = Marker()

        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "semantic_objects"
        marker.id = obj_id * 10
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = max(z, 0.05)
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18

        if status == "confirmed":
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.1
            marker.color.a = 0.9
        else:
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.1
            marker.color.a = 0.9

        return marker

    def make_text_marker(
        self,
        obj_id: int,
        label: str,
        x: float,
        y: float,
        z: float,
        confidence: float,
        observations: int,
        status: str
    ) -> Marker:
        marker = Marker()

        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "semantic_labels"
        marker.id = obj_id * 10 + 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = max(z + 0.35, 0.35)
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.18

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = f"{label}#{obj_id}\n{status} conf={confidence:.2f} obs={observations}"

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMarkerNode()
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
