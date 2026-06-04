#!/usr/bin/env python3
import json
import math
import os
import time
from typing import Dict, Any, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SemanticFusionNode(Node):
    def __init__(self):
        super().__init__("semantic_fusion_node")

        self.declare_parameter("observations_topic", "/semantic/observations_json")
        self.declare_parameter("semantic_map_topic", "/semantic/map_json")
        self.declare_parameter("semantic_map_path", "/home/sunrise/digua_ws/digua_maps/semantic/semantic_map.json")

        self.declare_parameter("merge_distance_default", 0.5)
        self.declare_parameter("min_observations_confirmed", 3)

        self.declare_parameter(
            "dynamic_classes",
            ["person", "dog", "cat"]
        )

        self.observations_topic = self.get_parameter("observations_topic").value
        self.semantic_map_topic = self.get_parameter("semantic_map_topic").value
        self.semantic_map_path = self.get_parameter("semantic_map_path").value

        self.merge_distance_default = float(self.get_parameter("merge_distance_default").value)
        self.min_observations_confirmed = int(self.get_parameter("min_observations_confirmed").value)
        self.dynamic_classes = list(self.get_parameter("dynamic_classes").value)

        self.semantic_map = self.load_semantic_map()

        self.obs_sub = self.create_subscription(
            String,
            self.observations_topic,
            self.observations_callback,
            10
        )

        self.map_pub = self.create_publisher(
            String,
            self.semantic_map_topic,
            10
        )

        self.timer = self.create_timer(2.0, self.publish_map)

        self.get_logger().info("semantic_fusion_node started")
        self.get_logger().info(f"observations_topic: {self.observations_topic}")
        self.get_logger().info(f"semantic_map_topic: {self.semantic_map_topic}")
        self.get_logger().info(f"semantic_map_path: {self.semantic_map_path}")
        self.get_logger().info(f"merge_distance_default: {self.merge_distance_default}")
        self.get_logger().info(f"min_observations_confirmed: {self.min_observations_confirmed}")

    def load_semantic_map(self) -> Dict[str, Any]:
        if os.path.exists(self.semantic_map_path):
            try:
                with open(self.semantic_map_path, "r", encoding="utf-8") as f:
                    data = json.load(f)

                if "objects" not in data:
                    data["objects"] = []

                self.get_logger().info(
                    f"loaded semantic map: {self.semantic_map_path}, objects={len(data['objects'])}"
                )
                return data
            except Exception as e:
                self.get_logger().warn(f"failed to load semantic map, create new one: {e}")

        return {
            "map_frame": "map",
            "version": "0.1",
            "updated_at": time.time(),
            "next_id": 1,
            "objects": []
        }

    def save_semantic_map(self):
        os.makedirs(os.path.dirname(self.semantic_map_path), exist_ok=True)

        self.semantic_map["updated_at"] = time.time()

        tmp_path = self.semantic_map_path + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(self.semantic_map, f, ensure_ascii=False, indent=2)

        os.replace(tmp_path, self.semantic_map_path)

    def observations_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"invalid observations json: {e}")
            return

        observations = payload.get("observations", [])
        if not isinstance(observations, list):
            self.get_logger().warn("observations field is not a list")
            return

        changed = False

        for obs in observations:
            ok = self.handle_one_observation(obs)
            changed = changed or ok

        if changed:
            self.save_semantic_map()
            self.publish_map()

    def handle_one_observation(self, obs: Dict[str, Any]) -> bool:
        label = str(obs.get("label", "unknown"))
        confidence = float(obs.get("confidence", 0.0))

        pos = obs.get("position_map", {})
        try:
            x = float(pos["x"])
            y = float(pos["y"])
            z = float(pos["z"])
        except Exception:
            self.get_logger().warn(f"invalid position_map in observation: {obs}")
            return False

        now = time.time()

        if label in self.dynamic_classes:
            self.get_logger().info(f"skip dynamic class: {label}")
            return False

        matched = self.find_matching_object(label, x, y, z)

        if matched is None:
            new_id = int(self.semantic_map.get("next_id", 1))
            self.semantic_map["next_id"] = new_id + 1

            obj = {
                "id": new_id,
                "label": label,
                "x": x,
                "y": y,
                "z": z,
                "confidence": confidence,
                "observations": 1,
                "status": "candidate",
                "first_seen": now,
                "last_seen": now,
                "merge_distance": self.merge_distance_default
            }

            self.semantic_map["objects"].append(obj)

            self.get_logger().info(
                f"new semantic object: id={new_id}, label={label}, "
                f"x={x:.3f}, y={y:.3f}, z={z:.3f}, conf={confidence:.2f}"
            )
            return True

        self.update_object(matched, x, y, z, confidence, now)

        self.get_logger().info(
            f"updated semantic object: id={matched['id']}, label={label}, "
            f"obs={matched['observations']}, status={matched['status']}, "
            f"x={matched['x']:.3f}, y={matched['y']:.3f}, z={matched['z']:.3f}, "
            f"conf={matched['confidence']:.2f}"
        )
        return True

    def find_matching_object(self, label: str, x: float, y: float, z: float) -> Optional[Dict[str, Any]]:
        best_obj = None
        best_dist = None

        for obj in self.semantic_map.get("objects", []):
            if obj.get("label") != label:
                continue

            dist = self.distance_3d(
                x, y, z,
                float(obj.get("x", 0.0)),
                float(obj.get("y", 0.0)),
                float(obj.get("z", 0.0))
            )

            merge_distance = float(obj.get("merge_distance", self.merge_distance_default))

            if dist <= merge_distance:
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_obj = obj

        return best_obj

    def update_object(self, obj: Dict[str, Any], x: float, y: float, z: float, confidence: float, now: float):
        n = int(obj.get("observations", 1))

        old_x = float(obj.get("x", x))
        old_y = float(obj.get("y", y))
        old_z = float(obj.get("z", z))
        old_conf = float(obj.get("confidence", confidence))

        new_n = n + 1

        obj["x"] = (old_x * n + x) / new_n
        obj["y"] = (old_y * n + y) / new_n
        obj["z"] = (old_z * n + z) / new_n
        obj["confidence"] = (old_conf * n + confidence) / new_n
        obj["observations"] = new_n
        obj["last_seen"] = now

        if new_n >= self.min_observations_confirmed:
            obj["status"] = "confirmed"
        else:
            obj["status"] = "candidate"

    def publish_map(self):
        msg = String()
        msg.data = json.dumps(self.semantic_map, ensure_ascii=False)
        self.map_pub.publish(msg)

    @staticmethod
    def distance_3d(x1, y1, z1, x2, y2, z2) -> float:
        return math.sqrt(
            (x1 - x2) * (x1 - x2)
            + (y1 - y2) * (y1 - y2)
            + (z1 - z2) * (z1 - z2)
        )


def main(args=None):
    rclpy.init(args=args)
    node = SemanticFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_semantic_map()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
