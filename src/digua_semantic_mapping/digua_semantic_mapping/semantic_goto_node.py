#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


def normalize_label(label):
    return str(label).strip().lower()


class SemanticGotoNode(Node):
    def __init__(
        self,
        target_label=None,
        dry_run=False,
        approach_distance=None,
        list_only=False,
        target_id=None,
        select_mode="nearest",
        allow_candidate=False,
    ):
        super().__init__("semantic_goto_node")

        self.target_label = normalize_label(target_label) if target_label else None
        self.dry_run = dry_run
        self.list_only = list_only
        self.target_id = target_id
        self.select_mode = select_mode
        self.cli_allow_candidate = allow_candidate

        self.declare_parameter(
            "semantic_map_path",
            "/home/sunrise/digua_ws/digua_maps/semantic/semantic_map.json"
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("approach_distance", 0.7)
        self.declare_parameter("allow_candidate", False)

        self.semantic_map_path = self.get_parameter("semantic_map_path").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.approach_distance = float(self.get_parameter("approach_distance").value)
        self.allow_candidate = bool(self.get_parameter("allow_candidate").value) or self.cli_allow_candidate

        if approach_distance is not None:
            self.approach_distance = float(approach_distance)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.get_logger().info("semantic_goto_node started")
        self.get_logger().info(f"target_label: {self.target_label}")
        self.get_logger().info(f"target_id: {self.target_id}")
        self.get_logger().info(f"select_mode: {self.select_mode}")
        self.get_logger().info(f"semantic_map_path: {self.semantic_map_path}")
        self.get_logger().info(f"map_frame: {self.map_frame}")
        self.get_logger().info(f"base_frame: {self.base_frame}")
        self.get_logger().info(f"approach_distance: {self.approach_distance}")
        self.get_logger().info(f"allow_candidate: {self.allow_candidate}")
        self.get_logger().info(f"dry_run: {self.dry_run}")
        self.get_logger().info(f"list_only: {self.list_only}")

    def load_semantic_map(self):
        path = Path(self.semantic_map_path)
        if not path.exists():
            raise FileNotFoundError(f"semantic map not found: {path}")

        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        return data.get("objects", [])

    def try_get_robot_xy(self, timeout_sec=2.0):
        deadline = time.time() + timeout_sec
        last_error = None

        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                if self.tf_buffer.can_transform(
                    self.map_frame,
                    self.base_frame,
                    Time(),
                    timeout=Duration(seconds=0.1)
                ):
                    tf = self.tf_buffer.lookup_transform(
                        self.map_frame,
                        self.base_frame,
                        Time()
                    )
                    return float(tf.transform.translation.x), float(tf.transform.translation.y)
            except Exception as e:
                last_error = e

        return None, None

    def get_robot_xy(self):
        self.get_logger().info(
            f"waiting for TF {self.map_frame} -> {self.base_frame}"
        )

        robot_x, robot_y = self.try_get_robot_xy(timeout_sec=10.0)
        if robot_x is None:
            raise RuntimeError(
                f"failed to get TF {self.map_frame} -> {self.base_frame} within 10s"
            )

        self.get_logger().info(f"robot pose in map: x={robot_x:.3f}, y={robot_y:.3f}")
        return robot_x, robot_y

    def object_distance(self, obj, robot_x, robot_y):
        if robot_x is None or robot_y is None:
            return None
        try:
            return math.hypot(float(obj["x"]) - robot_x, float(obj["y"]) - robot_y)
        except Exception:
            return None

    def list_objects(self, objects):
        robot_x, robot_y = self.try_get_robot_xy(timeout_sec=2.0)

        if robot_x is not None:
            self.get_logger().info(f"robot pose in map: x={robot_x:.3f}, y={robot_y:.3f}")
        else:
            self.get_logger().warn("TF unavailable, listing objects without distance")

        valid = []
        for obj in objects:
            label = normalize_label(obj.get("label", ""))
            if not label:
                continue

            dist = self.object_distance(obj, robot_x, robot_y)

            valid.append({
                "id": obj.get("id"),
                "label": label,
                "status": obj.get("status", ""),
                "x": float(obj.get("x", 0.0)),
                "y": float(obj.get("y", 0.0)),
                "z": float(obj.get("z", 0.0)),
                "confidence": float(obj.get("confidence", 0.0)),
                "observations": int(obj.get("observations", 0)),
                "dist": dist,
            })

        valid.sort(key=lambda item: (item["label"], item["id"] if item["id"] is not None else 999999))

        print("\n==== semantic objects ====")
        if not valid:
            print("No semantic objects found.")
            return

        for item in valid:
            dist_text = "unknown" if item["dist"] is None else f"{item['dist']:.3f}m"
            print(
                f"id={item['id']}  "
                f"label={item['label']}  "
                f"status={item['status']}  "
                f"obs={item['observations']}  "
                f"conf={item['confidence']:.3f}  "
                f"xyz=({item['x']:.3f}, {item['y']:.3f}, {item['z']:.3f})  "
                f"dist={dist_text}"
            )

        labels = sorted(set(item["label"] for item in valid))
        print("\nlabels:", ", ".join(labels))
        print("==========================\n")

    def select_object(self, objects, robot_x, robot_y):
        candidates = []

        for obj in objects:
            label = normalize_label(obj.get("label", ""))
            status = str(obj.get("status", "")).strip().lower()

            if self.target_id is not None:
                try:
                    if int(obj.get("id")) != int(self.target_id):
                        continue
                except Exception:
                    continue
            else:
                if label != self.target_label:
                    continue

            if status != "confirmed" and not self.allow_candidate:
                continue

            try:
                x = float(obj["x"])
                y = float(obj["y"])
            except Exception:
                continue

            dist = math.hypot(x - robot_x, y - robot_y)
            observations = int(obj.get("observations", 0))
            confidence = float(obj.get("confidence", 0.0))

            candidates.append({
                "obj": obj,
                "dist": dist,
                "observations": observations,
                "confidence": confidence,
            })

        if not candidates:
            available = sorted(set(
                normalize_label(o.get("label", ""))
                for o in objects
                if o.get("label", "")
            ))

            if self.target_id is not None:
                raise RuntimeError(
                    f"no semantic object found for id={self.target_id}. "
                    f"available labels={available}. "
                    f"Use --list to see object ids."
                )

            raise RuntimeError(
                f"no semantic object found for label='{self.target_label}'. "
                f"available labels={available}. "
                f"Use --list to see details."
            )

        if len(candidates) > 1:
            print("\n==== candidate objects ====")
            for item in candidates:
                obj = item["obj"]
                print(
                    f"id={obj.get('id')}  "
                    f"label={obj.get('label')}  "
                    f"status={obj.get('status')}  "
                    f"obs={obj.get('observations')}  "
                    f"conf={float(obj.get('confidence', 0.0)):.3f}  "
                    f"xy=({float(obj['x']):.3f}, {float(obj['y']):.3f})  "
                    f"dist={item['dist']:.3f}m"
                )
            print("===========================\n")

        if self.select_mode == "nearest":
            candidates.sort(key=lambda item: item["dist"])
        elif self.select_mode == "observations":
            candidates.sort(key=lambda item: (-item["observations"], item["dist"]))
        elif self.select_mode == "confidence":
            candidates.sort(key=lambda item: (-item["confidence"], item["dist"]))
        else:
            raise RuntimeError(f"unknown select_mode: {self.select_mode}")

        chosen = candidates[0]["obj"]

        self.get_logger().info(
            "selected object: "
            f"id={chosen.get('id')}, "
            f"label={chosen.get('label')}, "
            f"x={float(chosen['x']):.3f}, "
            f"y={float(chosen['y']):.3f}, "
            f"z={float(chosen.get('z', 0.0)):.3f}, "
            f"status={chosen.get('status')}, "
            f"obs={chosen.get('observations')}, "
            f"conf={float(chosen.get('confidence', 0.0)):.3f}, "
            f"dist_to_robot={candidates[0]['dist']:.3f}, "
            f"select_mode={self.select_mode}"
        )

        return chosen

    def make_goal_pose(self, obj, robot_x, robot_y):
        obj_x = float(obj["x"])
        obj_y = float(obj["y"])

        dx = robot_x - obj_x
        dy = robot_y - obj_y
        d = math.hypot(dx, dy)

        if d < 1e-3:
            ux, uy = -1.0, 0.0
        else:
            ux, uy = dx / d, dy / d

        goal_x = obj_x + ux * self.approach_distance
        goal_y = obj_y + uy * self.approach_distance

        yaw = math.atan2(obj_y - goal_y, obj_x - goal_x)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.get_logger().info(
            f"navigation goal: x={goal_x:.3f}, y={goal_y:.3f}, "
            f"yaw={yaw:.3f} rad, yaw_deg={math.degrees(yaw):.1f}"
        )

        return pose

    def send_nav_goal(self, pose):
        self.get_logger().info("waiting for Nav2 action server: navigate_to_pose")

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                "Nav2 action server navigate_to_pose not available. "
                "Please start Nav2 navigation first."
            )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info("sending navigation goal to Nav2...")
        send_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None:
            raise RuntimeError("failed to send goal: goal_handle is None")

        if not goal_handle.accepted:
            raise RuntimeError("Nav2 rejected the semantic navigation goal")

        self.get_logger().info("goal accepted by Nav2, waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        self.get_logger().info(f"Nav2 result status: {result.status}")

    def run(self):
        objects = self.load_semantic_map()
        self.get_logger().info(f"loaded semantic objects: {len(objects)}")

        if self.list_only:
            self.list_objects(objects)
            return

        if not self.target_label and self.target_id is None:
            raise RuntimeError("please provide a label, an --id, or use --list")

        robot_x, robot_y = self.get_robot_xy()
        obj = self.select_object(objects, robot_x, robot_y)
        goal_pose = self.make_goal_pose(obj, robot_x, robot_y)

        if self.dry_run:
            self.get_logger().info("dry_run=True, not sending goal to Nav2")
            return

        self.send_nav_goal(goal_pose)


def main():
    parser = argparse.ArgumentParser(
        description="Navigate to a semantic object from semantic_map.json"
    )
    parser.add_argument(
        "label",
        nargs="?",
        default=None,
        help="semantic label, for example: footwear, bottle, tv"
    )
    parser.add_argument("--list", action="store_true", help="list semantic objects and exit")
    parser.add_argument("--id", type=int, default=None, help="select object by semantic object id")
    parser.add_argument(
        "--select",
        choices=["nearest", "observations", "confidence"],
        default="nearest",
        help="selection rule when multiple objects match a label"
    )
    parser.add_argument(
        "--allow-candidate",
        action="store_true",
        help="allow navigating to candidate objects, not only confirmed objects"
    )
    parser.add_argument(
        "--distance",
        type=float,
        default=None,
        help="approach distance from object"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="only print selected object and goal, do not send Nav2 goal"
    )

    args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init()
    node = SemanticGotoNode(
        target_label=args.label,
        dry_run=args.dry_run,
        approach_distance=args.distance,
        list_only=args.list,
        target_id=args.id,
        select_mode=args.select,
        allow_candidate=args.allow_candidate,
    )

    try:
        node.run()
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
