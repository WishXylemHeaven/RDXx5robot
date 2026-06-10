#!/usr/bin/env python3
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion

from tf2_ros import Buffer, TransformListener


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer_node')

        self.declare_parameter('map_topic', '/rtabmap/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('action_name', 'navigate_to_pose')

        self.declare_parameter('free_value_max', 20)
        self.declare_parameter('unknown_value', -1)

        self.declare_parameter('min_frontier_size', 8)
        self.declare_parameter('min_goal_distance', 0.35)
        self.declare_parameter('max_goal_distance', 2.5)
        self.declare_parameter('blacklist_radius', 0.45)

        # Safety parameters for Ackermann chassis:
        # Do not navigate exactly onto frontier boundary.
        # Move selected goal back toward robot and require free-space clearance.
        self.declare_parameter('frontier_backoff', 0.45)
        self.declare_parameter('goal_clearance_radius', 0.25)
        self.declare_parameter('max_heading_error_deg', 70.0)

        self.declare_parameter('wait_after_goal', 3.0)
        self.declare_parameter('goal_timeout_sec', 60.0)
        self.declare_parameter('max_goals', 20)

        self.declare_parameter('dry_run', True)
        self.declare_parameter('once', True)

        self.map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.action_name = self.get_parameter('action_name').value

        self.free_value_max = int(self.get_parameter('free_value_max').value)
        self.unknown_value = int(self.get_parameter('unknown_value').value)

        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.min_goal_distance = float(self.get_parameter('min_goal_distance').value)
        self.max_goal_distance = float(self.get_parameter('max_goal_distance').value)
        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)

        self.frontier_backoff = float(self.get_parameter('frontier_backoff').value)
        self.goal_clearance_radius = float(self.get_parameter('goal_clearance_radius').value)
        self.max_heading_error = math.radians(float(self.get_parameter('max_heading_error_deg').value))

        self.wait_after_goal = float(self.get_parameter('wait_after_goal').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.max_goals = int(self.get_parameter('max_goals').value)

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.once = bool(self.get_parameter('once').value)

        self.latest_map = None
        self.blacklist = []

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)

        self.get_logger().info('frontier_explorer_node started')
        self.get_logger().info(f'map_topic: {self.map_topic}')
        self.get_logger().info(f'action_name: {self.action_name}')
        self.get_logger().info(f'map_frame: {self.map_frame}')
        self.get_logger().info(f'robot_frame: {self.robot_frame}')
        self.get_logger().info(f'dry_run: {self.dry_run}')
        self.get_logger().info(f'once: {self.once}')
        self.get_logger().info(f'max_goals: {self.max_goals}')
        self.get_logger().info(f'frontier_backoff: {self.frontier_backoff}')
        self.get_logger().info(f'goal_clearance_radius: {self.goal_clearance_radius}')
        self.get_logger().info(f'max_heading_error_deg: {math.degrees(self.max_heading_error):.1f}')

    def map_callback(self, msg):
        self.latest_map = msg

    def wait_for_map(self, timeout_sec=10.0):
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_map is not None:
                return True
        return False

    def get_robot_pose(self):
        tf = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.robot_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0)
        )

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation)
        return x, y, yaw

    def idx(self, x, y, width):
        return y * width + x

    def is_free(self, value):
        return 0 <= value <= self.free_value_max

    def is_unknown(self, value):
        return value == self.unknown_value

    def has_unknown_neighbor(self, data, x, y, width, height):
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue
            if self.is_unknown(data[self.idx(nx, ny, width)]):
                return True
        return False

    def detect_frontier_cells(self, grid):
        width = grid.info.width
        height = grid.info.height
        data = grid.data

        frontier = set()

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                v = data[self.idx(x, y, width)]
                if not self.is_free(v):
                    continue

                if self.has_unknown_neighbor(data, x, y, width, height):
                    frontier.add((x, y))

        return frontier

    def cluster_frontiers(self, frontier_cells):
        visited = set()
        clusters = []

        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster = []
            q = deque([cell])
            visited.add(cell)

            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))

                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue

                        nb = (cx + dx, cy + dy)
                        if nb in frontier_cells and nb not in visited:
                            visited.add(nb)
                            q.append(nb)

            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)

        return clusters

    def cell_to_world(self, grid, cell):
        x, y = cell
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        wx = ox + (x + 0.5) * res
        wy = oy + (y + 0.5) * res
        return wx, wy

    def world_to_cell(self, grid, wx, wy):
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        x = int((wx - ox) / res)
        y = int((wy - oy) / res)
        return x, y

    def in_bounds(self, x, y, width, height):
        return 0 <= x < width and 0 <= y < height

    def is_cell_free(self, grid, x, y):
        width = grid.info.width
        height = grid.info.height

        if not self.in_bounds(x, y, width, height):
            return False

        return self.is_free(grid.data[self.idx(x, y, width)])

    def is_goal_clear(self, grid, wx, wy):
        width = grid.info.width
        height = grid.info.height
        res = grid.info.resolution

        cx, cy = self.world_to_cell(grid, wx, wy)
        radius_cells = max(1, int(self.goal_clearance_radius / res))

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue

                x = cx + dx
                y = cy + dy

                if not self.in_bounds(x, y, width, height):
                    return False

                v = grid.data[self.idx(x, y, width)]

                # Goal area must be known free. Unknown or obstacle are both unsafe.
                if not self.is_free(v):
                    return False

        return True

    def angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def make_safe_goal_from_frontier(self, grid, frontier_x, frontier_y, robot_x, robot_y):
        # Move from frontier point back toward robot, so the goal is inside known free space.
        vx = robot_x - frontier_x
        vy = robot_y - frontier_y
        norm = math.hypot(vx, vy)

        if norm < 1e-6:
            return None

        ux = vx / norm
        uy = vy / norm

        # Try several backoff distances. Larger first, then smaller.
        for backoff in (
            self.frontier_backoff,
            self.frontier_backoff * 0.75,
            self.frontier_backoff * 0.5,
            self.frontier_backoff * 1.25,
        ):
            gx = frontier_x + ux * backoff
            gy = frontier_y + uy * backoff

            if self.is_goal_clear(grid, gx, gy):
                return gx, gy, backoff

        return None

    def is_blacklisted(self, wx, wy):
        for bx, by in self.blacklist:
            if math.hypot(wx - bx, wy - by) < self.blacklist_radius:
                return True
        return False

    def choose_goal(self, grid, clusters, robot_x, robot_y, robot_yaw):
        candidates = []

        for cluster in clusters:
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)

            best_cell = min(
                cluster,
                key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2
            )

            frontier_x, frontier_y = self.cell_to_world(grid, best_cell)

            safe_goal = self.make_safe_goal_from_frontier(
                grid,
                frontier_x,
                frontier_y,
                robot_x,
                robot_y
            )

            if safe_goal is None:
                continue

            wx, wy, used_backoff = safe_goal

            dist = math.hypot(wx - robot_x, wy - robot_y)

            if dist < self.min_goal_distance:
                continue
            if dist > self.max_goal_distance:
                continue
            if self.is_blacklisted(wx, wy):
                continue

            target_yaw = math.atan2(wy - robot_y, wx - robot_x)
            heading_error = abs(self.angle_diff(target_yaw, robot_yaw))

            # Ackermann chassis should avoid goals that require near in-place turning.
            if heading_error > self.max_heading_error:
                continue

            # Score:
            # - prefer closer goal
            # - prefer larger frontier
            # - prefer smaller heading error
            score = (
                -dist
                + 0.01 * len(cluster)
                - 0.35 * heading_error
            )

            candidates.append({
                'x': wx,
                'y': wy,
                'frontier_x': frontier_x,
                'frontier_y': frontier_y,
                'dist': dist,
                'size': len(cluster),
                'score': score,
                'heading_error_deg': math.degrees(heading_error),
                'backoff': used_backoff,
            })

        if not candidates:
            return None

        candidates.sort(key=lambda c: c['score'], reverse=True)
        return candidates[0]

    def send_nav_goal(self, goal_x, goal_y, robot_x, robot_y):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available')
            return -1

        yaw = math.atan2(goal_y - robot_y, goal_x - robot_x)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation = quat_from_yaw(yaw)

        self.get_logger().info(
            f'sending goal: x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(yaw):.1f} deg'
        )

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('goal rejected by Nav2')
            return -2

        self.get_logger().info('goal accepted by Nav2')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=self.goal_timeout_sec
        )

        if not result_future.done():
            self.get_logger().warn('goal timeout, canceling goal')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            return -3

        result = result_future.result()
        return int(result.status)

    def run(self):
        if not self.wait_for_map(timeout_sec=15.0):
            self.get_logger().error(f'no map received from {self.map_topic}')
            return

        self.get_logger().info('map received, starting frontier exploration')

        sent_goals = 0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            try:
                robot_x, robot_y, robot_yaw = self.get_robot_pose()
            except Exception as e:
                self.get_logger().warn(f'cannot get robot pose: {e}')
                time.sleep(0.5)
                continue

            grid = self.latest_map
            frontier_cells = self.detect_frontier_cells(grid)
            clusters = self.cluster_frontiers(frontier_cells)

            self.get_logger().info(
                f'frontier_cells={len(frontier_cells)}, clusters={len(clusters)}, '
                f'robot=({robot_x:.3f}, {robot_y:.3f})'
            )

            goal = self.choose_goal(grid, clusters, robot_x, robot_y, robot_yaw)

            if goal is None:
                self.get_logger().info('no valid frontier goal found, exploration finished or map too small')
                return

            self.get_logger().info(
                f'selected safe frontier goal: x={goal["x"]:.3f}, y={goal["y"]:.3f}, '
                f'frontier=({goal["frontier_x"]:.3f}, {goal["frontier_y"]:.3f}), '
                f'dist={goal["dist"]:.3f}, size={goal["size"]}, '
                f'heading_error={goal["heading_error_deg"]:.1f}deg, '
                f'backoff={goal["backoff"]:.2f}, score={goal["score"]:.3f}'
            )

            if self.dry_run:
                self.get_logger().info('dry_run=True, not sending goal to Nav2')
                return

            status = self.send_nav_goal(
                goal['x'],
                goal['y'],
                robot_x,
                robot_y
            )

            self.get_logger().info(f'Nav2 result status: {status}')

            sent_goals += 1

            if status == 4:
                self.get_logger().info(f'goal succeeded, wait {self.wait_after_goal:.1f}s for map update')
                time.sleep(self.wait_after_goal)
            else:
                self.get_logger().warn('goal failed, add to blacklist')
                self.blacklist.append((goal['x'], goal['y']))
                time.sleep(1.0)

            if self.once:
                self.get_logger().info('once=True, stop after one goal')
                return

            if sent_goals >= self.max_goals:
                self.get_logger().info('max_goals reached, stop exploration')
                return


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
