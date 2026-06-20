#!/usr/bin/env python3
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


def yaw_to_quat(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('frontier_explorer_node')

        # Basic interfaces
        self.declare_parameter('map_topic', '/rtabmap/map')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')

        # Basic exploration
        self.declare_parameter('dry_run', False)
        self.declare_parameter('once', True)
        self.declare_parameter('max_goals', 1)
        self.declare_parameter('min_frontier_size', 8)
        self.declare_parameter('min_goal_distance', 0.30)
        self.declare_parameter('max_goal_distance', 2.0)
        self.declare_parameter('blacklist_radius', 0.45)
        self.declare_parameter('frontier_backoff', 0.45)
        self.declare_parameter('goal_clearance_radius', 0.22)
        self.declare_parameter('max_heading_error_deg', 70.0)
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('wait_after_goal', 3.0)

        # Ackermann reverse recovery
        self.declare_parameter('enable_reverse_recovery', True)
        self.declare_parameter('reverse_recovery_distance', 0.25)
        self.declare_parameter('max_reverse_recovery_count', 1)
        self.declare_parameter('stop_on_nav_failure', True)

        # Anti-oscillation
        self.declare_parameter('recent_goal_radius', 0.35)
        self.declare_parameter('recent_frontier_radius', 0.45)
        self.declare_parameter('recent_history_size', 20)
        self.declare_parameter('min_progress_to_reset_reverse', 0.55)

        # Advanced strategy: multi-view + global fallback + cluster-level memory
        self.declare_parameter('multiview_angle_step_deg', 30.0)
        self.declare_parameter('multiview_lateral_offset', 0.20)
        self.declare_parameter('global_fallback_enabled', True)
        self.declare_parameter('global_fallback_max_goal_distance', 4.0)
        self.declare_parameter('global_fallback_max_heading_error_deg', 85.0)
        self.declare_parameter('cluster_recent_radius', 0.55)
        self.declare_parameter('cluster_blacklist_radius', 0.75)

        # Ackermann feasibility scoring
        self.declare_parameter('ackermann_heading_weight', 0.60)
        self.declare_parameter('ackermann_final_yaw_weight', 0.20)
        self.declare_parameter('cluster_size_weight', 0.015)
        self.declare_parameter('global_fallback_distance_bonus', 0.20)

        # V3 strategy: staging waypoint fallback
        # If both local and global frontier goals fail, move to a safe known-free
        # staging point first, then continue frontier exploration.
        self.declare_parameter('staging_fallback_enabled', True)
        self.declare_parameter('staging_sample_stride_cells', 4)
        self.declare_parameter('staging_clearance_radius', 0.30)
        self.declare_parameter('staging_min_distance', 0.50)
        self.declare_parameter('staging_max_distance', 2.20)
        self.declare_parameter('staging_max_heading_error_deg', 75.0)
        self.declare_parameter('staging_cluster_distance_weight', 0.55)
        self.declare_parameter('staging_robot_distance_weight', 0.35)
        self.declare_parameter('staging_heading_weight', 0.80)
        self.declare_parameter('staging_cluster_size_weight', 0.020)

        self.map_topic = str(self.get_parameter('map_topic').value)
        self.action_name = str(self.get_parameter('action_name').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.once = bool(self.get_parameter('once').value)
        self.max_goals = int(self.get_parameter('max_goals').value)
        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.min_goal_distance = float(self.get_parameter('min_goal_distance').value)
        self.max_goal_distance = float(self.get_parameter('max_goal_distance').value)
        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self.frontier_backoff = float(self.get_parameter('frontier_backoff').value)
        self.goal_clearance_radius = float(self.get_parameter('goal_clearance_radius').value)
        self.max_heading_error = math.radians(float(self.get_parameter('max_heading_error_deg').value))
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.wait_after_goal = float(self.get_parameter('wait_after_goal').value)

        self.enable_reverse_recovery = bool(self.get_parameter('enable_reverse_recovery').value)
        self.reverse_recovery_distance = float(self.get_parameter('reverse_recovery_distance').value)
        self.max_reverse_recovery_count = int(self.get_parameter('max_reverse_recovery_count').value)
        self.stop_on_nav_failure = bool(self.get_parameter('stop_on_nav_failure').value)

        self.recent_goal_radius = float(self.get_parameter('recent_goal_radius').value)
        self.recent_frontier_radius = float(self.get_parameter('recent_frontier_radius').value)
        self.recent_history_size = int(self.get_parameter('recent_history_size').value)
        self.min_progress_to_reset_reverse = float(self.get_parameter('min_progress_to_reset_reverse').value)

        self.multiview_angle_step = math.radians(float(self.get_parameter('multiview_angle_step_deg').value))
        self.multiview_lateral_offset = float(self.get_parameter('multiview_lateral_offset').value)
        self.global_fallback_enabled = bool(self.get_parameter('global_fallback_enabled').value)
        self.global_fallback_max_goal_distance = float(self.get_parameter('global_fallback_max_goal_distance').value)
        self.global_fallback_max_heading_error = math.radians(
            float(self.get_parameter('global_fallback_max_heading_error_deg').value)
        )
        self.cluster_recent_radius = float(self.get_parameter('cluster_recent_radius').value)
        self.cluster_blacklist_radius = float(self.get_parameter('cluster_blacklist_radius').value)

        self.ackermann_heading_weight = float(self.get_parameter('ackermann_heading_weight').value)
        self.ackermann_final_yaw_weight = float(self.get_parameter('ackermann_final_yaw_weight').value)
        self.cluster_size_weight = float(self.get_parameter('cluster_size_weight').value)
        self.global_fallback_distance_bonus = float(self.get_parameter('global_fallback_distance_bonus').value)

        self.staging_fallback_enabled = bool(self.get_parameter('staging_fallback_enabled').value)
        self.staging_sample_stride_cells = int(self.get_parameter('staging_sample_stride_cells').value)
        self.staging_clearance_radius = float(self.get_parameter('staging_clearance_radius').value)
        self.staging_min_distance = float(self.get_parameter('staging_min_distance').value)
        self.staging_max_distance = float(self.get_parameter('staging_max_distance').value)
        self.staging_max_heading_error = math.radians(
            float(self.get_parameter('staging_max_heading_error_deg').value)
        )
        self.staging_cluster_distance_weight = float(self.get_parameter('staging_cluster_distance_weight').value)
        self.staging_robot_distance_weight = float(self.get_parameter('staging_robot_distance_weight').value)
        self.staging_heading_weight = float(self.get_parameter('staging_heading_weight').value)
        self.staging_cluster_size_weight = float(self.get_parameter('staging_cluster_size_weight').value)

        self.latest_map = None
        self.blacklist = []
        self.cluster_blacklist = []
        self.recent_goals = []
        self.recent_frontiers = []
        self.recent_clusters = []
        self.last_success_goal = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, qos)
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
        self.get_logger().info(f'enable_reverse_recovery: {self.enable_reverse_recovery}')
        self.get_logger().info(f'reverse_recovery_distance: {self.reverse_recovery_distance}')
        self.get_logger().info(f'max_reverse_recovery_count: {self.max_reverse_recovery_count}')
        self.get_logger().info(f'stop_on_nav_failure: {self.stop_on_nav_failure}')
        self.get_logger().info(f'recent_goal_radius: {self.recent_goal_radius}')
        self.get_logger().info(f'recent_frontier_radius: {self.recent_frontier_radius}')
        self.get_logger().info(f'recent_history_size: {self.recent_history_size}')
        self.get_logger().info(f'min_progress_to_reset_reverse: {self.min_progress_to_reset_reverse}')
        self.get_logger().info('advanced strategy: multiview + global_fallback + cluster_memory + ackermann_score')
        self.get_logger().info(f'multiview_angle_step_deg: {math.degrees(self.multiview_angle_step):.1f}')
        self.get_logger().info(f'multiview_lateral_offset: {self.multiview_lateral_offset}')
        self.get_logger().info(f'global_fallback_enabled: {self.global_fallback_enabled}')
        self.get_logger().info(f'global_fallback_max_goal_distance: {self.global_fallback_max_goal_distance}')
        self.get_logger().info(f'global_fallback_max_heading_error_deg: {math.degrees(self.global_fallback_max_heading_error):.1f}')
        self.get_logger().info(f'cluster_recent_radius: {self.cluster_recent_radius}')
        self.get_logger().info(f'cluster_blacklist_radius: {self.cluster_blacklist_radius}')
        self.get_logger().info(f'staging_fallback_enabled: {self.staging_fallback_enabled}')
        self.get_logger().info(f'staging_sample_stride_cells: {self.staging_sample_stride_cells}')
        self.get_logger().info(f'staging_clearance_radius: {self.staging_clearance_radius}')
        self.get_logger().info(f'staging_min_distance: {self.staging_min_distance}')
        self.get_logger().info(f'staging_max_distance: {self.staging_max_distance}')
        self.get_logger().info(f'staging_max_heading_error_deg: {math.degrees(self.staging_max_heading_error):.1f}')

    def map_callback(self, msg):
        self.latest_map = msg

    def wait_for_map(self):
        while rclpy.ok() and self.latest_map is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_map is not None:
            self.get_logger().info('map received, starting frontier exploration')

    def spin_for(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            yaw = quat_to_yaw(trans.transform.rotation)
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(f'cannot get robot pose: {e}')
            return None

    def index(self, grid, cx, cy):
        return cy * grid.info.width + cx

    def in_bounds(self, grid, cx, cy):
        return 0 <= cx < grid.info.width and 0 <= cy < grid.info.height

    def cell_value(self, grid, cx, cy):
        if not self.in_bounds(grid, cx, cy):
            return 100
        return grid.data[self.index(grid, cx, cy)]

    def is_unknown_cell(self, grid, cx, cy):
        return self.cell_value(grid, cx, cy) < 0

    def is_free_cell(self, grid, cx, cy):
        v = self.cell_value(grid, cx, cy)
        return 0 <= v <= 25

    def is_occupied_cell(self, grid, cx, cy):
        v = self.cell_value(grid, cx, cy)
        return v >= 65

    def world_to_cell(self, grid, wx, wy):
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        res = grid.info.resolution
        cx = int(math.floor((wx - ox) / res))
        cy = int(math.floor((wy - oy) / res))
        return cx, cy

    def cell_to_world(self, grid, cx, cy):
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        res = grid.info.resolution
        return ox + (cx + 0.5) * res, oy + (cy + 0.5) * res

    def is_goal_clear(self, grid, wx, wy, radius=None):
        if radius is None:
            radius = self.goal_clearance_radius
        cx, cy = self.world_to_cell(grid, wx, wy)
        if not self.is_free_cell(grid, cx, cy):
            return False
        r_cells = max(1, int(math.ceil(radius / grid.info.resolution)))
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                if math.hypot(dx, dy) * grid.info.resolution > radius:
                    continue
                nx = cx + dx
                ny = cy + dy
                if not self.in_bounds(grid, nx, ny):
                    return False
                if not self.is_free_cell(grid, nx, ny):
                    return False
        return True

    def line_to_frontier_ok(self, grid, gx, gy, fx, fy):
        dist = math.hypot(fx - gx, fy - gy)
        if dist < 1e-6:
            return False
        step = max(grid.info.resolution * 0.75, 0.03)
        n = max(1, int(dist / step))
        for i in range(n + 1):
            t = i / n
            wx = gx + (fx - gx) * t
            wy = gy + (fy - gy) * t
            cx, cy = self.world_to_cell(grid, wx, wy)
            if not self.in_bounds(grid, cx, cy):
                return False
            if self.is_occupied_cell(grid, cx, cy):
                return False
        return True

    def is_recent_point(self, wx, wy, history, radius):
        for hx, hy in history:
            if math.hypot(wx - hx, wy - hy) < radius:
                return True
        return False

    def remember_point(self, history, wx, wy):
        history.append((wx, wy))
        while len(history) > self.recent_history_size:
            del history[0]

    def is_blacklisted(self, wx, wy):
        return self.is_recent_point(wx, wy, self.blacklist, self.blacklist_radius)

    def is_cluster_blacklisted(self, wx, wy):
        return self.is_recent_point(wx, wy, self.cluster_blacklist, self.cluster_blacklist_radius)

    def is_cluster_recent(self, wx, wy):
        return self.is_recent_point(wx, wy, self.recent_clusters, self.cluster_recent_radius)

    def find_frontier_cells(self, grid):
        cells = []
        w = grid.info.width
        h = grid.info.height
        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if not self.is_free_cell(grid, x, y):
                    continue
                has_unknown = False
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        if self.is_unknown_cell(grid, x + dx, y + dy):
                            has_unknown = True
                            break
                    if has_unknown:
                        break
                if has_unknown:
                    cells.append((x, y))
        return cells

    def cluster_frontiers(self, frontier_cells):
        frontier_set = set(frontier_cells)
        clusters = []
        while frontier_set:
            start = frontier_set.pop()
            q = deque([start])
            cluster = [start]
            while q:
                cx, cy = q.popleft()
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nb = (cx + dx, cy + dy)
                        if nb in frontier_set:
                            frontier_set.remove(nb)
                            q.append(nb)
                            cluster.append(nb)
            clusters.append(cluster)
        return clusters

    def cluster_info(self, grid, cluster):
        wxs = []
        wys = []
        for cx, cy in cluster:
            wx, wy = self.cell_to_world(grid, cx, cy)
            wxs.append(wx)
            wys.append(wy)
        mx = sum(wxs) / len(wxs)
        my = sum(wys) / len(wys)
        return {
            'cells': cluster,
            'size': len(cluster),
            'x': mx,
            'y': my,
        }

    def representative_frontier_cells(self, cluster):
        cells = cluster['cells']
        if len(cells) <= 5:
            return cells

        cx_mean = sum(c[0] for c in cells) / len(cells)
        cy_mean = sum(c[1] for c in cells) / len(cells)

        candidates = [
            min(cells, key=lambda c: (c[0] - cx_mean) ** 2 + (c[1] - cy_mean) ** 2),
            min(cells, key=lambda c: c[0]),
            max(cells, key=lambda c: c[0]),
            min(cells, key=lambda c: c[1]),
            max(cells, key=lambda c: c[1]),
            max(cells, key=lambda c: (c[0] - cx_mean) ** 2 + (c[1] - cy_mean) ** 2),
        ]

        uniq = []
        seen = set()
        for c in candidates:
            if c not in seen:
                seen.add(c)
                uniq.append(c)
        return uniq

    def make_reject_stats(self):
        return {
            'too_small': 0,
            'no_safe_backoff_or_clearance': 0,
            'too_near': 0,
            'too_far': 0,
            'blacklisted': 0,
            'cluster_blacklisted': 0,
            'heading_too_large': 0,
            'recent_goal': 0,
            'recent_frontier': 0,
            'recent_cluster': 0,
            'line_blocked': 0,
        }

    def add_stats(self, dst, src):
        for k, v in src.items():
            dst[k] = dst.get(k, 0) + v

    def generate_viewpoints_for_cluster(self, grid, cluster, robot_x, robot_y, robot_yaw,
                                        mode, max_goal_distance, max_heading_error):
        stats = self.make_reject_stats()
        candidates = []

        if cluster['size'] < self.min_frontier_size:
            stats['too_small'] += 1
            return candidates, stats

        if self.is_cluster_blacklisted(cluster['x'], cluster['y']):
            stats['cluster_blacklisted'] += 1
            return candidates, stats

        if self.is_cluster_recent(cluster['x'], cluster['y']):
            stats['recent_cluster'] += 1
            return candidates, stats

        reps = self.representative_frontier_cells(cluster)

        backoffs = sorted(set([
            max(0.25, self.frontier_backoff),
            max(0.25, self.frontier_backoff * 1.25),
            max(0.25, self.frontier_backoff * 1.55),
        ]))
        laterals = [-self.multiview_lateral_offset, 0.0, self.multiview_lateral_offset]

        angle_step = max(math.radians(10.0), self.multiview_angle_step)
        angle_count = max(8, int(math.ceil((2.0 * math.pi) / angle_step)))

        for fc in reps:
            fx, fy = self.cell_to_world(grid, fc[0], fc[1])

            for ai in range(angle_count):
                theta = -math.pi + ai * (2.0 * math.pi / angle_count)
                ux = math.cos(theta)
                uy = math.sin(theta)
                px = -uy
                py = ux

                for backoff in backoffs:
                    for lateral in laterals:
                        gx = fx - backoff * ux + lateral * px
                        gy = fy - backoff * uy + lateral * py

                        if not self.is_goal_clear(grid, gx, gy):
                            stats['no_safe_backoff_or_clearance'] += 1
                            continue

                        if not self.line_to_frontier_ok(grid, gx, gy, fx, fy):
                            stats['line_blocked'] += 1
                            continue

                        dist = math.hypot(gx - robot_x, gy - robot_y)
                        if dist < self.min_goal_distance:
                            stats['too_near'] += 1
                            continue
                        if dist > max_goal_distance:
                            stats['too_far'] += 1
                            continue

                        if self.is_blacklisted(gx, gy):
                            stats['blacklisted'] += 1
                            continue

                        if self.is_recent_point(gx, gy, self.recent_goals, self.recent_goal_radius):
                            stats['recent_goal'] += 1
                            continue

                        if self.is_recent_point(fx, fy, self.recent_frontiers, self.recent_frontier_radius):
                            stats['recent_frontier'] += 1
                            continue

                        travel_yaw = math.atan2(gy - robot_y, gx - robot_x)
                        heading_error = abs(angle_diff(travel_yaw, robot_yaw))
                        if heading_error > max_heading_error:
                            stats['heading_too_large'] += 1
                            continue

                        final_yaw = math.atan2(fy - gy, fx - gx)
                        final_yaw_error = abs(angle_diff(final_yaw, robot_yaw))

                        score = 0.0
                        score += self.cluster_size_weight * cluster['size']
                        score -= 1.0 * dist
                        score -= self.ackermann_heading_weight * heading_error
                        score -= self.ackermann_final_yaw_weight * final_yaw_error
                        score += 0.18 * backoff
                        score -= 0.08 * abs(lateral)

                        if mode == 'global':
                            score += self.global_fallback_distance_bonus * dist
                            score += 0.010 * cluster['size']

                        candidates.append({
                            'x': gx,
                            'y': gy,
                            'yaw': final_yaw,
                            'frontier_x': fx,
                            'frontier_y': fy,
                            'cluster_x': cluster['x'],
                            'cluster_y': cluster['y'],
                            'size': cluster['size'],
                            'dist': dist,
                            'heading_error': heading_error,
                            'final_yaw_error': final_yaw_error,
                            'backoff': backoff,
                            'mode': mode,
                            'score': score,
                        })

        return candidates, stats

    def log_reject_stats(self, prefix, stats):
        self.get_logger().info(
            f'{prefix} reject stats: '
            f'too_small={stats.get("too_small", 0)}, '
            f'no_safe_backoff_or_clearance={stats.get("no_safe_backoff_or_clearance", 0)}, '
            f'too_near={stats.get("too_near", 0)}, '
            f'too_far={stats.get("too_far", 0)}, '
            f'blacklisted={stats.get("blacklisted", 0)}, '
            f'cluster_blacklisted={stats.get("cluster_blacklisted", 0)}, '
            f'heading_too_large={stats.get("heading_too_large", 0)}, '
            f'recent_goal={stats.get("recent_goal", 0)}, '
            f'recent_frontier={stats.get("recent_frontier", 0)}, '
            f'recent_cluster={stats.get("recent_cluster", 0)}, '
            f'line_blocked={stats.get("line_blocked", 0)}'
        )

    def choose_goal(self, grid, clusters, robot_x, robot_y, robot_yaw):
        cluster_infos = [self.cluster_info(grid, c) for c in clusters]
        cluster_infos.sort(
            key=lambda c: (-c['size'], math.hypot(c['x'] - robot_x, c['y'] - robot_y))
        )

        modes = [('local', self.max_goal_distance, self.max_heading_error)]
        if self.global_fallback_enabled:
            modes.append(('global', self.global_fallback_max_goal_distance, self.global_fallback_max_heading_error))

        for mode, max_dist, max_heading in modes:
            all_candidates = []
            total_stats = self.make_reject_stats()

            for cluster in cluster_infos:
                candidates, stats = self.generate_viewpoints_for_cluster(
                    grid, cluster, robot_x, robot_y, robot_yaw,
                    mode, max_dist, max_heading
                )
                all_candidates.extend(candidates)
                self.add_stats(total_stats, stats)

            if all_candidates:
                all_candidates.sort(key=lambda g: g['score'], reverse=True)
                goal = all_candidates[0]
                self.get_logger().info(
                    f'selected {mode} multiview frontier goal: '
                    f'x={goal["x"]:.3f}, y={goal["y"]:.3f}, '
                    f'frontier=({goal["frontier_x"]:.3f}, {goal["frontier_y"]:.3f}), '
                    f'cluster=({goal["cluster_x"]:.3f}, {goal["cluster_y"]:.3f}), '
                    f'dist={goal["dist"]:.3f}, size={goal["size"]}, '
                    f'heading_error={math.degrees(goal["heading_error"]):.1f}deg, '
                    f'final_yaw_error={math.degrees(goal["final_yaw_error"]):.1f}deg, '
                    f'backoff={goal["backoff"]:.2f}, score={goal["score"]:.3f}'
                )
                return goal

            self.log_reject_stats(f'{mode} multiview', total_stats)
            if mode == 'local' and self.global_fallback_enabled:
                self.get_logger().info('local search found no valid goal, trying global frontier fallback')

        return None

    def choose_staging_goal(self, grid, clusters, robot_x, robot_y, robot_yaw):
        if not self.staging_fallback_enabled:
            return None

        cluster_infos = [self.cluster_info(grid, c) for c in clusters]
        cluster_infos.sort(
            key=lambda c: (-c['size'], math.hypot(c['x'] - robot_x, c['y'] - robot_y))
        )

        stride = max(2, int(self.staging_sample_stride_cells))
        candidates = []

        stats = {
            'too_small': 0,
            'cluster_blacklisted': 0,
            'not_clear': 0,
            'too_near': 0,
            'too_far': 0,
            'heading_too_large': 0,
            'recent_goal': 0,
            'line_blocked': 0,
            'generated': 0,
        }

        # Limit to the largest/front-most clusters to avoid excessive CPU.
        for cluster in cluster_infos[:12]:
            if cluster['size'] < self.min_frontier_size:
                stats['too_small'] += 1
                continue

            if self.is_cluster_blacklisted(cluster['x'], cluster['y']):
                stats['cluster_blacklisted'] += 1
                continue

            cluster_recent_penalty = 1.0 if self.is_cluster_recent(cluster['x'], cluster['y']) else 0.0

            for cy in range(1, grid.info.height - 1, stride):
                for cx in range(1, grid.info.width - 1, stride):
                    if not self.is_free_cell(grid, cx, cy):
                        continue

                    gx, gy = self.cell_to_world(grid, cx, cy)
                    stats['generated'] += 1

                    if not self.is_goal_clear(grid, gx, gy, self.staging_clearance_radius):
                        stats['not_clear'] += 1
                        continue

                    dist_robot = math.hypot(gx - robot_x, gy - robot_y)
                    if dist_robot < self.staging_min_distance:
                        stats['too_near'] += 1
                        continue
                    if dist_robot > self.staging_max_distance:
                        stats['too_far'] += 1
                        continue

                    if self.is_recent_point(gx, gy, self.recent_goals, self.recent_goal_radius):
                        stats['recent_goal'] += 1
                        continue

                    travel_yaw = math.atan2(gy - robot_y, gx - robot_x)
                    heading_error = abs(angle_diff(travel_yaw, robot_yaw))
                    if heading_error > self.staging_max_heading_error:
                        stats['heading_too_large'] += 1
                        continue

                    # Staging point should be able to "see" the target frontier cluster
                    # without an occupied cell in between. Unknown cells are allowed.
                    if not self.line_to_frontier_ok(grid, gx, gy, cluster['x'], cluster['y']):
                        stats['line_blocked'] += 1
                        continue

                    final_yaw = math.atan2(cluster['y'] - gy, cluster['x'] - gx)
                    final_yaw_error = abs(angle_diff(final_yaw, robot_yaw))
                    dist_cluster = math.hypot(cluster['x'] - gx, cluster['y'] - gy)

                    # Ackermann-friendly staging score:
                    # - prefer large clusters
                    # - avoid excessive steering from current pose
                    # - avoid very long staging motions
                    # - prefer positions that are close enough to observe the frontier
                    # - penalize recent clusters but do not totally ban them
                    score = 0.0
                    score += self.staging_cluster_size_weight * cluster['size']
                    score -= self.staging_robot_distance_weight * dist_robot
                    score -= self.staging_cluster_distance_weight * dist_cluster
                    score -= self.staging_heading_weight * heading_error
                    score -= 0.20 * final_yaw_error
                    score -= 0.80 * cluster_recent_penalty

                    candidates.append({
                        'x': gx,
                        'y': gy,
                        'yaw': final_yaw,
                        'frontier_x': cluster['x'],
                        'frontier_y': cluster['y'],
                        'cluster_x': cluster['x'],
                        'cluster_y': cluster['y'],
                        'size': cluster['size'],
                        'dist': dist_robot,
                        'heading_error': heading_error,
                        'final_yaw_error': final_yaw_error,
                        'backoff': 0.0,
                        'mode': 'staging',
                        'score': score,
                    })

        if not candidates:
            self.get_logger().info(
                'staging fallback reject stats: '
                f'generated={stats["generated"]}, '
                f'too_small={stats["too_small"]}, '
                f'cluster_blacklisted={stats["cluster_blacklisted"]}, '
                f'not_clear={stats["not_clear"]}, '
                f'too_near={stats["too_near"]}, '
                f'too_far={stats["too_far"]}, '
                f'heading_too_large={stats["heading_too_large"]}, '
                f'recent_goal={stats["recent_goal"]}, '
                f'line_blocked={stats["line_blocked"]}'
            )
            return None

        candidates.sort(key=lambda g: g['score'], reverse=True)
        goal = candidates[0]

        self.get_logger().info(
            f'selected staging waypoint fallback goal: '
            f'x={goal["x"]:.3f}, y={goal["y"]:.3f}, '
            f'target_cluster=({goal["cluster_x"]:.3f}, {goal["cluster_y"]:.3f}), '
            f'dist={goal["dist"]:.3f}, size={goal["size"]}, '
            f'heading_error={math.degrees(goal["heading_error"]):.1f}deg, '
            f'final_yaw_error={math.degrees(goal["final_yaw_error"]):.1f}deg, '
            f'score={goal["score"]:.3f}'
        )
        return goal

    def make_reverse_recovery_goal(self, grid, robot_x, robot_y, robot_yaw):
        gx = robot_x - self.reverse_recovery_distance * math.cos(robot_yaw)
        gy = robot_y - self.reverse_recovery_distance * math.sin(robot_yaw)

        if not self.is_goal_clear(grid, gx, gy):
            self.get_logger().warn(f'reverse recovery goal not clear: x={gx:.3f}, y={gy:.3f}')
            return None

        return gx, gy, robot_yaw

    def send_nav_goal(self, goal_x, goal_y, robot_x, robot_y, goal_yaw=None):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available')
            return 6

        if goal_yaw is None:
            yaw = math.atan2(goal_y - robot_y, goal_x - robot_x)
        else:
            yaw = goal_yaw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(goal_x)
        goal_msg.pose.pose.position.y = float(goal_y)
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f'sending goal: x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(yaw):.1f} deg')

        send_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error('failed to send goal to Nav2')
            return 6

        if not goal_handle.accepted:
            self.get_logger().warn('goal rejected by Nav2')
            return 6

        self.get_logger().info('goal accepted by Nav2')

        result_future = goal_handle.get_result_async()
        start = time.time()

        while rclpy.ok():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)
            if result_future.done():
                result = result_future.result()
                return int(result.status)

            if time.time() - start > self.goal_timeout_sec:
                self.get_logger().warn(f'goal timeout after {self.goal_timeout_sec:.1f}s, canceling goal')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
                return 6

        return 6

    def run(self):
        self.wait_for_map()

        sent_goals = 0
        reverse_recovery_count = 0

        while rclpy.ok():
            if sent_goals >= self.max_goals:
                self.get_logger().info(f'max_goals reached: {self.max_goals}')
                return

            grid = self.latest_map
            pose = self.get_robot_pose()
            if pose is None:
                self.spin_for(1.0)
                continue

            robot_x, robot_y, robot_yaw = pose

            frontier_cells = self.find_frontier_cells(grid)
            clusters = self.cluster_frontiers(frontier_cells)

            self.get_logger().info(
                f'frontier_cells={len(frontier_cells)}, clusters={len(clusters)}, '
                f'robot=({robot_x:.3f}, {robot_y:.3f})'
            )

            goal = self.choose_goal(grid, clusters, robot_x, robot_y, robot_yaw)

            if goal is None:
                self.get_logger().info('no direct frontier goal found, trying staging waypoint fallback')
                goal = self.choose_staging_goal(grid, clusters, robot_x, robot_y, robot_yaw)

            if goal is None:
                self.get_logger().info('no valid frontier or staging goal found')

                if self.enable_reverse_recovery and reverse_recovery_count < self.max_reverse_recovery_count:
                    self.get_logger().info(
                        f'trying reverse recovery {reverse_recovery_count + 1}/{self.max_reverse_recovery_count}'
                    )
                    reverse_goal = self.make_reverse_recovery_goal(grid, robot_x, robot_y, robot_yaw)
                    if reverse_goal is None:
                        self.get_logger().warn('reverse recovery not safe, stop exploration')
                        return

                    rgx, rgy, ryaw = reverse_goal
                    self.get_logger().info(
                        f'reverse recovery goal: x={rgx:.3f}, y={rgy:.3f}, yaw={math.degrees(ryaw):.1f}deg'
                    )

                    if self.dry_run:
                        self.get_logger().info('dry_run=True, not sending reverse recovery goal')
                        return

                    status = self.send_nav_goal(rgx, rgy, robot_x, robot_y, ryaw)
                    self.get_logger().info(f'reverse recovery Nav2 result status: {status}')

                    if status == 4:
                        reverse_recovery_count += 1
                        self.get_logger().info(
                            f'reverse recovery succeeded, wait {self.wait_after_goal:.1f}s for map update'
                        )
                        self.spin_for(self.wait_after_goal)
                        continue

                    self.get_logger().warn('reverse recovery failed, stop exploration')
                    return

                self.get_logger().info('exploration finished or map too small')
                return

            if self.dry_run:
                self.get_logger().info('dry_run=True, not sending goal to Nav2')
                if self.once:
                    return
                self.spin_for(self.wait_after_goal)
                continue

            status = self.send_nav_goal(goal['x'], goal['y'], robot_x, robot_y, goal['yaw'])
            self.get_logger().info(f'Nav2 result status: {status}')

            if status == 4:
                sent_goals += 1

                self.remember_point(self.recent_goals, goal['x'], goal['y'])
                self.remember_point(self.recent_frontiers, goal['frontier_x'], goal['frontier_y'])
                self.remember_point(self.recent_clusters, goal['cluster_x'], goal['cluster_y'])

                if self.last_success_goal is None:
                    progress = 999.0
                else:
                    progress = math.hypot(
                        goal['x'] - self.last_success_goal[0],
                        goal['y'] - self.last_success_goal[1]
                    )

                if progress >= self.min_progress_to_reset_reverse:
                    reverse_recovery_count = 0
                    self.get_logger().info(
                        f'real progress detected: {progress:.3f}m, reset reverse recovery counter'
                    )
                else:
                    self.get_logger().info(
                        f'small progress only: {progress:.3f}m, keep reverse recovery counter={reverse_recovery_count}'
                    )

                self.last_success_goal = (goal['x'], goal['y'])

                self.get_logger().info(f'goal succeeded, wait {self.wait_after_goal:.1f}s for map update')
                self.spin_for(self.wait_after_goal)

                if self.once:
                    return

            else:
                self.get_logger().warn('goal failed, add goal and cluster to blacklist')
                self.blacklist.append((goal['x'], goal['y']))
                self.cluster_blacklist.append((goal['cluster_x'], goal['cluster_y']))

                if self.stop_on_nav_failure:
                    self.get_logger().warn(
                        'stop_on_nav_failure=True, stop exploration. '
                        'Check Nav2 log, costmap, and whether robot starts in lethal space.'
                    )
                    return

                self.spin_for(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
