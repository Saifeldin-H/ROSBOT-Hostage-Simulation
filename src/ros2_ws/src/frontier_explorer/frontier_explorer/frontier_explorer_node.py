from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


GridIndex = Tuple[int, int]
WorldPoint = Tuple[float, float]


@dataclass
class FrontierCluster:
    cells: List[GridIndex]
    centroid: WorldPoint
    distance: float


class FrontierExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("robot_base_frames", ["base_link", "base_footprint"])
        self.declare_parameter("nav_to_pose_action", "navigate_to_pose")
        self.declare_parameter("frontier_connectivity", 8)
        self.declare_parameter("min_frontier_size", 12)
        self.declare_parameter("occupancy_threshold", 50)
        self.declare_parameter("goal_tolerance", 0.5)
        self.declare_parameter("retry_limit", 2)
        self.declare_parameter("replan_period_sec", 2.0)
        self.declare_parameter("frontier_blacklist_radius", 0.8)

        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.base_frames = list(
            self.get_parameter("robot_base_frames")
            .get_parameter_value()
            .string_array_value
        )
        action_name = (
            self.get_parameter("nav_to_pose_action").get_parameter_value().string_value
        )
        self.frontier_connectivity = (
            self.get_parameter("frontier_connectivity").get_parameter_value().integer_value
        )
        self.min_frontier_size = (
            self.get_parameter("min_frontier_size").get_parameter_value().integer_value
        )
        self.occupancy_threshold = (
            self.get_parameter("occupancy_threshold").get_parameter_value().integer_value
        )
        self.goal_tolerance = (
            self.get_parameter("goal_tolerance").get_parameter_value().double_value
        )
        self.retry_limit = (
            self.get_parameter("retry_limit").get_parameter_value().integer_value
        )
        self.replan_period = (
            self.get_parameter("replan_period_sec").get_parameter_value().double_value
        )
        self.blacklist_radius = (
            self.get_parameter("frontier_blacklist_radius")
            .get_parameter_value()
            .double_value
        )

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_ready_logged = False
        self.nav_ready_logged = False
        self.no_frontier_logged = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, action_name)
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, 10
        )
        self.timer = self.create_timer(self.replan_period, self._on_timer)

        self.goal_handle = None
        self.current_goal: Optional[WorldPoint] = None
        self.blacklist: List[WorldPoint] = []
        self.failure_counts: dict[WorldPoint, int] = {}

        self.get_logger().info("Frontier explorer node started.")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.no_frontier_logged = False
        if not self.map_ready_logged:
            self.get_logger().info("Map stream detected, exploration can start.")
            self.map_ready_logged = True

    def _on_timer(self) -> None:
        if self.goal_handle is not None:
            return

        if self.map_msg is None:
            return

        if not self.nav_client.server_is_ready():
            if self.nav_client.wait_for_server(timeout_sec=0.0):
                if not self.nav_ready_logged:
                    self.get_logger().info("Nav2 NavigateToPose action server is ready.")
                    self.nav_ready_logged = True
            elif not self.nav_ready_logged:
                self.get_logger().info("Waiting for Nav2 action server...")
            return

        if not self.nav_ready_logged:
            self.get_logger().info("Nav2 NavigateToPose action server is ready.")
            self.nav_ready_logged = True

        robot_position = self._lookup_robot_position()
        if robot_position is None:
            return

        cluster = self._select_frontier(robot_position)
        if cluster is None:
            if not self.no_frontier_logged:
                self.get_logger().info("No reachable frontiers remain.")
                self.no_frontier_logged = True
            return

        self.no_frontier_logged = False
        self._send_goal(cluster.centroid)

    def _lookup_robot_position(self) -> Optional[WorldPoint]:
        if self.map_msg is None:
            return None

        frame_id = self.map_msg.header.frame_id or "map"
        for base_frame in self.base_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    frame_id, base_frame, rclpy.time.Time()
                )
            except TransformException:
                continue

            translation = transform.transform.translation
            return (translation.x, translation.y)

        self.get_logger().debug("Robot pose not available yet from TF.")
        return None

    def _select_frontier(self, robot_position: WorldPoint) -> Optional[FrontierCluster]:
        assert self.map_msg is not None
        frontier_cells = self._find_frontier_cells(self.map_msg)
        if not frontier_cells:
            return None

        clusters = self._cluster_frontiers(frontier_cells, self.map_msg, robot_position)
        valid_clusters = [
            cluster
            for cluster in clusters
            if len(cluster.cells) >= self.min_frontier_size
            and cluster.distance > self.goal_tolerance
            and not self._is_blacklisted(cluster.centroid)
        ]
        if not valid_clusters:
            return None

        valid_clusters.sort(key=lambda cluster: (cluster.distance, -len(cluster.cells)))
        return valid_clusters[0]

    def _find_frontier_cells(self, map_msg: OccupancyGrid) -> set[GridIndex]:
        width = map_msg.info.width
        height = map_msg.info.height
        data = map_msg.data
        frontiers: set[GridIndex] = set()

        for y in range(1, height - 1):
            row_offset = y * width
            for x in range(1, width - 1):
                value = data[row_offset + x]
                if value < 0 or value > self.occupancy_threshold:
                    continue
                if self._has_unknown_neighbor(x, y, width, height, data):
                    frontiers.add((x, y))

        return frontiers

    def _has_unknown_neighbor(
        self,
        x: int,
        y: int,
        width: int,
        height: int,
        data: Sequence[int],
    ) -> bool:
        for nx, ny in self._neighbors(x, y, width, height, include_diagonal=True):
            if data[ny * width + nx] == -1:
                return True
        return False

    def _cluster_frontiers(
        self,
        frontier_cells: set[GridIndex],
        map_msg: OccupancyGrid,
        robot_position: WorldPoint,
    ) -> List[FrontierCluster]:
        visited: set[GridIndex] = set()
        clusters: List[FrontierCluster] = []
        width = map_msg.info.width
        height = map_msg.info.height

        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster_cells: List[GridIndex] = []
            queue: deque[GridIndex] = deque([cell])
            visited.add(cell)

            while queue:
                current = queue.popleft()
                cluster_cells.append(current)
                for neighbor in self._neighbors(
                    current[0],
                    current[1],
                    width,
                    height,
                    include_diagonal=self.frontier_connectivity == 8,
                ):
                    if neighbor in frontier_cells and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)

            centroid = self._centroid(cluster_cells, map_msg)
            distance = self._distance(robot_position, centroid)
            clusters.append(
                FrontierCluster(cells=cluster_cells, centroid=centroid, distance=distance)
            )

        return clusters

    def _neighbors(
        self,
        x: int,
        y: int,
        width: int,
        height: int,
        *,
        include_diagonal: bool,
    ) -> Iterable[GridIndex]:
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if include_diagonal:
            directions.extend([(1, 1), (1, -1), (-1, 1), (-1, -1)])

        for dx, dy in directions:
            nx = x + dx
            ny = y + dy
            if 0 <= nx < width and 0 <= ny < height:
                yield (nx, ny)

    def _centroid(
        self, cluster_cells: Sequence[GridIndex], map_msg: OccupancyGrid
    ) -> WorldPoint:
        x_sum = 0.0
        y_sum = 0.0
        for cell in cluster_cells:
            wx, wy = self._grid_to_world(cell, map_msg)
            x_sum += wx
            y_sum += wy

        count = float(len(cluster_cells))
        return (x_sum / count, y_sum / count)

    def _grid_to_world(self, cell: GridIndex, map_msg: OccupancyGrid) -> WorldPoint:
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position
        x = origin.x + (cell[0] + 0.5) * resolution
        y = origin.y + (cell[1] + 0.5) * resolution
        return (x, y)

    def _send_goal(self, point: WorldPoint) -> None:
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_msg.header.frame_id if self.map_msg else "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        goal.pose = pose

        self.current_goal = point
        self.get_logger().info(
            f"Sending exploration goal to x={point[0]:.2f}, y={point[1]:.2f}"
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - transport/runtime failure
            self.get_logger().warn(f"Failed to send goal: {exc}")
            self._register_failed_goal()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the exploration goal.")
            self._register_failed_goal()
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self.goal_handle = None

        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - transport/runtime failure
            self.get_logger().warn(f"Failed to receive goal result: {exc}")
            self._register_failed_goal()
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if self.current_goal is not None:
                self.get_logger().info(
                    f"Reached exploration goal near x={self.current_goal[0]:.2f}, "
                    f"y={self.current_goal[1]:.2f}"
                )
            self.current_goal = None
            return

        self.get_logger().warn(f"Goal failed with status {result.status}.")
        self._register_failed_goal()

    def _register_failed_goal(self) -> None:
        if self.current_goal is None:
            self.goal_handle = None
            return

        retry_key = self._retry_key(self.current_goal)
        failures = self.failure_counts.get(retry_key, 0) + 1
        self.failure_counts[retry_key] = failures
        if failures >= self.retry_limit:
            self.blacklist.append(self.current_goal)
            self.get_logger().warn(
                f"Blacklisting frontier near x={self.current_goal[0]:.2f}, "
                f"y={self.current_goal[1]:.2f}"
            )
        self.current_goal = None
        self.goal_handle = None

    def _retry_key(self, point: WorldPoint) -> WorldPoint:
        return (
            round(point[0] / self.blacklist_radius) * self.blacklist_radius,
            round(point[1] / self.blacklist_radius) * self.blacklist_radius,
        )

    def _is_blacklisted(self, point: WorldPoint) -> bool:
        return any(
            self._distance(point, blocked) <= self.blacklist_radius
            for blocked in self.blacklist
        )

    @staticmethod
    def _distance(first: WorldPoint, second: WorldPoint) -> float:
        return math.hypot(first[0] - second[0], first[1] - second[1])


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down frontier explorer.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
