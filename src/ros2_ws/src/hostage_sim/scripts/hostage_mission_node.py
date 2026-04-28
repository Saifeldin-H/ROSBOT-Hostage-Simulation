#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener


WorldPoint = Tuple[float, float]
RobotPose = Tuple[float, float, float]


@dataclass(frozen=True)
class HostageConfig:
    hostage_id: str
    pose_topic: str
    cmd_vel_topic: str
    zone: str


@dataclass(frozen=True)
class ScenarioConfig:
    name: str
    hostages: Sequence[HostageConfig]
    safe_zone: Optional[WorldPoint]
    return_after_detection: bool = False
    inaccessible_zone: Optional[str] = None


SCENARIOS: Dict[str, ScenarioConfig] = {
    "scenario_1": ScenarioConfig(
        name="scenario_1",
        hostages=(
            HostageConfig("hostage_1", "/hostage_1/pose", "/hostage_1/cmd_vel", "conference_room"),
        ),
        safe_zone=None,
    ),
    "scenario_2": ScenarioConfig(
        name="scenario_2",
        hostages=(
            HostageConfig("hostage_1", "/hostage_1/pose", "/hostage_1/cmd_vel", "open_office"),
            HostageConfig("hostage_2", "/hostage_2/pose", "/hostage_2/cmd_vel", "kitchen_side"),
        ),
        safe_zone=None,
    ),
    "scenario_3": ScenarioConfig(
        name="scenario_3",
        hostages=(
            HostageConfig("hostage_1", "/hostage_1/pose", "/hostage_1/cmd_vel", "constrained_room"),
        ),
        safe_zone=None,
        inaccessible_zone="constrained_room",
    ),
    "scenario_4": ScenarioConfig(
        name="scenario_4",
        hostages=(
            HostageConfig("hostage_1", "/hostage_1/pose", "/hostage_1/cmd_vel", "kitchen_side"),
        ),
        safe_zone=(0.38, -0.14),
        return_after_detection=True,
    ),
}


def distance(first: WorldPoint, second: WorldPoint) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def yaw_from_pose(pose: Pose) -> float:
    q = pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class HostageMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("hostage_mission")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frames", ["base_link", "base_footprint"])
        self.declare_parameter("rescue_radius", 0.9)
        self.declare_parameter("check_rate", 2.0)
        self.declare_parameter("return_goal_tolerance", 0.8)
        self.declare_parameter("breadcrumb_spacing", 0.75)
        self.declare_parameter("hostage_follow_distance", 0.9)
        self.declare_parameter("camera_image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_frame_horizontal_fov", 1.3962634)
        self.declare_parameter("camera_detection_range", 4.0)
        self.declare_parameter("camera_image_timeout", 1.0)
        self.declare_parameter("camera_detection_pixel_threshold", 8)
        self.declare_parameter("diagnostic_log_period_sec", 2.0)

        scenario_name = os.environ.get("RESCUE_SCENARIO", "scenario_1")
        if scenario_name not in SCENARIOS:
            valid = ", ".join(sorted(SCENARIOS))
            raise RuntimeError(f"Unknown RESCUE_SCENARIO={scenario_name!r}. Valid values: {valid}")

        self.scenario = SCENARIOS[scenario_name]
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.robot_base_frames = list(self.get_parameter("robot_base_frames").value)
        self.rescue_radius = float(self.get_parameter("rescue_radius").value)
        self.return_goal_tolerance = float(self.get_parameter("return_goal_tolerance").value)
        self.breadcrumb_spacing = float(self.get_parameter("breadcrumb_spacing").value)
        self.hostage_follow_distance = float(self.get_parameter("hostage_follow_distance").value)
        self.camera_topic = str(self.get_parameter("camera_image_topic").value)
        self.camera_fov = float(self.get_parameter("camera_frame_horizontal_fov").value)
        self.camera_detection_range = float(self.get_parameter("camera_detection_range").value)
        self.camera_image_timeout = float(self.get_parameter("camera_image_timeout").value)
        self.camera_detection_pixel_threshold = int(
            self.get_parameter("camera_detection_pixel_threshold").value
        )
        self.diagnostic_log_period = float(
            self.get_parameter("diagnostic_log_period_sec").value
        )
        check_rate = float(self.get_parameter("check_rate").value)

        self.hostage_poses: Dict[str, Pose] = {}
        self.detected: Dict[str, bool] = {
            hostage.hostage_id: False for hostage in self.scenario.hostages
        }
        self.acquired: Dict[str, bool] = {
            hostage.hostage_id: False for hostage in self.scenario.hostages
        }
        self.phase = "search"
        self.message = "Searching for hostages with frontier exploration and camera detection."
        self.breadcrumbs: List[WorldPoint] = []
        self.return_goals: List[WorldPoint] = []
        self.return_goal_handle = None
        self.return_goal_pending = False
        self.approach_goal_handle = None
        self.approach_goal_pending = False
        self.approach_target_id: Optional[str] = None
        self.approach_goal: Optional[WorldPoint] = None
        self.last_image: Optional[Image] = None
        self.last_image_time: Optional[rclpy.time.Time] = None
        self.camera_seen: Dict[str, bool] = {
            hostage.hostage_id: False for hostage in self.scenario.hostages
        }
        self.camera_pixels: Dict[str, Optional[Dict[str, int]]] = {
            hostage.hostage_id: None for hostage in self.scenario.hostages
        }
        self.camera_bearings: Dict[str, Optional[float]] = {
            hostage.hostage_id: None for hostage in self.scenario.hostages
        }
        self.camera_color_counts: Dict[str, int] = {
            hostage.hostage_id: 0 for hostage in self.scenario.hostages
        }
        self.last_diagnostic_log_time = -1.0e9
        self.last_exploration_enabled: Optional[bool] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.rescue_pub = self.create_publisher(String, "/rescue", 10)
        self.distance_pub = self.create_publisher(Float32, "/rescue_mission/hostage_distance", 10)
        self.rescued_pub = self.create_publisher(Bool, "/rescue_mission/hostage_rescued", 10)
        self.exploration_enabled_pub = self.create_publisher(
            Bool, "/rescue_mission/exploration_enabled", 10
        )
        self.create_subscription(Image, self.camera_topic, self._image_callback, 5)

        self.cmd_pubs = {}
        for hostage in self.scenario.hostages:
            self.cmd_pubs[hostage.hostage_id] = self.create_publisher(
                Twist, hostage.cmd_vel_topic, 10
            )
            self.create_subscription(
                Pose,
                hostage.pose_topic,
                lambda msg, hostage_id=hostage.hostage_id: self._hostage_pose_callback(
                    hostage_id, msg
                ),
                10,
            )

        self.timer = self.create_timer(1.0 / max(check_rate, 0.5), self._tick)
        self.get_logger().info(
            f"Rescue mission started for {self.scenario.name}; camera detection on {self.camera_topic}."
        )

    def _hostage_pose_callback(self, hostage_id: str, msg: Pose) -> None:
        self.hostage_poses[hostage_id] = msg

    def _image_callback(self, msg: Image) -> None:
        self.last_image = msg
        self.last_image_time = self.get_clock().now()

    def _lookup_robot_pose(self) -> Optional[RobotPose]:
        for base_frame in self.robot_base_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    str(base_frame),
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                t = transform.transform.translation
                q = transform.transform.rotation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                return t.x, t.y, math.atan2(siny_cosp, cosy_cosp)
            except TransformException:
                continue
        return None

    def _tick(self) -> None:
        robot_pose = self._lookup_robot_pose()
        robot_xy = None if robot_pose is None else (robot_pose[0], robot_pose[1])
        self._update_detections(robot_pose)

        if robot_xy is not None and self.phase in ("search", "approach"):
            self._record_breadcrumb(robot_xy)

        if self.phase == "approach":
            self._run_approach_phase()
        elif self.phase == "search":
            self._set_frontier_enabled(True, "searching")

        if self.scenario.return_after_detection and any(self.detected.values()):
            self._run_return_phase(robot_xy)

        self._publish_legacy_topics(robot_xy)
        self._publish_rescue_status(robot_xy)
        self._log_diagnostics(robot_pose)

    def _update_detections(self, robot_pose: Optional[RobotPose]) -> None:
        if robot_pose is None:
            return
        robot_xy = (robot_pose[0], robot_pose[1])

        for hostage in self.scenario.hostages:
            if self.detected[hostage.hostage_id]:
                continue

            pose = self.hostage_poses.get(hostage.hostage_id)
            if pose is None:
                continue

            hostage_xy = (pose.position.x, pose.position.y)
            if self._camera_detects_hostage(hostage.hostage_id, robot_pose, hostage_xy):
                self.acquired[hostage.hostage_id] = True
                if self.phase == "search":
                    self.phase = "approach"
                    self.message = (
                        f"{hostage.hostage_id} visually acquired in {hostage.zone}; "
                        "pausing frontier exploration and navigating to hostage."
                    )
                    self.get_logger().info(self.message)
                    self._log_target_geometry(hostage.hostage_id, robot_pose, hostage_xy)

            if (
                self.acquired[hostage.hostage_id]
                and distance(robot_xy, hostage_xy) <= self.rescue_radius
            ):
                self.detected[hostage.hostage_id] = True
                self.message = f"{hostage.hostage_id} reached and confirmed in {hostage.zone}."
                self.get_logger().info(self.message)

        if all(self.detected.values()) and self.phase == "search":
            if self.scenario.return_after_detection:
                self.phase = "returning"
                self.message = "Hostage located; returning to safe zone."
                self._prepare_return_goals()
            else:
                self.phase = "complete"
                self.message = "All hostage locations confirmed."

        if self.phase == "approach":
            if all(self.detected.values()):
                self._clear_approach_goal()
                if self.scenario.return_after_detection:
                    self.phase = "returning"
                    self.message = "Hostage reached; returning to safe zone."
                    self._prepare_return_goals()
                else:
                    self.phase = "complete"
                    self.message = "All hostage locations confirmed."
            elif not self._active_acquired_hostage():
                self.phase = "search"
                self.message = "No active visual target; resuming frontier exploration."
                self._set_frontier_enabled(True, "no active visual target")

    def _camera_detects_hostage(
        self,
        hostage_id: str,
        robot_pose: RobotPose,
        hostage_xy: WorldPoint,
    ) -> bool:
        image = self.last_image
        if image is None or self.last_image_time is None:
            self.camera_seen[hostage_id] = False
            self.camera_pixels[hostage_id] = None
            self.camera_bearings[hostage_id] = None
            self.camera_color_counts[hostage_id] = 0
            return False

        image_age = (self.get_clock().now() - self.last_image_time).nanoseconds / 1.0e9
        if image_age > self.camera_image_timeout:
            self.camera_seen[hostage_id] = False
            self.camera_pixels[hostage_id] = None
            self.camera_bearings[hostage_id] = None
            self.camera_color_counts[hostage_id] = 0
            return False

        robot_xy = (robot_pose[0], robot_pose[1])
        target_distance = distance(robot_xy, hostage_xy)
        if target_distance > self.camera_detection_range:
            self.camera_seen[hostage_id] = False
            self.camera_pixels[hostage_id] = None
            self.camera_bearings[hostage_id] = None
            self.camera_color_counts[hostage_id] = 0
            return False

        bearing = normalize_angle(
            math.atan2(hostage_xy[1] - robot_pose[1], hostage_xy[0] - robot_pose[0])
            - robot_pose[2]
        )
        self.camera_bearings[hostage_id] = bearing
        if abs(bearing) > self.camera_fov * 0.5:
            self.camera_seen[hostage_id] = False
            self.camera_pixels[hostage_id] = None
            self.camera_color_counts[hostage_id] = 0
            return False

        projection = 0.5 - bearing / self.camera_fov
        visual_pixels, camera_pixel = self._count_hostage_colored_pixels(image, projection)
        visible = visual_pixels >= self.camera_detection_pixel_threshold
        self.camera_seen[hostage_id] = visible
        self.camera_pixels[hostage_id] = camera_pixel if visible else None
        self.camera_color_counts[hostage_id] = visual_pixels
        return visible

    def _count_hostage_colored_pixels(
        self, image: Image, horizontal_projection: float
    ) -> Tuple[int, Optional[Dict[str, int]]]:
        channels = self._image_channels(image.encoding)
        if channels is None or image.width == 0 or image.height == 0:
            return 0, None

        center_x = int(max(0.0, min(1.0, horizontal_projection)) * float(image.width - 1))
        half_width = max(6, image.width // 12)
        x_min = max(0, center_x - half_width)
        x_max = min(image.width, center_x + half_width + 1)
        y_min = image.height // 5
        y_max = min(image.height, (image.height * 9) // 10)
        data = image.data

        count = 0
        x_sum = 0
        y_sum = 0
        for y in range(y_min, y_max):
            row_start = y * image.step
            for x in range(x_min, x_max):
                offset = row_start + x * channels
                if offset + channels > len(data):
                    continue
                r, g, b = self._rgb_at(data, offset, image.encoding)
                if self._is_hostage_color(r, g, b):
                    count += 1
                    x_sum += x
                    y_sum += y
                    if count >= self.camera_detection_pixel_threshold:
                        return count, {"x": x_sum // count, "y": y_sum // count}
        if count == 0:
            return 0, None
        return count, {"x": x_sum // count, "y": y_sum // count}

    @staticmethod
    def _image_channels(encoding: str) -> Optional[int]:
        normalized = encoding.lower()
        if normalized in ("rgb8", "bgr8"):
            return 3
        if normalized in ("rgba8", "bgra8"):
            return 4
        return None

    @staticmethod
    def _rgb_at(data: Sequence[int], offset: int, encoding: str) -> Tuple[int, int, int]:
        normalized = encoding.lower()
        if normalized in ("bgr8", "bgra8"):
            return data[offset + 2], data[offset + 1], data[offset]
        return data[offset], data[offset + 1], data[offset + 2]

    @staticmethod
    def _is_hostage_color(r: int, g: int, b: int) -> bool:
        green_sweater = g > 45 and g > r * 1.35 and g > b * 1.15
        blue_jeans = b > 55 and b > r * 1.35 and b > g * 1.15
        skin = r > 110 and g > 70 and b > 45 and r > g * 1.08 and g > b * 1.08
        return green_sweater or blue_jeans or skin

    def _record_breadcrumb(self, robot_xy: WorldPoint) -> None:
        if not self.breadcrumbs or distance(robot_xy, self.breadcrumbs[-1]) >= self.breadcrumb_spacing:
            self.breadcrumbs.append(robot_xy)

    def _active_acquired_hostage(self) -> Optional[Tuple[str, WorldPoint]]:
        for hostage in self.scenario.hostages:
            hostage_id = hostage.hostage_id
            if self.detected[hostage_id] or not self.acquired[hostage_id]:
                continue

            pose = self.hostage_poses.get(hostage_id)
            if pose is None:
                continue
            return hostage_id, (pose.position.x, pose.position.y)
        return None

    def _run_approach_phase(self) -> None:
        target = self._active_acquired_hostage()
        if target is None:
            return

        hostage_id, hostage_xy = target
        self._set_frontier_enabled(False, f"approaching {hostage_id}")

        if self.approach_goal_handle is not None:
            if self.approach_goal is not None and distance(self.approach_goal, hostage_xy) > 0.5:
                self.get_logger().info(
                    f"Updating approach goal for moving target {hostage_id}."
                )
                self.approach_goal_handle.cancel_goal_async()
                self._clear_approach_goal()
            return

        if self.approach_goal_pending:
            return

        self._send_approach_goal(hostage_id, hostage_xy)

    def _send_approach_goal(self, hostage_id: str, point: WorldPoint) -> None:
        if not self.nav_client.server_is_ready():
            if not self.nav_client.wait_for_server(timeout_sec=0.0):
                self.get_logger().info("Waiting for Nav2 before approaching hostage.")
                return

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.w = 1.0
        goal.pose = pose

        self.approach_target_id = hostage_id
        self.approach_goal = point
        self.approach_goal_pending = True
        self.message = (
            f"Following visual target {hostage_id} at x={point[0]:.2f}, y={point[1]:.2f}."
        )
        self.get_logger().info(self.message)
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_approach_goal_response)

    def _on_approach_goal_response(self, future) -> None:
        self.approach_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - runtime transport failure
            self.get_logger().warn(f"Failed to send approach goal: {exc}")
            self._clear_approach_goal()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected hostage approach goal.")
            self._clear_approach_goal()
            return

        self.get_logger().info(
            f"Nav2 accepted approach goal for {self.approach_target_id} at "
            f"x={self.approach_goal[0]:.2f}, y={self.approach_goal[1]:.2f}."
        )
        self.approach_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_approach_goal_result)

    def _on_approach_goal_result(self, future) -> None:
        target_id = self.approach_target_id
        self.approach_goal_handle = None

        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - runtime transport failure
            self.get_logger().warn(f"Failed to receive approach result: {exc}")
            self._clear_approach_goal()
            return

        self.approach_goal = None
        self.approach_target_id = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached approach goal for {target_id}.")
            return

        self.get_logger().warn(f"Hostage approach goal failed with status {result.status}.")

    def _clear_approach_goal(self) -> None:
        self.approach_goal_handle = None
        self.approach_goal_pending = False
        self.approach_target_id = None
        self.approach_goal = None

    def _prepare_return_goals(self) -> None:
        safe_zone = self.scenario.safe_zone
        if safe_zone is None:
            return

        self._clear_approach_goal()
        reversed_trail = list(reversed(self.breadcrumbs[:-1]))
        sampled = reversed_trail[::3]
        self.return_goals = sampled + [safe_zone]
        self._set_frontier_enabled(False, "returning to safe zone")

    def _run_return_phase(self, robot_xy: Optional[WorldPoint]) -> None:
        self._set_frontier_enabled(False, "returning to safe zone")
        if robot_xy is None:
            return

        self._command_detected_hostages_to_follow(robot_xy)

        safe_zone = self.scenario.safe_zone
        if safe_zone is not None and distance(robot_xy, safe_zone) <= self.return_goal_tolerance:
            self.phase = "complete"
            self.message = "Robot and hostage returned to the safe zone."
            self._stop_all_hostages()
            return

        if (
            self.phase == "complete"
            or self.return_goal_handle is not None
            or self.return_goal_pending
        ):
            return

        while self.return_goals and distance(robot_xy, self.return_goals[0]) <= self.return_goal_tolerance:
            self.return_goals.pop(0)

        if not self.return_goals and safe_zone is not None:
            self.return_goals.append(safe_zone)

        if self.return_goals:
            self._send_return_goal(self.return_goals[0])

    def _command_detected_hostages_to_follow(self, robot_xy: WorldPoint) -> None:
        for hostage in self.scenario.hostages:
            if not self.detected[hostage.hostage_id]:
                continue

            pose = self.hostage_poses.get(hostage.hostage_id)
            pub = self.cmd_pubs.get(hostage.hostage_id)
            if pose is None or pub is None:
                continue

            dx = robot_xy[0] - pose.position.x
            dy = robot_xy[1] - pose.position.y
            separation = math.hypot(dx, dy)
            cmd = Twist()
            if separation > self.hostage_follow_distance:
                target_heading = math.atan2(dy, dx)
                heading_error = normalize_angle(target_heading - yaw_from_pose(pose))
                cmd.angular.z = max(-0.9, min(0.9, 1.8 * heading_error))
                if abs(heading_error) < 0.45:
                    cmd.linear.x = min(0.45, 0.25 * separation)
            pub.publish(cmd)

    def _send_return_goal(self, point: WorldPoint) -> None:
        if not self.nav_client.server_is_ready():
            if not self.nav_client.wait_for_server(timeout_sec=0.0):
                self.get_logger().info("Waiting for Nav2 before returning to safe zone.")
                return

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.w = 1.0
        goal.pose = pose

        self.get_logger().info(f"Returning via x={point[0]:.2f}, y={point[1]:.2f}.")
        self.return_goal_pending = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_return_goal_response)

    def _on_return_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - runtime transport failure
            self.get_logger().warn(f"Failed to send return goal: {exc}")
            self.return_goal_pending = False
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected return goal.")
            self.return_goal_pending = False
            return

        self.return_goal_pending = False
        self.return_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_return_goal_result)

    def _on_return_goal_result(self, future) -> None:
        self.return_goal_handle = None
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - runtime transport failure
            self.get_logger().warn(f"Failed to receive return result: {exc}")
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if self.return_goals:
                self.return_goals.pop(0)
            return

        self.get_logger().warn(f"Return goal failed with status {result.status}.")
        if self.return_goals:
            self.return_goals.pop(0)

    def _publish_legacy_topics(self, robot_xy: Optional[WorldPoint]) -> None:
        min_distance = None
        if robot_xy is not None:
            for pose in self.hostage_poses.values():
                hostage_distance = distance(robot_xy, (pose.position.x, pose.position.y))
                min_distance = (
                    hostage_distance
                    if min_distance is None
                    else min(min_distance, hostage_distance)
                )

        if min_distance is not None:
            self.distance_pub.publish(Float32(data=float(min_distance)))
        self.rescued_pub.publish(Bool(data=all(self.detected.values())))

    def _publish_rescue_status(self, robot_xy: Optional[WorldPoint]) -> None:
        hostages = []
        for hostage in self.scenario.hostages:
            pose = self.hostage_poses.get(hostage.hostage_id)
            hostages.append(
                {
                    "id": hostage.hostage_id,
                    "zone": hostage.zone,
                    "acquired": self.acquired[hostage.hostage_id],
                    "detected": self.detected[hostage.hostage_id],
                    "camera_visible": self.camera_seen[hostage.hostage_id],
                    "camera_pixel": self.camera_pixels[hostage.hostage_id],
                    "pose": None
                    if pose is None
                    else {
                        "x": round(pose.position.x, 3),
                        "y": round(pose.position.y, 3),
                        "z": round(pose.position.z, 3),
                    },
                }
            )

        cleared_zones = [
            hostage.zone
            for hostage in self.scenario.hostages
            if self.detected[hostage.hostage_id]
        ]
        if self.scenario.inaccessible_zone and self.phase == "search":
            self.message = "Mapping constrained access; frontier failures indicate inaccessible regions."

        payload = {
            "scenario": self.scenario.name,
            "phase": self.phase,
            "hostages": hostages,
            "cleared_zones": cleared_zones,
            "safe_zone": None
            if self.scenario.safe_zone is None
            else {"x": self.scenario.safe_zone[0], "y": self.scenario.safe_zone[1]},
            "robot_pose": None
            if robot_xy is None
            else {"x": round(robot_xy[0], 3), "y": round(robot_xy[1], 3)},
            "message": self.message,
            "detection_method": "camera_color_image",
            "active_target": self.approach_target_id,
        }
        self.rescue_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _set_frontier_enabled(self, enabled: bool, reason: str) -> None:
        if self.last_exploration_enabled != enabled:
            state = "resumed" if enabled else "paused"
            self.get_logger().info(f"Frontier exploration {state}: {reason}.")
            self.last_exploration_enabled = enabled
        self.exploration_enabled_pub.publish(Bool(data=enabled))

    def _log_target_geometry(
        self,
        hostage_id: str,
        robot_pose: RobotPose,
        hostage_xy: WorldPoint,
    ) -> None:
        robot_xy = (robot_pose[0], robot_pose[1])
        bearing = self.camera_bearings.get(hostage_id)
        bearing_text = "n/a" if bearing is None else f"{bearing:.2f}"
        pixel = self.camera_pixels.get(hostage_id)
        self.get_logger().info(
            f"Target geometry {hostage_id}: robot=({robot_xy[0]:.2f},{robot_xy[1]:.2f},"
            f" yaw={robot_pose[2]:.2f}), hostage=({hostage_xy[0]:.2f},{hostage_xy[1]:.2f}), "
            f"distance={distance(robot_xy, hostage_xy):.2f}, "
            f"bearing={bearing_text}, "
            f"pixel={pixel}, color_count={self.camera_color_counts.get(hostage_id, 0)}."
        )

    def _log_diagnostics(self, robot_pose: Optional[RobotPose]) -> None:
        now = self.get_clock().now().nanoseconds / 1.0e9
        if now - self.last_diagnostic_log_time < self.diagnostic_log_period:
            return
        self.last_diagnostic_log_time = now

        if self.last_image_time is None:
            image_age = "none"
        else:
            image_age = f"{(self.get_clock().now() - self.last_image_time).nanoseconds / 1.0e9:.2f}s"

        robot_text = "unknown"
        robot_xy = None
        if robot_pose is not None:
            robot_xy = (robot_pose[0], robot_pose[1])
            robot_text = f"({robot_pose[0]:.2f},{robot_pose[1]:.2f}, yaw={robot_pose[2]:.2f})"

        parts = []
        for hostage in self.scenario.hostages:
            pose = self.hostage_poses.get(hostage.hostage_id)
            if pose is None:
                parts.append(f"{hostage.hostage_id}: pose=none")
                continue
            hostage_xy = (pose.position.x, pose.position.y)
            distance_text = "n/a" if robot_xy is None else f"{distance(robot_xy, hostage_xy):.2f}"
            bearing = self.camera_bearings.get(hostage.hostage_id)
            bearing_text = "n/a" if bearing is None else f"{bearing:.2f}"
            parts.append(
                f"{hostage.hostage_id}: pose=({hostage_xy[0]:.2f},{hostage_xy[1]:.2f}), "
                f"dist={distance_text}, bearing={bearing_text}, "
                f"visible={self.camera_seen[hostage.hostage_id]}, "
                f"acquired={self.acquired[hostage.hostage_id]}, "
                f"detected={self.detected[hostage.hostage_id]}, "
                f"pixel={self.camera_pixels[hostage.hostage_id]}, "
                f"colors={self.camera_color_counts[hostage.hostage_id]}"
            )

        goal_text = "none"
        if self.approach_goal is not None:
            goal_text = f"({self.approach_goal[0]:.2f},{self.approach_goal[1]:.2f})"
        self.get_logger().info(
            f"Mission status: phase={self.phase}, robot={robot_text}, "
            f"active_target={self.approach_target_id}, approach_goal={goal_text}, "
            f"approach_pending={self.approach_goal_pending}, "
            f"approach_active={self.approach_goal_handle is not None}, "
            f"image_age={image_age}, frontier_enabled={self.last_exploration_enabled}; "
            + " | ".join(parts)
        )

    def _stop_all_hostages(self) -> None:
        for pub in self.cmd_pubs.values():
            pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = HostageMissionNode()
    try:
        rclpy.spin(node)
    finally:
        node._stop_all_hostages()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
