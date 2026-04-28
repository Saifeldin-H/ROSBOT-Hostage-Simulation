#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node


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


class HostagePatrolNode(Node):
    def __init__(self) -> None:
        super().__init__("hostage_patrol")
        self.declare_parameter("cmd_vel_topic", "/hostage/cmd_vel")
        self.declare_parameter("pose_topic", "/hostage/pose")
        self.declare_parameter("waypoints", [2.0, -4.0, 4.2, -4.0, 4.2, -5.2, 2.0, -5.2])
        self.declare_parameter("linear_speed", 0.35)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("heading_tolerance", 0.25)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("loop", True)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        control_rate = float(self.get_parameter("control_rate").value)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.heading_tolerance = float(self.get_parameter("heading_tolerance").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.waypoints = self._parse_waypoints(self.get_parameter("waypoints").value)
        self.current_waypoint = 0
        self.pose: Optional[Pose] = None

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.pose_sub = self.create_subscription(Pose, pose_topic, self._pose_callback, 10)
        self.timer = self.create_timer(1.0 / max(control_rate, 1.0), self._tick)

    def _parse_waypoints(self, values: List[float]) -> List[Tuple[float, float]]:
        if len(values) < 2 or len(values) % 2 != 0:
            raise ValueError("waypoints must be a flat list of x, y pairs")
        return [(float(values[i]), float(values[i + 1])) for i in range(0, len(values), 2)]

    def _pose_callback(self, msg: Pose) -> None:
        self.pose = msg

    def _tick(self) -> None:
        cmd = Twist()
        if self.pose is None or not self.waypoints:
            self.cmd_pub.publish(cmd)
            return

        if self.current_waypoint >= len(self.waypoints):
            self.cmd_pub.publish(cmd)
            return

        target_x, target_y = self.waypoints[self.current_waypoint]
        x = self.pose.position.x
        y = self.pose.position.y
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            self.current_waypoint += 1
            if self.current_waypoint >= len(self.waypoints) and self.loop:
                self.current_waypoint = 0
            self.cmd_pub.publish(cmd)
            return

        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - yaw_from_pose(self.pose))
        cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 2.0 * heading_error))
        if abs(heading_error) <= self.heading_tolerance:
            cmd.linear.x = self.linear_speed
        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = HostagePatrolNode()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
