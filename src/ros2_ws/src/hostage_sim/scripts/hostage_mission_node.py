#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformException, TransformListener


class HostageMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("hostage_mission")
        self.declare_parameter("hostage_pose_topic", "/hostage/pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frames", ["base_link", "base_footprint"])
        self.declare_parameter("rescue_radius", 0.8)
        self.declare_parameter("check_rate", 2.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.robot_base_frames = list(self.get_parameter("robot_base_frames").value)
        self.rescue_radius = float(self.get_parameter("rescue_radius").value)
        check_rate = float(self.get_parameter("check_rate").value)

        self.hostage_pose: Optional[Pose] = None
        self.rescued = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_sub = self.create_subscription(
            Pose,
            str(self.get_parameter("hostage_pose_topic").value),
            self._hostage_pose_callback,
            10,
        )
        self.distance_pub = self.create_publisher(Float32, "/rescue_mission/hostage_distance", 10)
        self.rescued_pub = self.create_publisher(Bool, "/rescue_mission/hostage_rescued", 10)
        self.timer = self.create_timer(1.0 / max(check_rate, 0.5), self._tick)

    def _hostage_pose_callback(self, msg: Pose) -> None:
        self.hostage_pose = msg

    def _lookup_robot_xy(self) -> Optional[tuple[float, float]]:
        for base_frame in self.robot_base_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    str(base_frame),
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                t = transform.transform.translation
                return t.x, t.y
            except TransformException:
                continue
        return None

    def _tick(self) -> None:
        if self.hostage_pose is None:
            self.rescued_pub.publish(Bool(data=self.rescued))
            return

        robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            self.rescued_pub.publish(Bool(data=self.rescued))
            return

        distance = math.hypot(
            self.hostage_pose.position.x - robot_xy[0],
            self.hostage_pose.position.y - robot_xy[1],
        )
        self.distance_pub.publish(Float32(data=float(distance)))

        if distance <= self.rescue_radius and not self.rescued:
            self.rescued = True
            self.get_logger().info(
                f"Hostage rescued at distance {distance:.2f} m."
            )
        self.rescued_pub.publish(Bool(data=self.rescued))


def main() -> None:
    rclpy.init()
    node = HostageMissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
