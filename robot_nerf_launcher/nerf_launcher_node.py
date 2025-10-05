"""Nerf Launcher Node.

This node exposes simple topics/services to control a Nerf launcher:
- Pan and tilt angles for aiming (servos)
- Flywheel speed (ESCs)
- Trigger command to fire a dart

It is intentionally hardware-agnostic at the ROS level. You can:
- Connect it to ros2_control controllers for servos/ESCs, or
- Bridge to a microcontroller topic/service, depending on your setup.

Parameters:
- pan_joint_name (str): Name of servo joint for pan (if using ros2_control)
- tilt_joint_name (str): Name of servo joint for tilt (if using ros2_control)
- flywheel_left_name (str): Name of left flywheel interface (velocity)
- flywheel_right_name (str): Name of right flywheel interface (velocity)

Topics:
- ~/cmd/aim (geometry_msgs/msg/Vector3): x=pan(rad), y=tilt(rad), z unused
- ~/cmd/flywheel (std_msgs/msg/Float32): normalized [0..1] or rad/s if bound to controller
- ~/cmd/fire (std_msgs/msg/Bool): True => actuate trigger once

This is a minimal skeleton to be wired up in bringup.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Bool


class NerfLauncherNode(Node):
    """ROS 2 node providing simple command interfaces for a Nerf launcher."""

    def __init__(self) -> None:
        super().__init__("nerf_launcher")

        # Parameters (names for integration; not used directly unless you wire to controllers)
        self.declare_parameter("pan_joint_name", "pan_servo_joint")
        self.declare_parameter("tilt_joint_name", "tilt_servo_joint")
        self.declare_parameter("flywheel_left_name", "flywheel_left")
        self.declare_parameter("flywheel_right_name", "flywheel_right")

        # Internal state
        self._pan: float = 0.0
        self._tilt: float = 0.0
        self._flywheel_speed: float = 0.0

        # Subscriptions (command API)
        self.create_subscription(Vector3, "cmd/aim", self._on_aim, 10)
        self.create_subscription(Float32, "cmd/flywheel", self._on_flywheel, 10)
        self.create_subscription(Bool, "cmd/fire", self._on_fire, 10)

        # Publishers (feedback for UI/debug)
        self._aim_fb_pub = self.create_publisher(Vector3, "status/aim", 10)
        self._flywheel_fb_pub = self.create_publisher(Float32, "status/flywheel", 10)
        self._fire_fb_pub = self.create_publisher(Bool, "status/fired", 10)

        self.get_logger().info("nerf_launcher_node ready. Topics: cmd/aim, cmd/flywheel, cmd/fire")

    # Handlers
    def _on_aim(self, msg: Vector3) -> None:
        # Clamp to reasonable servo ranges (example: pan [-pi, pi], tilt [-0.7, 0.7] rad)
        pan = max(-math.pi, min(math.pi, msg.x))
        tilt = max(-0.7, min(0.7, msg.y))
        self._pan, self._tilt = pan, tilt

        # Here you would send to your controller (e.g. position controller command topic)
        # For now, just publish feedback
        fb = Vector3(x=pan, y=tilt, z=0.0)
        self._aim_fb_pub.publish(fb)

    def _on_flywheel(self, msg: Float32) -> None:
        # Expect normalized [0..1]; clamp for safety
        speed = max(0.0, min(1.0, float(msg.data)))
        self._flywheel_speed = speed
        self._flywheel_fb_pub.publish(Float32(data=speed))

    def _on_fire(self, msg: Bool) -> None:
        if not msg.data:
            return
        # Trigger actuation would go here (servo pulse, etc.)
        # Publish status for debugging
        self._fire_fb_pub.publish(Bool(data=True))
        self.get_logger().info("FIRE command received")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = NerfLauncherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
