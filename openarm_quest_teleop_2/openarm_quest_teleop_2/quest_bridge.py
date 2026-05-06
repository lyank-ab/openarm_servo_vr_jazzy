#!/usr/bin/env python3
"""
Quest 3 to OpenArm Bimanual Teleoperation Bridge.

Receives 6-DOF controller poses from Quest 3 via WebSocket,
computes velocity commands, and publishes TwistStamped to
two MoveIt Servo instances (left/right arm).

Quest 3 sends JSON at ~72Hz:
{
  "left":  {"pos": [x,y,z], "rot": [qx,qy,qz,qw], "grip": 0.0-1.0, "trigger": 0.0-1.0},
  "right": {"pos": [x,y,z], "rot": [qx,qy,qz,qw], "grip": 0.0-1.0, "trigger": 0.0-1.0},
  "timestamp": 1234567890.123
}

Architecture:
  Quest 3 (WebXR browser) → WebSocket → this node → TwistStamped → MoveIt Servo → JointTrajectory → robot
"""

import asyncio
import json
import math
import os
import ssl
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from moveit_msgs.srv import ServoCommandType

try:
    import websockets
except ImportError:
    print("ERROR: websockets not installed. Run: pip install websockets")
    raise


def quat_to_matrix(q):
    """Quaternion [x, y, z, w] to 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def rotation_to_angular_velocity(R_prev, R_curr, dt):
    """Compute angular velocity from two rotation matrices."""
    if dt < 1e-6:
        return np.zeros(3)
    # R_delta = R_curr @ R_prev^T
    R_delta = R_curr @ R_prev.T
    # Log map: extract axis-angle
    cos_angle = (np.trace(R_delta) - 1.0) / 2.0
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = math.acos(cos_angle)
    if abs(angle) < 1e-6:
        return np.zeros(3)
    # Axis from skew-symmetric part
    axis = np.array([
        R_delta[2, 1] - R_delta[1, 2],
        R_delta[0, 2] - R_delta[2, 0],
        R_delta[1, 0] - R_delta[0, 1],
    ]) / (2.0 * math.sin(angle))
    return axis * angle / dt


class QuestBridge(Node):
    def __init__(self):
        super().__init__("quest_bridge")

        # Parameters
        self.declare_parameter("ws_port", 9090)
        self.declare_parameter("position_scale", 1.0)
        self.declare_parameter("deadman_button", "grip")  # "grip" or "trigger"
        self.declare_parameter("deadman_threshold", 0.5)
        self.declare_parameter("max_linear_vel", 0.4)    # m/s
        self.declare_parameter("max_angular_vel", 0.8)    # rad/s
        self.declare_parameter("gripper_open", 0.044)
        self.declare_parameter("gripper_closed", 0.0)
        self.declare_parameter("log_controller_inputs", False)

        self.ws_port = self.get_parameter("ws_port").value
        self.pos_scale = self.get_parameter("position_scale").value
        self.deadman_btn = self.get_parameter("deadman_button").value
        self.deadman_thresh = self.get_parameter("deadman_threshold").value
        self.max_lin = self.get_parameter("max_linear_vel").value
        self.max_ang = self.get_parameter("max_angular_vel").value
        self.grip_open = self.get_parameter("gripper_open").value
        self.grip_closed = self.get_parameter("gripper_closed").value
        self.log_controller_inputs = self.get_parameter("log_controller_inputs").value
        self.last_input_log_time = 0.0

        # Publishers — TwistStamped to each servo's cartesian command topic
        # MoveIt Servo in Jazzy listens on ~/delta_twist_cmds by default
        self.left_twist_pub = self.create_publisher(
            TwistStamped, "/left/servo_node/delta_twist_cmds", 10
        )
        self.right_twist_pub = self.create_publisher(
            TwistStamped, "/right/servo_node/delta_twist_cmds", 10
        )

        # Gripper command publishers (position)
        self.left_grip_pub = self.create_publisher(
            Float64, "/left_gripper_position", 10
        )
        self.right_grip_pub = self.create_publisher(
            Float64, "/right_gripper_position", 10
        )

        # State tracking
        self.prev_left_pos: Optional[np.ndarray] = None
        self.prev_left_rot: Optional[np.ndarray] = None
        self.prev_right_pos: Optional[np.ndarray] = None
        self.prev_right_rot: Optional[np.ndarray] = None
        self.prev_time: Optional[float] = None
        self.calibrated = False
        self.left_offset = np.zeros(3)
        self.right_offset = np.zeros(3)

        # Servo enable services
        self.left_servo_start = self.create_client(
            Trigger, "/left/servo_node/start_servo"
        )
        self.right_servo_start = self.create_client(
            Trigger, "/right/servo_node/start_servo"
        )
        # Switch command type to TWIST (1) — required in Jazzy
        self.left_cmd_type = self.create_client(
            ServoCommandType, "/left/servo_node/switch_command_type"
        )
        self.right_cmd_type = self.create_client(
            ServoCommandType, "/right/servo_node/switch_command_type"
        )

        self.get_logger().info(f"Quest Bridge starting on ws://0.0.0.0:{self.ws_port}")
        self.get_logger().info(f"Deadman: {self.deadman_btn} (threshold: {self.deadman_thresh})")
        self.get_logger().info("Waiting for Quest 3 connection...")

    def start_servos(self):
        """Start servo nodes and set command type to TWIST."""
        for start_client, type_client, name in [
            (self.left_servo_start, self.left_cmd_type, "left"),
            (self.right_servo_start, self.right_cmd_type, "right"),
        ]:
            if start_client.wait_for_service(timeout_sec=5.0):
                start_client.call_async(Trigger.Request())
                self.get_logger().info(f"Started {name} servo")
            else:
                self.get_logger().warn(f"{name} servo_node not found — is servo_bimanual.launch.py running?")
                continue

            if type_client.wait_for_service(timeout_sec=3.0):
                req = ServoCommandType.Request()
                req.command_type = 1  # TWIST
                type_client.call_async(req)
                self.get_logger().info(f"Set {name} servo command type to TWIST")

    def process_quest_data(self, data: dict):
        """Process one frame of Quest 3 controller data."""
        now = time.time()
        dt = (now - self.prev_time) if self.prev_time else 0.02
        self.prev_time = now

        if dt < 1e-4 or dt > 0.5:
            dt = 0.02  # Clamp to reasonable range

        left_grip = data.get("left", {}).get("grip", 0.0)
        right_grip = data.get("right", {}).get("grip", 0.0)
        if self.log_controller_inputs and now - self.last_input_log_time >= 1.0:
            self.get_logger().info(
                f"Grip values - left: {left_grip:.2f}, right: {right_grip:.2f}, "
                f"deadman_thresh: {self.deadman_thresh}"
            )
            self.last_input_log_time = now

        for side in ["left", "right"]:
            controller = data.get(side)
            if not controller:
                continue

            pos = np.array(controller["pos"]) * self.pos_scale
            rot_q = controller["rot"]  # [qx, qy, qz, qw]
            rot = quat_to_matrix(rot_q)
            grip_val = controller.get("grip", 0.0)
            trigger_val = controller.get("trigger", 0.0)

            # Deadman switch check
            deadman_val = grip_val if self.deadman_btn == "grip" else trigger_val
            is_active = deadman_val > self.deadman_thresh

            # Get previous state
            if side == "left":
                prev_pos = self.prev_left_pos
                prev_rot = self.prev_left_rot
                twist_pub = self.left_twist_pub
                grip_pub = self.left_grip_pub
            else:
                prev_pos = self.prev_right_pos
                prev_rot = self.prev_right_rot
                twist_pub = self.right_twist_pub
                grip_pub = self.right_grip_pub

            if is_active and prev_pos is not None and prev_rot is not None:
                # Compute velocity commands
                linear_vel = (pos - prev_pos) / dt
                angular_vel = rotation_to_angular_velocity(prev_rot, rot, dt)

                # Coordinate frame mapping: Quest → Robot
                # Quest: X=right, Y=up, Z=back (towards user)
                # Robot (typical): X=forward, Y=left, Z=up
                # Remap: robot_x = -quest_z, robot_y = -quest_x, robot_z = quest_y
                linear_vel_robot = np.array([
                    -linear_vel[2],  # forward = -quest_z
                    -linear_vel[0],  # left = -quest_x
                    linear_vel[1],  # up = quest_y
                ])
                angular_vel_robot = np.array([
                    -angular_vel[2],
                    -angular_vel[0],
                    angular_vel[1],
                ])

                # Clamp velocities
                lin_norm = np.linalg.norm(linear_vel_robot)
                if lin_norm > self.max_lin:
                    linear_vel_robot *= self.max_lin / lin_norm

                ang_norm = np.linalg.norm(angular_vel_robot)
                if ang_norm > self.max_ang:
                    angular_vel_robot *= self.max_ang / ang_norm

                # Publish TwistStamped
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = (
                    "openarm_left_hand" if side == "left" else "openarm_right_hand"
                )
                msg.twist.linear.x = float(linear_vel_robot[0])
                msg.twist.linear.y = float(linear_vel_robot[1])
                msg.twist.linear.z = float(linear_vel_robot[2])
                msg.twist.angular.x = float(angular_vel_robot[0])
                msg.twist.angular.y = float(angular_vel_robot[1])
                msg.twist.angular.z = float(angular_vel_robot[2])
                twist_pub.publish(msg)

            # Publish gripper position (map grip trigger 0-1 to open-closed)
            # grip=0 → fully open, grip=1 → fully closed
            other_btn = trigger_val if self.deadman_btn == "grip" else grip_val
            grip_pos = self.grip_open - other_btn * (self.grip_open - self.grip_closed)
            grip_msg = Float64()
            grip_msg.data = float(grip_pos)
            grip_pub.publish(grip_msg)

            # Update previous state
            if side == "left":
                self.prev_left_pos = pos.copy()
                self.prev_left_rot = rot.copy()
            else:
                self.prev_right_pos = pos.copy()
                self.prev_right_rot = rot.copy()

    async def ws_handler(self, websocket):
        """Handle a WebSocket connection from Quest 3."""
        self.get_logger().info(
            f"Quest 3 connected from {websocket.remote_address}"
        )

        # Reset state on new connection
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.prev_left_rot = None
        self.prev_right_rot = None
        self.prev_time = None

        # Start servo nodes
        self.start_servos()

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self.process_quest_data(data)
                except (json.JSONDecodeError, KeyError, IndexError) as e:
                    self.get_logger().warn(f"Bad message: {e}")
        except websockets.ConnectionClosed:
            self.get_logger().info("Quest 3 disconnected")

    async def run_ws_server(self):
        """Run the WebSocket server."""
        # Try to use SSL for WSS, fall back to plain WS if certificates don't exist
        ssl_context = None
        cert_file = f"{os.path.expanduser('~')}/.openarm_teleop_certs/cert.pem"
        key_file = f"{os.path.expanduser('~')}/.openarm_teleop_certs/key.pem"

        if os.path.exists(cert_file) and os.path.exists(key_file):
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(cert_file, key_file)
            protocol = "wss"
        else:
            protocol = "ws"

        async with websockets.serve(
            self.ws_handler, "0.0.0.0", self.ws_port, ssl=ssl_context
        ):
            self.get_logger().info(
                f"WebSocket server running on {protocol}://0.0.0.0:{self.ws_port}"
            )
            await asyncio.Future()  # Run forever


def main(args=None):
    rclpy.init(args=args)
    node = QuestBridge()

    # Run ROS2 spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run WebSocket server in asyncio event loop
    try:
        asyncio.run(node.run_ws_server())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
