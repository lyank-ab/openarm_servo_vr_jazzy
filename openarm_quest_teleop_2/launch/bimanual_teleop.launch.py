import os
import yaml
import subprocess
import tempfile
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Robot description ────────────────────────────────────────────────────
    # Read URDF from the already-running robot_state_publisher.
    # REQUIRES: demo.launch.py must be running before this launch file.
    result = subprocess.run(
        ["ros2", "param", "get",
         "/robot_state_publisher", "robot_description",
         "--hide-type"],
        capture_output=True,
        text=True,
        timeout=10,
    )
    if result.returncode != 0 or not result.stdout.strip():
        raise RuntimeError(
            "Could not read robot_description from /robot_state_publisher. "
            "Make sure demo.launch.py is running first.\n"
            f"stderr: {result.stderr}"
        )
    robot_description_content = result.stdout.strip()

    # ── SRDF + kinematics ────────────────────────────────────────────────────
    moveit_share = get_package_share_directory("openarm_bimanual_moveit_config")

    with open(os.path.join(moveit_share, "config", "openarm_bimanual.srdf"), "r") as f:
        robot_description_semantic = f.read()

    # Load kinematics and write to a temp params file with the correct
    # ROS2 parameter namespace (robot_description_kinematics.<group>.*).
    # Passing a nested dict directly to Node(parameters=...) does NOT
    # reliably create the nested parameter structure — a params file does.
    with open(os.path.join(moveit_share, "config", "kinematics.yaml"), "r") as f:
        kin_raw = yaml.safe_load(f) or {}

    kin_tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="servo_kinematics_"
    )
    yaml.dump(
        {"/**": {"ros__parameters": {"robot_description_kinematics": kin_raw}}},
        kin_tmp,
    )
    kin_tmp.close()

    # ── Servo configs ────────────────────────────────────────────────────────
    servo_share = get_package_share_directory("openarm_quest_teleop_2")
    left_cfg = os.path.join(servo_share, "config", "left_arm_servo.yaml")
    right_cfg = os.path.join(servo_share, "config", "right_arm_servo.yaml")

    common = [
        {"robot_description":          robot_description_content},
        {"robot_description_semantic": robot_description_semantic},
        kin_tmp.name,   # kinematics params file — most reliable way
        {"update_period": 0.01},
    ]

    left_common = [*common, {"planning_group_name": "left_arm"}]
    right_common = [*common, {"planning_group_name": "right_arm"}]

    left_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        namespace="left",
        name="servo_node",
        parameters=[*left_common, left_cfg],
        remappings=[
            ("/left/joint_states", "/joint_states"),
        ],
        output="screen",
    )

    right_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        namespace="right",
        name="servo_node",
        parameters=[*right_common, right_cfg],
        remappings=[
            ("/right/joint_states", "/joint_states"),
        ],
        output="screen",
    )

    # ── Auto-start servo nodes and set command type ──────────────────────────
    # In Jazzy, MoveIt Servo starts in PAUSED state. Two service calls are
    # needed: start_servo (un-pause) and switch_command_type (set TWIST=1).
    start_left = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=["ros2", "service", "call",
                 "/left/servo_node/start_servo",
                 "std_srvs/srv/Trigger", "{}"],
            output="screen",
        )],
    )

    start_right = TimerAction(
        period=5.5,
        actions=[ExecuteProcess(
            cmd=["ros2", "service", "call",
                 "/right/servo_node/start_servo",
                 "std_srvs/srv/Trigger", "{}"],
            output="screen",
        )],
    )

    # command_type: 0=JOINT_JOG, 1=TWIST, 2=POSE_TARGET
    cmd_type_left = TimerAction(
        period=7.0,
        actions=[ExecuteProcess(
            cmd=["ros2", "service", "call",
                 "/left/servo_node/switch_command_type",
                 "moveit_msgs/srv/ServoCommandType",
                 "{command_type: 1}"],
            output="screen",
        )],
    )

    cmd_type_right = TimerAction(
        period=7.5,
        actions=[ExecuteProcess(
            cmd=["ros2", "service", "call",
                 "/right/servo_node/switch_command_type",
                 "moveit_msgs/srv/ServoCommandType",
                 "{command_type: 1}"],
            output="screen",
        )],
    )

    return LaunchDescription([
        left_servo,
        right_servo,
        start_left,
        start_right,
        cmd_type_left,
        cmd_type_right,
    ])
