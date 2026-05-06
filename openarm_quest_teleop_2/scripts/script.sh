#!/usr/bin/env bash
#
# OpenArm Quest 3 bimanual teleoperation launcher.
#
# This script wraps the normal multi-terminal workflow into one supervised
# terminal. It is intended for first tests with fake hardware, but it also has
# options for real hardware once the robot is physically ready and safe.
#
# First-time preflight:
#
#   Before running this full pipeline, verify that the Quest is transmitting
#   controller data:
#
#       ./scripts/check_teleop_stream.sh
#
# What this script starts, in order:
#
#   1. Robot + MoveIt demo launch
#      - Starts /robot_state_publisher
#      - Starts ros2_control controllers
#      - Starts MoveIt move_group and RViz
#
#   2. Quest teleop MoveIt Servo launch
#      - Starts /left/servo_node
#      - Starts /right/servo_node
#      - Each Servo node listens for Cartesian TwistStamped commands:
#          /left/servo_node/delta_twist_cmds
#          /right/servo_node/delta_twist_cmds
#      - Each Servo node outputs JointTrajectory commands:
#          /left_joint_trajectory_controller/joint_trajectory
#          /right_joint_trajectory_controller/joint_trajectory
#
#   3. Quest bridge node
#      - Starts a WebSocket server, default ws://0.0.0.0:9090.
#        If HTTPS certs are present, the bridge serves wss:// instead.
#      - Receives Quest controller poses from web/index.html
#      - Converts frame-to-frame controller motion into TwistStamped commands
#      - Publishes those commands to the two MoveIt Servo nodes
#
#   4. Optional HTTPS web server for the Quest page
#      - Serves this package's web/ directory, default https://<this-ip>:8443
#      - WebXR on a Quest headset normally requires HTTPS.
#      - You must accept the self-signed certificate warning in the Quest
#        browser the first time you open the page.
#
# Basic fake-hardware usage:
#
#   cd ~/open_ws/openarm_servo_vr_jazzy/openarm_quest_teleop_2
#   chmod +x scripts/script.sh
#   ./scripts/script.sh
#
# On the Quest:
#
#   1. Open the printed HTTPS URL.
#   2. Accept the certificate warning.
#   3. Confirm the WebSocket field matches the URL printed by this script.
#   4. Press "Connect WebSocket".
#   5. Press "Enter VR".
#   6. Hold the grip button as the deadman switch and move the controllers.
#
# Useful checks from another terminal:
#
#   ros2 topic echo /left/servo_node/delta_twist_cmds
#   ros2 topic echo /right/servo_node/delta_twist_cmds
#   ros2 topic echo /left_joint_trajectory_controller/joint_trajectory
#   ros2 topic echo /right_joint_trajectory_controller/joint_trajectory
#
# Safety notes:
#
#   - Default mode is fake hardware.
#   - For real hardware, use --real only after the robot area is clear, E-stop
#     is available, CAN interfaces are correct, and joint limits are verified.
#   - MoveIt Servo is live velocity control. Small controller motions can
#     produce immediate robot motion while the deadman button is held.
#
# Current package caveats:
#
#   - The bridge publishes gripper values to /left_gripper_position and
#     /right_gripper_position, but this repository does not currently include a
#     matching gripper adapter for those Float64 topics.
#   - When this script serves the WebXR page over HTTPS, it creates the same
#     self-signed certs before starting the bridge so the bridge can use WSS.
#
# Stop everything:
#
#   Press Ctrl+C in this terminal. The script will try to stop every process it
#   launched.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PACKAGE_DIR}/../.." && pwd)"
WEB_DIR="${PACKAGE_DIR}/web"

USE_FAKE_HARDWARE="true"
START_ROBOT="true"
SERVE_WEB="true"
BUILD_FIRST="false"
WS_PORT="9090"
WEB_PORT="8443"
RIGHT_CAN_INTERFACE="can0"
LEFT_CAN_INTERFACE="can1"
LOG_DIR="${LOG_DIR:-/tmp/openarm_quest_teleop_2_$(date +%Y%m%d_%H%M%S)}"
CERT_DIR="${HOME}/.openarm_teleop_certs"
CERT_FILE="${CERT_DIR}/cert.pem"
KEY_FILE="${CERT_DIR}/key.pem"

PIDS=()
NAMES=()
LOGS=()

usage() {
  cat <<EOF
Usage:
  $0 [options]

Options:
  --fake                 Use fake hardware. This is the default.
  --real                 Use real hardware through the MoveIt demo launch.
  --right-can IFACE      Right arm CAN interface for --real. Default: can0.
  --left-can IFACE       Left arm CAN interface for --real. Default: can1.
  --ws-port PORT         WebSocket port for quest_bridge. Default: 9090.
  --web-port PORT        HTTPS port for the Quest web page. Default: 8443.
  --no-web               Do not start the HTTPS web server.
  --skip-robot           Do not start demo.launch.py; assume it is already running.
  --build                Run colcon build --symlink-install before launching.
  -h, --help             Show this help.

Examples:
  $0
  $0 --no-web
  $0 --ws-port 9091 --web-port 8444
  $0 --real --right-can can0 --left-can can1
  $0 --skip-robot
EOF
}

log() {
  printf '[openarm-quest-teleop] %s\n' "$*"
}

die() {
  log "ERROR: $*"
  exit 1
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --fake)
        USE_FAKE_HARDWARE="true"
        shift
        ;;
      --real)
        USE_FAKE_HARDWARE="false"
        shift
        ;;
      --right-can)
        [[ $# -ge 2 ]] || die "--right-can requires a value"
        RIGHT_CAN_INTERFACE="$2"
        shift 2
        ;;
      --left-can)
        [[ $# -ge 2 ]] || die "--left-can requires a value"
        LEFT_CAN_INTERFACE="$2"
        shift 2
        ;;
      --ws-port)
        [[ $# -ge 2 ]] || die "--ws-port requires a value"
        WS_PORT="$2"
        shift 2
        ;;
      --web-port)
        [[ $# -ge 2 ]] || die "--web-port requires a value"
        WEB_PORT="$2"
        shift 2
        ;;
      --no-web)
        SERVE_WEB="false"
        shift
        ;;
      --skip-robot)
        START_ROBOT="false"
        shift
        ;;
      --build)
        BUILD_FIRST="true"
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        die "Unknown option: $1"
        ;;
    esac
  done
}

cleanup() {
  local status=$?
  trap - INT TERM EXIT

  if [[ ${#PIDS[@]} -gt 0 ]]; then
    log "Stopping launched processes..."
  fi

  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -- "-${pid}" >/dev/null 2>&1 || kill "${pid}" >/dev/null 2>&1 || true
    fi
  done

  sleep 2

  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -KILL -- "-${pid}" >/dev/null 2>&1 || kill -KILL "${pid}" >/dev/null 2>&1 || true
    fi
  done

  if [[ -d "${LOG_DIR}" ]]; then
    log "Logs are in: ${LOG_DIR}"
  fi

  exit "${status}"
}

require_file() {
  local path="$1"
  local message="$2"
  [[ -f "${path}" ]] || die "${message}: ${path}"
}

source_ros_environment() {
  require_file "/opt/ros/jazzy/setup.bash" "ROS Jazzy setup file not found"
  # shellcheck source=/opt/ros/jazzy/setup.bash
  set +u
  source "/opt/ros/jazzy/setup.bash"
  set -u

  if [[ "${BUILD_FIRST}" == "true" ]]; then
    log "Building workspace with colcon build --symlink-install..."
    (
      cd "${WORKSPACE_DIR}"
      colcon build --symlink-install
    )
  fi

  require_file "${WORKSPACE_DIR}/install/setup.bash" \
    "Workspace setup file not found. Run with --build or build the workspace first"
  # shellcheck source=/dev/null
  set +u
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u

  command -v ros2 >/dev/null 2>&1 || die "ros2 command is not available after sourcing setup files"

  if ! python3 -c 'import websockets' >/dev/null 2>&1; then
    die "Python module 'websockets' is missing. Install it, then rerun this script."
  fi
}

start_process() {
  local name="$1"
  local logfile="$2"
  shift 2

  log "Starting ${name}"
  log "  log: ${logfile}"

  setsid "$@" >"${logfile}" 2>&1 &
  local pid=$!

  PIDS+=("${pid}")
  NAMES+=("${name}")
  LOGS+=("${logfile}")

  sleep 2
  if ! kill -0 "${pid}" >/dev/null 2>&1; then
    log "${name} exited early. Last log lines:"
    tail -n 80 "${logfile}" || true
    die "${name} failed to start"
  fi
}

wait_for_robot_description() {
  log "Waiting for /robot_state_publisher robot_description..."
  for _ in $(seq 1 90); do
    if ros2 param get /robot_state_publisher robot_description --hide-type >/dev/null 2>&1; then
      log "Robot description is available."
      return 0
    fi
    sleep 1
  done

  die "Timed out waiting for /robot_state_publisher. Check ${LOG_DIR}/robot_demo.log"
}

wait_for_node() {
  local node_name="$1"
  local label="$2"

  log "Waiting for ${label} (${node_name})..."
  for _ in $(seq 1 60); do
    if ros2 node list 2>/dev/null | grep -Fxq "${node_name}"; then
      log "${label} is available."
      return 0
    fi
    sleep 1
  done

  die "Timed out waiting for ${label}"
}

get_local_ip() {
  local ip_addr
  ip_addr="$(ip route get 8.8.8.8 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
  if [[ -z "${ip_addr}" ]]; then
    ip_addr="$(hostname -I 2>/dev/null | awk '{print $1}')"
  fi
  if [[ -z "${ip_addr}" ]]; then
    ip_addr="127.0.0.1"
  fi
  printf '%s\n' "${ip_addr}"
}

generate_https_cert() {
  if [[ -f "${CERT_FILE}" && -f "${KEY_FILE}" ]]; then
    return 0
  fi

  command -v openssl >/dev/null 2>&1 || die "openssl is required to generate HTTPS/WSS certificates"

  mkdir -p "${CERT_DIR}"
  log "Generating self-signed HTTPS certificate in ${CERT_DIR}"
  openssl req -x509 -newkey rsa:2048 \
    -keyout "${KEY_FILE}" \
    -out "${CERT_FILE}" \
    -days 365 \
    -nodes \
    -subj "/CN=openarm-teleop/O=AIR-Lab/C=US" >/dev/null 2>&1
}

start_web_server() {
  [[ -d "${WEB_DIR}" ]] || die "Web directory not found: ${WEB_DIR}"
  generate_https_cert

  local logfile="${LOG_DIR}/web_https.log"
  log "  serving: ${WEB_DIR}"
  start_process \
    "web_https" \
    "${logfile}" \
    python3 -c '
import http.server
import os
import ssl
import sys

port = int(sys.argv[1])
cert_file = sys.argv[2]
key_file = sys.argv[3]
web_dir = sys.argv[4]

os.chdir(web_dir)
handler = http.server.SimpleHTTPRequestHandler
httpd = http.server.HTTPServer(("0.0.0.0", port), handler)
context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(cert_file, key_file)
httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
print(f"Serving HTTPS on 0.0.0.0:{port}", flush=True)
httpd.serve_forever()
' "${WEB_PORT}" "${CERT_FILE}" "${KEY_FILE}" "${WEB_DIR}"
}

print_ready_instructions() {
  local local_ip="$1"
  local ws_scheme="ws"

  if [[ -f "${CERT_FILE}" && -f "${KEY_FILE}" ]]; then
    ws_scheme="wss"
  fi

  cat <<EOF

OpenArm Quest teleop is running.

Local machine IP:
  ${local_ip}
EOF

  if [[ "${SERVE_WEB}" == "true" ]]; then
    cat <<EOF
Quest browser:
  https://${local_ip}:${WEB_PORT}

WebSocket field in the page:
  ${ws_scheme}://${local_ip}:${WS_PORT}
EOF
  else
    cat <<EOF
HTTPS web server:
  skipped because --no-web was used

WebSocket bridge:
  ${ws_scheme}://${local_ip}:${WS_PORT}
EOF
  fi

  cat <<EOF

Controls:
  Left controller  -> left arm
  Right controller -> right arm
  Grip             -> deadman switch, hold to move
  Index trigger    -> gripper value publish, if a gripper adapter is present

Logs:
  ${LOG_DIR}

Helpful live checks from another terminal:
  source /opt/ros/jazzy/setup.bash
  source ${WORKSPACE_DIR}/install/setup.bash
  ros2 topic echo /left/servo_node/delta_twist_cmds
  ros2 topic echo /left_joint_trajectory_controller/joint_trajectory

Press Ctrl+C here to stop everything launched by this script.

EOF
}

main() {
  parse_args "$@"
  trap cleanup INT TERM EXIT

  mkdir -p "${LOG_DIR}"

  log "Package directory: ${PACKAGE_DIR}"
  log "Workspace directory: ${WORKSPACE_DIR}"
  log "Log directory: ${LOG_DIR}"

  source_ros_environment

  if [[ "${START_ROBOT}" == "true" ]]; then
    start_process \
      "robot_demo" \
      "${LOG_DIR}/robot_demo.log" \
      ros2 launch openarm_bimanual_moveit_config demo.launch.py \
      use_fake_hardware:="${USE_FAKE_HARDWARE}" \
      right_can_interface:="${RIGHT_CAN_INTERFACE}" \
      left_can_interface:="${LEFT_CAN_INTERFACE}"
  else
    log "Skipping robot demo launch; assuming it is already running."
  fi

  wait_for_robot_description

  start_process \
    "bimanual_moveit_servo" \
    "${LOG_DIR}/bimanual_moveit_servo.log" \
    ros2 launch openarm_quest_teleop_2 bimanual_teleop.launch.py

  wait_for_node "/left/servo_node" "left Servo node"
  wait_for_node "/right/servo_node" "right Servo node"

  log "Waiting a few seconds for Servo start and TWIST command-type service calls..."
  sleep 8

  if [[ "${SERVE_WEB}" == "true" ]]; then
    generate_https_cert
  fi

  start_process \
    "quest_bridge" \
    "${LOG_DIR}/quest_bridge.log" \
    ros2 run openarm_quest_teleop_2 quest_bridge \
    --ros-args \
    -p ws_port:="${WS_PORT}"

  local local_ip
  local_ip="$(get_local_ip)"

  if [[ "${SERVE_WEB}" == "true" ]]; then
    start_web_server
  else
    log "Skipping HTTPS web server."
  fi

  print_ready_instructions "${local_ip}"

  set +e
  wait -n "${PIDS[@]}"
  local exited_status=$?
  log "One launched process exited; stopping the rest."
  exit "${exited_status}"
}

main "$@"
