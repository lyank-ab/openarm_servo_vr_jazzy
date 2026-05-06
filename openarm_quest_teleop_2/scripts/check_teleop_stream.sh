#!/usr/bin/env bash
#
# Quest teleop stream checker.
#
# Purpose:
#
#   Use this BEFORE starting MoveIt Servo or the robot. It verifies only the
#   data transmission path:
#
#       Quest browser WebXR page -> WebSocket -> this terminal
#
#   It does not start ROS, MoveIt, ros2_control, quest_bridge.py, or any robot
#   controller. Nothing in this script can command the arms.
#
# What success looks like:
#
#   After you connect from the Quest page and enter VR, this terminal should
#   print a live message rate and left/right controller data, for example:
#
#       messages=144 rate=71.9 Hz left=yes grip=0.83 trigger=0.01 right=yes ...
#
# Recommended Quest test:
#
#   1. Run this script:
#
#        cd ~/open_ws/openarm_servo_vr_jazzy/openarm_quest_teleop_2
#        ./scripts/check_teleop_stream.sh
#
#   2. On the Quest browser, open the printed HTTPS URL.
#
#   3. Accept the self-signed certificate warning.
#
#   4. In the page's "Server" field, use the printed WSS URL:
#
#        wss://<this-computer-ip>:9090
#
#      The page may auto-fill ws://..., but from an HTTPS WebXR page, wss:// is
#      the safer diagnostic path.
#
#   5. Press "Connect WebSocket", then "Enter VR".
#
#   6. Move the controllers. Hold or press buttons if you also want to see grip
#      and trigger values change.
#
# Stop:
#
#   Press Ctrl+C in this terminal.
#
# Notes:
#
#   - This diagnostic uses WSS by default because WebXR on Quest normally needs
#     HTTPS, and browsers often block plain ws:// connections from HTTPS pages.
#   - The runtime quest_bridge.py also uses these cert files when present, so
#     HTTPS WebXR and WSS bridge traffic can use the same local certificate.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WEB_DIR="${PACKAGE_DIR}/web"

WEB_PORT="8443"
WS_PORT="9090"
SECURE="true"
SERVE_WEB="true"
CERT_DIR="${HOME}/.openarm_teleop_certs"
CERT_FILE="${CERT_DIR}/cert.pem"
KEY_FILE="${CERT_DIR}/key.pem"

PIDS=()

usage() {
  cat <<EOF
Usage:
  $0 [options]

Options:
  --web-port PORT      Web page port. Default: 8443.
  --ws-port PORT       WebSocket monitor port. Default: 9090.
  --plain              Use HTTP + ws:// instead of HTTPS + wss://.
  --no-web             Do not serve web/index.html; only listen for WebSocket.
  -h, --help           Show this help.

Examples:
  $0
  $0 --ws-port 9091 --web-port 8444
  $0 --plain
  $0 --no-web
EOF
}

log() {
  printf '[teleop-stream-check] %s\n' "$*"
}

die() {
  log "ERROR: $*"
  exit 1
}

cleanup() {
  trap - INT TERM EXIT
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -- "-${pid}" >/dev/null 2>&1 || kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --web-port)
        [[ $# -ge 2 ]] || die "--web-port requires a value"
        WEB_PORT="$2"
        shift 2
        ;;
      --ws-port)
        [[ $# -ge 2 ]] || die "--ws-port requires a value"
        WS_PORT="$2"
        shift 2
        ;;
      --plain)
        SECURE="false"
        shift
        ;;
      --no-web)
        SERVE_WEB="false"
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

require_tools() {
  [[ -d "${WEB_DIR}" ]] || die "Web directory not found: ${WEB_DIR}"
  command -v python3 >/dev/null 2>&1 || die "python3 is required"

  if ! python3 -c 'import websockets' >/dev/null 2>&1; then
    die "Python module 'websockets' is missing. Install it before running this checker."
  fi

  if [[ "${SECURE}" == "true" ]]; then
    command -v openssl >/dev/null 2>&1 || die "openssl is required for HTTPS/WSS mode"
  fi
}

generate_https_cert() {
  if [[ -f "${CERT_FILE}" && -f "${KEY_FILE}" ]]; then
    return 0
  fi

  mkdir -p "${CERT_DIR}"
  log "Generating self-signed certificate in ${CERT_DIR}"
  openssl req -x509 -newkey rsa:2048 \
    -keyout "${KEY_FILE}" \
    -out "${CERT_FILE}" \
    -days 365 \
    -nodes \
    -subj "/CN=openarm-teleop/O=AIR-Lab/C=US" >/dev/null 2>&1
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

start_web_server() {
  local scheme_arg="http"
  local cert_arg=""
  local key_arg=""

  if [[ "${SECURE}" == "true" ]]; then
    generate_https_cert
    scheme_arg="https"
    cert_arg="${CERT_FILE}"
    key_arg="${KEY_FILE}"
  fi

  log "Starting ${scheme_arg^^} web server on port ${WEB_PORT}"
  setsid python3 -c '
import http.server
import os
import ssl
import sys

port = int(sys.argv[1])
web_dir = sys.argv[2]
scheme = sys.argv[3]
cert_file = sys.argv[4]
key_file = sys.argv[5]

os.chdir(web_dir)
handler = http.server.SimpleHTTPRequestHandler
httpd = http.server.HTTPServer(("0.0.0.0", port), handler)

if scheme == "https":
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(cert_file, key_file)
    httpd.socket = context.wrap_socket(httpd.socket, server_side=True)

print(f"Serving {scheme.upper()} on 0.0.0.0:{port}", flush=True)
httpd.serve_forever()
' "${WEB_PORT}" "${WEB_DIR}" "${scheme_arg}" "${cert_arg}" "${key_arg}" &

  local pid=$!
  PIDS+=("${pid}")
  sleep 1

  if ! kill -0 "${pid}" >/dev/null 2>&1; then
    die "Web server failed to start"
  fi
}

run_websocket_monitor() {
  local mode_arg="ws"
  local cert_arg=""
  local key_arg=""

  if [[ "${SECURE}" == "true" ]]; then
    generate_https_cert
    mode_arg="wss"
    cert_arg="${CERT_FILE}"
    key_arg="${KEY_FILE}"
  fi

  python3 - "${WS_PORT}" "${mode_arg}" "${cert_arg}" "${key_arg}" <<'PY'
import asyncio
import json
import ssl
import sys
import time

import websockets

port = int(sys.argv[1])
mode = sys.argv[2]
cert_file = sys.argv[3]
key_file = sys.argv[4]

ssl_context = None
if mode == "wss":
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(cert_file, key_file)

stats = {
    "count": 0,
    "start": None,
    "last_print": 0.0,
    "last_payload": None,
}


def compact_side(payload, side):
    controller = payload.get(side)
    if not controller:
        return f"{side}=no"

    pos = controller.get("pos", [0.0, 0.0, 0.0])
    grip = float(controller.get("grip", 0.0))
    trigger = float(controller.get("trigger", 0.0))

    try:
        pos_text = ",".join(f"{float(value): .3f}" for value in pos[:3])
    except Exception:
        pos_text = "bad-pos"

    return (
        f"{side}=yes "
        f"pos=({pos_text}) "
        f"grip={grip:.2f} "
        f"trigger={trigger:.2f}"
    )


async def handler(websocket):
    peer = getattr(websocket, "remote_address", "unknown")
    print(f"Quest/WebXR client connected from {peer}", flush=True)

    async for message in websocket:
        now = time.time()
        if stats["start"] is None:
            stats["start"] = now

        try:
            payload = json.loads(message)
        except json.JSONDecodeError as exc:
            print(f"Bad JSON message: {exc}", flush=True)
            continue

        stats["count"] += 1
        stats["last_payload"] = payload

        if now - stats["last_print"] >= 1.0:
            elapsed = max(now - stats["start"], 1e-6)
            rate = stats["count"] / elapsed
            left = compact_side(payload, "left")
            right = compact_side(payload, "right")
            print(
                f"messages={stats['count']} "
                f"rate={rate:5.1f} Hz "
                f"{left} "
                f"{right}",
                flush=True,
            )
            stats["last_print"] = now


async def main():
    print(f"Listening for {mode.upper()} teleop data on 0.0.0.0:{port}", flush=True)
    async with websockets.serve(handler, "0.0.0.0", port, ssl=ssl_context):
        await asyncio.Future()


try:
    asyncio.run(main())
except OSError as exc:
    print(
        f"Could not bind {mode.upper()} server on 0.0.0.0:{port}: {exc}",
        file=sys.stderr,
        flush=True,
    )
    print(
        "Choose another port with --ws-port, or stop the process using this port.",
        file=sys.stderr,
        flush=True,
    )
    sys.exit(2)
except KeyboardInterrupt:
    pass
PY
}

print_instructions() {
  local local_ip="$1"
  local web_scheme="http"
  local ws_scheme="ws"

  if [[ "${SECURE}" == "true" ]]; then
    web_scheme="https"
    ws_scheme="wss"
  fi

  cat <<EOF

Teleop stream checker is ready.

This test only checks:
  Quest browser WebXR page -> WebSocket -> this terminal

It does not start ROS or command the robot.

EOF

  if [[ "${SERVE_WEB}" == "true" ]]; then
    cat <<EOF
Open this on the Quest:
  ${web_scheme}://${local_ip}:${WEB_PORT}

In the page's Server field, use:
  ${ws_scheme}://${local_ip}:${WS_PORT}

EOF
  else
    cat <<EOF
Web page serving is disabled.

Point your existing page's Server field at:
  ${ws_scheme}://${local_ip}:${WS_PORT}

EOF
  fi

  cat <<EOF
Then press:
  Connect WebSocket
  Enter VR

Expected terminal output:
  messages=... rate=...Hz left=yes ... right=yes ...

Press Ctrl+C to stop.

EOF
}

main() {
  parse_args "$@"
  trap cleanup INT TERM EXIT

  require_tools

  if [[ "${SERVE_WEB}" == "true" ]]; then
    start_web_server
  fi

  local local_ip
  local_ip="$(get_local_ip)"
  print_instructions "${local_ip}"

  run_websocket_monitor
}

main "$@"
