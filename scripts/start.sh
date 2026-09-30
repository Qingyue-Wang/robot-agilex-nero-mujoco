#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Start the complete Nero simulation, Robonix, and MoveIt stack.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
TIMEOUT="${NERO_STARTUP_TIMEOUT:-90}"
LOG_DIR="${NERO_LOG_DIR:-$ROOT/logs/startup}"
BUILD=1
CLEANED=0
RBNX_PID=""
MOVEIT_PID=""

usage() {
  cat <<EOF
Usage: bash scripts/start.sh [--skip-build] [--timeout SECONDS]

Starts Robonix (which owns the MuJoCo bridge) and MoveIt move_group.
EOF
}

while (($#)); do
  case "$1" in
    --skip-build) BUILD=0; shift ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[startup] unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

fail() { echo "[startup] ERROR: $*" >&2; exit 1; }
need_file() { [[ -f "$1" ]] || fail "missing $1"; }
need_cmd() { command -v "$1" >/dev/null 2>&1 || fail "command not found: $1"; }

need_file "/opt/ros/$ROS_DISTRO/setup.bash"
need_file "$ROOT/moveit/build.sh"
need_file "$ROOT/moveit/nero_gripper_moveit_config/launch/move_group.launch.py"
need_file "$ROOT/scripts/wait_for_ros.py"
need_cmd rbnx
need_cmd python3
mkdir -p "$LOG_DIR"

# ROS setup scripts assume bash and may reference variables which strict mode
# considers unset. Never source these from the user's zsh shell.
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX 2>/dev/null || true
set +u
# shellcheck disable=SC1091
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

if ((BUILD)); then
  echo "[startup] building MoveIt overlay"
  bash "$ROOT/moveit/build.sh" | tee "$LOG_DIR/moveit-build.log"
fi
need_file "$ROOT/moveit/ws/install/setup.bash"
set +u
# shellcheck disable=SC1091
source "$ROOT/moveit/ws/install/setup.bash"
set -u
ros2 pkg prefix nero_gripper_moveit_config >/dev/null 2>&1 || \
  fail "MoveIt package is not discoverable; run bash moveit/build.sh"

if [[ -f "$ROOT/.env" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$ROOT/.env"
  set -u
fi

cleanup() {
  local status=$?
  [[ "$CLEANED" -eq 1 ]] && return "$status"
  CLEANED=1
  trap - INT TERM EXIT
  echo "[startup] stopping MoveIt and Robonix"
  if [[ -n "$MOVEIT_PID" ]] && kill -0 "$MOVEIT_PID" 2>/dev/null; then
    kill -TERM -- "-$MOVEIT_PID" 2>/dev/null || kill -TERM "$MOVEIT_PID" 2>/dev/null || true
  fi
  bash "$ROOT/scripts/cleanup.sh" --all || true
  return "$status"
}
trap cleanup INT TERM EXIT

# The primitive starts sim/start.sh on activation; starting it here would make
# two bridge nodes publish the same ROS topics.
echo "[startup] starting Robonix; primitive owns the MuJoCo bridge"
setsid rbnx boot -f "$ROOT/robonix_manifest.yaml" >"$LOG_DIR/rbnx.log" 2>&1 &
RBNX_PID=$!

wait_ros() {
  local kind="$1" name="$2"
  python3 "$ROOT/scripts/wait_for_ros.py" "--$kind" "$name" --timeout "$TIMEOUT"
}

wait_ros topic /joint_states || {
  echo "[startup] Robonix/bridge did not publish /joint_states; see $LOG_DIR/rbnx.log" >&2
  tail -n 40 "$LOG_DIR/rbnx.log" >&2 || true
  exit 1
}

if ! kill -0 "$RBNX_PID" 2>/dev/null; then
  echo "[startup] rbnx exited before MoveIt startup" >&2
  tail -n 40 "$LOG_DIR/rbnx.log" >&2 || true
  exit 1
fi

echo "[startup] starting move_group"
setsid ros2 launch nero_gripper_moveit_config move_group.launch.py >"$LOG_DIR/move_group.log" 2>&1 &
MOVEIT_PID=$!

for service in /compute_fk /compute_ik /compute_cartesian_path; do
  wait_ros service "$service" || {
    echo "[startup] MoveIt did not provide $service; see $LOG_DIR/move_group.log" >&2
    tail -n 50 "$LOG_DIR/move_group.log" >&2 || true
    exit 1
  }
done

echo "[startup] stack ready"
echo "[startup] logs: $LOG_DIR"
wait "$RBNX_PID"
