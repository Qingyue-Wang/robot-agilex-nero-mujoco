#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Start the Nero Native MuJoCo simulation + ROS 2 bridge (foreground).
#
# The bridge owns the physics loop AND the ROS topics, so this single
# process is the whole sim layer. Bring it up BEFORE `rbnx boot` (the arm
# primitive's on_init waits for the first /joint_states as its sentinel).
#
# Usage:
#   bash sim/start.sh                 # headless
#   bash sim/start.sh --viewer        # open a mujoco.viewer window
#
# Interpreter: NERO_PYTHON overrides the Python used. The default auto-detects
# the repo-local `.venv` (a Python 3.10 venv with MuJoCo + rclpy; see
# scripts/setup_venv.sh) or `$NERO_VENV`. ROS 2 must be installed at
# /opt/ros/<ROS_DISTRO>.
set -euo pipefail

PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
else
  echo "[nero/start] WARN: /opt/ros/${ROS_DISTRO}/setup.bash not found; rclpy may be unavailable" >&2
fi

if [[ -z "${NERO_PYTHON:-}" ]]; then
  NERO_PYTHON=""
  for cand in \
      "$PKG/.venv/bin/python" \
      "${NERO_VENV:-/nonexistent}/bin/python"; do
    if [[ -x "$cand" ]]; then NERO_PYTHON="$cand"; break; fi
  done
  [[ -z "$NERO_PYTHON" ]] && NERO_PYTHON="python3"
fi
echo "[nero/start] using python: $NERO_PYTHON"

export PYTHONPATH="$PKG/sim:${PYTHONPATH:-}"
cd "$PKG"
exec "$NERO_PYTHON" "$PKG/sim/bridge/bridge_node.py" "$@"
