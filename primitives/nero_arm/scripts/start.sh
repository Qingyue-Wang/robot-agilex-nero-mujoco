#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Run the Nero arm primitive (foreground). Sources ROS 2 so rclpy + std_msgs
# are importable, then launches nero_arm.main with the package venv.
#
# The primitive spawns sim/start.sh itself in on_activate, so nothing else
# needs to be running first (except the Atlas stack from `rbnx boot`).
set -euo pipefail

PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$PKG/.venv"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "[nero_arm/start] missing $VENV/bin/python — run scripts/build.sh first" >&2
  exit 1
fi

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
else
  echo "[nero_arm/start] WARN: /opt/ros/${ROS_DISTRO}/setup.bash not found; rclpy may be unavailable" >&2
fi

# robonix-api is installed in the venv; the codegen dirs are added to sys.path
# automatically by robonix_api.ensure_proto_gen() when Primitive() is built.
export PYTHONPATH="$PKG:${PYTHONPATH:-}"
export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
export ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-127.0.0.1}"

cd "$PKG"
exec "$VENV/bin/python" -m nero_arm.main
