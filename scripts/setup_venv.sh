#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# (Re)create the repo-local `.venv` used by sim/start.sh.
#
# The sim needs a Python 3.10 interpreter with both MuJoCo and rclpy. rclpy is
# provided by the system ROS 2 install (/opt/ros/humble) and is pulled in via a
# `.pth` file rather than installed into the venv; MuJoCo + its Python deps are
# installed into the venv with `uv`.
#
# Usage:
#   bash scripts/setup_venv.sh          # (re)create .venv
#   NERO_VENV=/path/to/venv bash sim/start.sh   # alternative: use another venv
set -euo pipefail

PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
UV="${UV:-uv}"

PY="${PYTHON3_10:-/usr/bin/python3.10}"
if [[ ! -x "$PY" ]]; then
  echo "[setup_venv] /usr/bin/python3.10 not found; set PYTHON3_10=/path/to/python3.10" >&2
  exit 1
fi

echo "[setup_venv] creating venv at $PKG/.venv (python $PY)"
"$UV" venv --python "$PY" "$PKG/.venv"

# MuJoCo + deps (pinned to what the sim is currently tested against).
"$UV" pip install --python "$PKG/.venv/bin/python" \
  mujoco==3.3.4 \
  numpy==2.2.6 \
  glfw==2.10.2 \
  pyopengl==3.1.10 \
  absl-py==2.5.0 \
  etils==1.13.0 \
  fsspec==2026.7.0 \
  importlib-resources==7.1.0 \
  typing-extensions==4.16.0 \
  zipp==4.1.0

# Pull rclpy + ROS message types in from the system ROS 2 install.
SITE="$PKG/.venv/lib/python3.10/site-packages"
mkdir -p "$SITE"
cat > "$SITE/ros2_humble.pth" <<EOF
/opt/ros/${ROS_DISTRO}/local/lib/python3.10/dist-packages
/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages
EOF

echo "[setup_venv] verifying:"
"$PKG/.venv/bin/python" -c "import mujoco, rclpy; print('  mujoco', mujoco.__version__, '+ rclpy OK')"
