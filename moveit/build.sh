#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build the MoveIt config package into a local colcon overlay at moveit/ws.
#
# The config is a plain ament_cmake package; it does not need MoveIt or the
# sim at build time. After this, `get_package_share_directory("nero_gripper_moveit_config")`
# and `ros2 launch nero_gripper_moveit_config move_group.launch.py` resolve.
#
# Usage:  bash moveit/build.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HERE/ws"
ROS_DISTRO="${ROS_DISTRO:-humble}"

mkdir -p "$WS/src"
ln -sfn "$HERE/nero_gripper_moveit_config" "$WS/src/nero_gripper_moveit_config"

# ROS 2 Humble is built for Python 3.10. If `uv` (Python 3.11) is earlier on
# PATH, CMake's package_xml_2_cmake.py would run under 3.11 and miss catkin_pkg,
# so force the system interpreter.
export PATH="/usr/bin:$PATH"

# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

cd "$WS"
colcon build --symlink-install --packages-select nero_gripper_moveit_config \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

echo "[moveit/build] done. source this before launching:"
echo "  source \"$WS/install/setup.bash\""
