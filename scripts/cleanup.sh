#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Tear down the whole Robonix deployment + its spawned sim, even when
# `rbnx shutdown` can't (state.json missing / boot crashed). Idempotent.
#
#   bash scripts/cleanup.sh            # robonix stack + sim bridge
#   bash scripts/cleanup.sh --all      # + MoveIt move_group (needed by `pick`)
#
# Safe: only matches robonix/rbnx/bridge/primitive process names, never a
# bare `python` or `ros2`. Leaves move_group alone unless `--all` is passed.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KILL_ALL=0
[[ "${1:-}" == "--all" ]] && KILL_ALL=1

# 1) Graceful teardown via the official state file (best effort).
rbnx shutdown -f "$ROOT/robonix_manifest.yaml" >/dev/null 2>&1 || true

# 2) Kill any remaining orchestration / service processes.
pkill -TERM -f "robonix-"            2>/dev/null || true
pkill -TERM -f "rbnx boot"           2>/dev/null || true
pkill -TERM -f "rbnx start"          2>/dev/null || true
pkill -TERM -f "sim/bridge/bridge_node.py" 2>/dev/null || true
pkill -TERM -f "nero_arm.main"       2>/dev/null || true
sleep 1
pkill -KILL -f "robonix-"            2>/dev/null || true
pkill -KILL -f "rbnx boot"           2>/dev/null || true
pkill -KILL -f "rbnx start"          2>/dev/null || true
pkill -KILL -f "sim/bridge/bridge_node.py" 2>/dev/null || true
pkill -KILL -f "nero_arm.main"       2>/dev/null || true

# 3) Optionally also stop MoveIt move_group (spawned outside rbnx, needed by pick).
if [[ "$KILL_ALL" == "1" ]]; then
  pkill -TERM -f "move_group.launch.py" 2>/dev/null || true
  pkill -TERM -f "moveit_ros_move_group" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "move_group.launch.py" 2>/dev/null || true
  pkill -KILL -f "moveit_ros_move_group" 2>/dev/null || true
fi

# 4) Report leftovers + port state.
sleep 1
LEFTOVER=$(ps -eo pid,comm,args 2>/dev/null | grep -E "robonix-|rbnx (boot|start)|bridge_node.py|nero_arm.main" | grep -v grep)
if [[ -n "$LEFTOVER" ]]; then
  echo "[cleanup] leftover processes:"; echo "$LEFTOVER"
else
  echo "[cleanup] no robonix processes remaining"
fi

PORTS=$(ss -tln 2>/dev/null | grep -E ":50051 |:50061 |:50072 |:50082 |:50091 " || true)
if [[ -n "$PORTS" ]]; then
  echo "[cleanup] robonix ports still listening:"; echo "$PORTS"
else
  echo "[cleanup] robonix ports (50051/50061/50072/50082/50091) free"
fi
