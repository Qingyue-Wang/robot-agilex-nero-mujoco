#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Stop the Nero sim bridge (the single foreground process started by start.sh).
set -euo pipefail

pkill -TERM -f "sim/bridge/bridge_node.py" 2>/dev/null || true
sleep 1
pkill -KILL -f "sim/bridge/bridge_node.py" 2>/dev/null || true
echo "[nero/stop] bridge stopped"
