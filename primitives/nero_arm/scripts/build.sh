#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build the Nero arm primitive: create a dedicated venv with robonix-api +
# grpcio-tools, then run rbnx codegen (--mcp) into rbnx-build/codegen/.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
VENV="$PKG/.venv"
ROBONIX_API="$(rbnx path robonix-api)"

# One venv serves both codegen (grpcio-tools) and runtime (robonix-api + deps).
# Pin grpc/protobuf so the generated stubs and the runtime share one protobuf
# major, and pin mcp/fastmcp to the versions the Robonix uv.lock resolves to
# (mcp 1.x — robonix_api imports FastMCP from mcp.server.fastmcp, which mcp 2.x
# renamed to MCPServer).
if [[ ! -x "$VENV/bin/python" ]]; then
  uv venv --python 3.10 "$VENV"
  uv pip install --python "$VENV/bin/python" \
    protobuf==6.33.6 \
    grpcio-tools==1.76.0 \
    grpcio==1.78.0 \
    mcp==1.27.0 \
    fastmcp==3.2.4 \
    numpy \
    "$ROBONIX_API"
fi

# rbnx codegen shells out to `python3 -m grpc_tools.protoc`, resolving python3
# from PATH (it has no python override flag). Put the venv's python3 first so
# the grpcio-tools installed above is the one protoc uses.
export PATH="$VENV/bin:$PATH"
rbnx codegen -p "$PKG" --mcp

echo "[build] done: $PKG"
