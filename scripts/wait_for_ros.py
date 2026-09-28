#!/usr/bin/env python3
"""Wait for one ROS 2 topic or service without using the ros2 CLI daemon."""
from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--topic")
    group.add_argument("--service")
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        print(f"[wait_for_ros] rclpy unavailable: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = Node("nero_startup_readiness")
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if args.topic:
                ready = any(name == args.topic for name, _types in node.get_topic_names_and_types())
            else:
                ready = any(name == args.service for name, _types in node.get_service_names_and_types())
            if ready:
                print(f"[wait_for_ros] ready: {args.topic or args.service}", flush=True)
                return 0
            rclpy.spin_once(node, timeout_sec=0.25)
        print(f"[wait_for_ros] timeout waiting for {args.topic or args.service}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
