#!/usr/bin/env python3

import argparse
import re
import subprocess
import sys
import time
from typing import Optional, Set, List

import rclpy
from rclpy.node import Node


DEFAULT_SPACING = 1.5
DEFAULT_X = 0.0
DEFAULT_Y_BASE = 0.0
DEFAULT_YAW = 0.0


class NamespaceInspector(Node):
    """Helper node to inspect existing namespaces and determine next robot index."""

    def __init__(self):
        super().__init__('robot_namespace_inspector')

    def get_existing_indices(self) -> Set[int]:
        pattern = re.compile(r'^tb3_(\d+)$')
        indices: Set[int] = set()

        for _, namespace in self.get_node_names_and_namespaces():
            ns = namespace.strip('/')
            if not ns:
                continue
            match = pattern.match(ns)
            if match:
                try:
                    indices.add(int(match.group(1)))
                except ValueError:
                    continue
        return indices


def determine_next_index() -> int:
    rclpy.init(args=None)
    inspector = None
    try:
        inspector = NamespaceInspector()
        rclpy.spin_once(inspector, timeout_sec=0.1)
        indices = inspector.get_existing_indices()
    finally:
        if inspector is not None:
            try:
                inspector.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()

    if not indices:
        return 0
    return max(indices) + 1


def build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Spawn an additional TurtleBot and start its controller with an automatic namespace.'
    )
    parser.add_argument(
        '--x',
        type=float,
        default=None,
        help='Initial X position (meters). Defaults to 0.0 if not provided.',
    )
    parser.add_argument(
        '--y',
        type=float,
        default=None,
        help='Initial Y position (meters). Defaults to DEFAULT_Y_BASE + index * spacing.',
    )
    parser.add_argument(
        '--yaw',
        type=float,
        default=DEFAULT_YAW,
        help='Initial yaw (radians). Defaults to 0.0.',
    )
    parser.add_argument(
        '--spacing',
        type=float,
        default=DEFAULT_SPACING,
        help=f'Spacing applied to auto-generated Y positions (default: {DEFAULT_SPACING}).',
    )
    parser.add_argument(
        '--use-sim-time',
        type=str,
        default='true',
        choices=['true', 'false', 'True', 'False', '1', '0'],
        help='Set use_sim_time parameter (default: true).',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the computed command without executing it.',
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_cli_parser()
    args = parser.parse_args(argv)

    try:
        next_index = determine_next_index()
    except Exception as exc:
        print(f"[spawn_robot_cli] Failed to determine next namespace: {exc}", file=sys.stderr)
        return 1

    namespace = f'tb3_{next_index}'
    x_pos = args.x if args.x is not None else DEFAULT_X
    y_pos = args.y if args.y is not None else DEFAULT_Y_BASE + next_index * args.spacing
    yaw = args.yaw

    use_sim_time = str(args.use_sim_time).lower()
    if use_sim_time in ('false', '0'):
        use_sim_time = 'false'
    else:
        use_sim_time = 'true'

    cmd = [
        'ros2',
        'launch',
        'multi_robot_challenge_23',
        'spawn_robot_with_controller.launch.py',
        f'namespace:={namespace}',
        f'x:={x_pos}',
        f'y:={y_pos}',
        f'yaw:={yaw}',
        f'use_sim_time:={use_sim_time}',
    ]

    if args.dry_run:
        print(' '.join(cmd))
        return 0

    print(f"[spawn_robot_cli] Spawning new robot with namespace '{namespace}' at position ({x_pos}, {y_pos}, yaw={yaw}).")

    try:
        process = subprocess.Popen(cmd, start_new_session=True)
    except FileNotFoundError:
        print("[spawn_robot_cli] Unable to execute 'ros2'. Make sure ROS 2 environment is sourced.", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"[spawn_robot_cli] Failed to start launch process: {exc}", file=sys.stderr)
        return 1

    try:
        time.sleep(2.0)
    except KeyboardInterrupt:
        pass

    retcode = process.poll()
    if retcode is not None:
        print(f"[spawn_robot_cli] Launch process exited early with return code {retcode}. See output above.", file=sys.stderr)
        return retcode

    print(f"[spawn_robot_cli] Launch for '{namespace}' running in background (PID {process.pid}).")
    print(f"                     To stop it manually, terminate that PID (e.g. `kill {process.pid}`) or close the spawned launch output.")
    return 0


if __name__ == '__main__':
    sys.exit(main())

