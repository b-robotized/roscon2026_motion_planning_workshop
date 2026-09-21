#!/usr/bin/env python3
"""Section 4: add a cube to the planning scene and attach it to the tool.

Same result as the `ros2 service call /apply_planning_scene` commands from the
glossary, done from Python. All message building lives in utils.py.

Usage:
    ros2 run workshop_planning_scene workshop_scenario.py
"""

import sys

import rclpy
from rclpy.node import Node

from workshop_planning_scene.utils import add_object, attach_object


def main() -> int:
    rclpy.init()
    node = Node("workshop_scenario")
    ok = True
    try:
        ok &= add_object(
            node,
            object_id="cube",
            frame_id="base_link",
            size=(0.05, 0.05, 0.05),
            position=(0.5, 0.2, 0.3),
        )
        ok &= attach_object(
            node,
            object_id="cube",
            link_name="tool0",
            touch_links=["tool0", "wrist_3_link", "wrist_2_link"],
        )
        node.get_logger().info("scenario done" if ok else "scenario FAILED")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
