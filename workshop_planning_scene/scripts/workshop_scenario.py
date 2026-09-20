#!/usr/bin/env python3
"""Section 4: build the workshop scene, then run a pick-and-place loop.

The scene is two tables separated by a dividing wall, with a small cube
sitting on table A. The robot picks the cube off A, carries it over the
divider to B, drops it, and goes home. Nothing here tells the arm *how* to
get over the wall -- that is the planner's job, and watching it solve that is
the point of the exercise.

All message building lives in utils.py.

Usage:
    ros2 run workshop_planning_scene workshop_scenario.py
    ros2 run workshop_planning_scene workshop_scenario.py --scene-only

Needs move_group running and scaled_joint_trajectory_controller active:
    ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node

from workshop_planning_scene.utils import (
    BASE_FRAME,
    EE_LINK,
    HOME_JOINTS,
    READY_JOINTS,
    add_object,
    allow_collision,
    attach_object,
    detach_object,
    move_to_joints,
    move_to_pose,
)

# --- scene layout, all in base_link --------------------------------------
#
# +x is straight out in front of the robot (base_link is identity with world,
# and at shoulder_pan = 0 the arm reaches along +x). Both tables sit in front,
# side by side, with a low wall on the centreline between them.
#
#            +y  table B  ][  divider at y = 0
#             |
#   base  ----+-----------> +x
#             |
#            -y  table A  <- cube starts here

# Which way the whole scene faces, in degrees about the base z axis. 0 puts it
# along +x, the direction the arm reaches at shoulder_pan = 0. Use 90, 180 or
# 270 to swing it round; only right angles are supported, because the boxes
# are axis-aligned and rotating them just swaps their x and y extents.
SCENE_YAW_DEG = 0

CUBE_ID = "cube"
CUBE_EDGE = 0.05

TABLE_SIZE = (0.34, 0.28, 0.04)
TABLE_TOP_Z = 0.20                      # height of the working surface
TABLE_CENTRE_Z = TABLE_TOP_Z - TABLE_SIZE[2] / 2
TABLE_X = 0.45
TABLE_A_Y = -0.22
TABLE_B_Y = 0.22

# Only as long as the tables are deep, and low enough to lift over
# comfortably. Making it longer or taller walls the arm in and OMPL starts
# failing to find any path at all.
DIVIDER_SIZE = (0.34, 0.03, 0.12)
DIVIDER_CENTRE_Z = TABLE_TOP_Z + DIVIDER_SIZE[2] / 2

# The cube rests on table A.
CUBE_Z = TABLE_TOP_Z + CUBE_EDGE / 2

# tool0 sits this far above the cube centre when picking. No gripper is
# modelled, so "grasping" is just parking the flange above the cube and
# attaching it -- the offset is what keeps the two from overlapping.
GRASP_OFFSET_Z = 0.075
GRASP_Z = CUBE_Z + GRASP_OFFSET_Z



# Height to lift to before crossing. The attached cube hangs GRASP_OFFSET_Z +
# half an edge below tool0, so a straight A -> B move at grasp height would
# drag it through the divider. Going over is not optional, and leaving it to
# the planner is fragile: OMPL does find a path, but trajectory smoothing
# shaves the corner and the result fails validation against the wall.
TRANSIT_Z = 0.50

# Robot links the cube may touch once attached.
TOUCH_LINKS = [EE_LINK, "wrist_3_link", "wrist_2_link"]

# The cube rests on a table at both ends of the journey, and an attached
# object touching a world object is a collision. touch_links cannot express
# this -- it only covers robot links -- so the tables go in the ACM instead.
SUPPORTS = ["table_a", "table_b"]


def _place(x, y):
    """Rotate a layout point about the base z axis by SCENE_YAW_DEG."""
    c = math.cos(math.radians(SCENE_YAW_DEG))
    s = math.sin(math.radians(SCENE_YAW_DEG))
    return (c * x - s * y, s * x + c * y)


def _at(x, y, z):
    """A scene position, rotated into place."""
    rx, ry = _place(x, y)
    return (rx, ry, z)


def _span(dx, dy, dz):
    """Box extents, swapped if the scene is turned through a quarter turn."""
    return (dx, dy, dz) if SCENE_YAW_DEG % 180 == 0 else (dy, dx, dz)


def build_scene(node: Node) -> bool:
    """Add the ground, both tables, the divider and the cube.

    Safe to re-run: adding an object that already exists replaces it, so this
    doubles as a reset. The cube is detached first in case a previous run died
    while holding it -- otherwise the same id would exist both as an attached
    body and a world object.
    """
    detach_object(node, CUBE_ID, EE_LINK)
    ok = True
    ok &= add_object(
        node, object_id="ground", frame_id=BASE_FRAME,
        size=(2.0, 2.0, 0.02), position=(0.0, 0.0, -0.05),
    )
    ok &= add_object(
        node, object_id="table_a", frame_id=BASE_FRAME,
        size=_span(*TABLE_SIZE), position=_at(TABLE_X, TABLE_A_Y, TABLE_CENTRE_Z),
    )
    ok &= add_object(
        node, object_id="table_b", frame_id=BASE_FRAME,
        size=_span(*TABLE_SIZE), position=_at(TABLE_X, TABLE_B_Y, TABLE_CENTRE_Z),
    )
    ok &= add_object(
        node, object_id="divider", frame_id=BASE_FRAME,
        size=_span(*DIVIDER_SIZE), position=_at(TABLE_X, 0.0, DIVIDER_CENTRE_Z),
    )
    ok &= add_object(
        node, object_id=CUBE_ID, frame_id=BASE_FRAME,
        size=(CUBE_EDGE, CUBE_EDGE, CUBE_EDGE),
        position=_at(TABLE_X, TABLE_A_Y, CUBE_Z),
    )
    # Must come after the objects exist, so the names resolve.
    ok &= allow_collision(node, CUBE_ID, SUPPORTS)
    node.get_logger().info("scene built" if ok else "scene FAILED")
    return ok


def run_demo(node: Node) -> bool:
    """Cube on A -> attach -> over the divider to B -> detach -> home."""
    log = node.get_logger()

    grasp_on_a = _at(TABLE_X, TABLE_A_Y, GRASP_Z)
    place_on_b = _at(TABLE_X, TABLE_B_Y, GRASP_Z)
    above_a = _at(TABLE_X, TABLE_A_Y, TRANSIT_Z)
    above_b = _at(TABLE_X, TABLE_B_Y, TRANSIT_Z)

    # The sim starts the arm straight up, which is singular (elbow and wrist_2
    # both at 0). Pose goals planned out of that make RRTConnect time out every
    # so often; a joint-space goal needs no IK, so this always works.
    log.info("0/3 moving to a well-conditioned start configuration")
    if not move_to_joints(node, READY_JOINTS):
        return False

    log.info("1/3 picking the cube up off table A")
    if not move_to_pose(node, above_a):
        return False
    if not move_to_pose(node, grasp_on_a):
        return False
    if not attach_object(node, CUBE_ID, EE_LINK, TOUCH_LINKS):
        return False
    if not move_to_pose(node, above_a):
        return False

    log.info("2/3 carrying it over the divider to table B")
    if not move_to_pose(node, above_b):
        return False
    if not move_to_pose(node, place_on_b):
        return False
    if not detach_object(node, CUBE_ID, EE_LINK):
        return False
    if not move_to_pose(node, above_b):
        return False

    log.info("3/3 going home")
    if not move_to_joints(node, HOME_JOINTS):
        return False

    log.info("demo done")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene-only", action="store_true",
        help="build the scene and exit, without moving the robot",
    )
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node("workshop_scenario")
    try:
        ok = build_scene(node)
        if ok and not args.scene_only:
            ok = run_demo(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
