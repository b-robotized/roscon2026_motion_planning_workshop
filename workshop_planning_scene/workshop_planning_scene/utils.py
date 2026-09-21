"""Helpers for editing the MoveIt planning scene from rclpy.

Everything here goes through the /apply_planning_scene service, exactly like
the `ros2 service call` commands from the previous section. The only
difference is that Python builds the messages for us.

All three helpers call the service synchronously and return True on success.
That matters: if we only *published* the scene diff, the very next
plan request could reach move_group before the object does.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

__all__ = ["add_object", "attach_object", "detach_object"]

_SERVICE = "/apply_planning_scene"


def add_object(node: Node, object_id: str, frame_id: str, size, position) -> bool:
    """Add a box to the planning scene world.

    Args:
        node: an already-spinning-capable rclpy Node used for the service call.
        object_id: name of the object, e.g. "cube".
        frame_id: frame the pose is expressed in, e.g. "base_link".
        size: (x, y, z) box dimensions in metres.
        position: (x, y, z) of the box centre in `frame_id`.

    Returns:
        True if move_group accepted the scene update.
    """
    # TODO: build a SolidPrimitive BOX + CollisionObject (operation=ADD, pose=_pose(position)),
    # put it in _new_scene_diff().world.collision_objects and return _apply(node, scene).
    raise NotImplementedError


def attach_object(node: Node, object_id: str, link_name: str, touch_links) -> bool:
    """Attach an object already in the world to a robot link.

    The object is removed from the world and becomes part of the robot, so
    collisions between it and `touch_links` are ignored.

    Args:
        node: rclpy Node used for the service call.
        object_id: name of the object to attach, e.g. "cube".
        link_name: link to attach it to, e.g. "tool0".
        touch_links: links allowed to touch the object, e.g. the gripper links.

    Returns:
        True if move_group accepted the scene update.
    """
    # TODO: build an AttachedCollisionObject (link_name, touch_links, object.id, object.operation=ADD),
    # put it in _new_scene_diff().robot_state.attached_collision_objects and return _apply(node, scene).
    raise NotImplementedError


def detach_object(node: Node, object_id: str, link_name: str) -> bool:
    """Detach an object from a robot link and put it back into the world.

    Args:
        node: rclpy Node used for the service call.
        object_id: name of the attached object, e.g. "cube".
        link_name: link it is currently attached to, e.g. "tool0".

    Returns:
        True if move_group accepted the scene update.
    """
    # TODO: same as attach_object but with object.operation=REMOVE and no touch_links.
    raise NotImplementedError


# --- private helpers -------------------------------------------------------

def _pose(position) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    pose.orientation.w = 1.0
    return pose


def _new_scene_diff() -> PlanningScene:
    """A PlanningScene that only *changes* things instead of replacing everything."""
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    return scene


def _apply(node: Node, scene: PlanningScene, timeout_sec: float = 5.0) -> bool:
    """Call /apply_planning_scene synchronously and return its success flag."""
    client = node.create_client(ApplyPlanningScene, _SERVICE)
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().error(f"{_SERVICE} not available -- is move_group running?")
            return False
        future = client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if future.result() is None:
            node.get_logger().error(f"{_SERVICE} call timed out")
            return False
        return future.result().success
    finally:
        node.destroy_client(client)
