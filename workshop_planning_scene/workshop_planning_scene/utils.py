"""Helpers for driving MoveIt from rclpy.

Two halves:

* Scene editing -- `add_object`, `attach_object`, `detach_object` go through
  the /apply_planning_scene service, exactly like the `ros2 service call`
  commands from the previous section. The only difference is that Python
  builds the messages for us.
* Motion -- `move_to_pose` and `move_to_joints` send a MotionPlanRequest to
  the /move_action action server and wait for execution to finish.

Every helper is synchronous and returns True on success. That matters: if we
only *published* a scene diff, the very next plan request could reach
move_group before the object does.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PlanningSceneComponents,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive

__all__ = [
    "add_object",
    "attach_object",
    "detach_object",
    "allow_collision",
    "move_to_pose",
    "move_to_joints",
    "BASE_FRAME",
    "EE_LINK",
    "HOME_JOINTS",
    "READY_JOINTS",
    "PLANNING_GROUP",
    "TOOL_DOWN",
]

_SERVICE = "/apply_planning_scene"
_MOVE_ACTION = "/move_action"
_GET_SCENE = "/get_planning_scene"

#: Planning group and tip link from ur_moveit_config's SRDF.
PLANNING_GROUP = "ur_manipulator"
EE_LINK = "tool0"
BASE_FRAME = "base_link"

#: (x, y, z, w) -- 180 deg about base_link X, i.e. tool0's z axis points down.
#: This is the orientation you want for reaching down at something on a table.
TOOL_DOWN = (1.0, 0.0, 0.0, 0.0)

#: A well-conditioned configuration to plan *from*. The sim starts the arm
#: straight up, which is singular (elbow and wrist_2 both at 0); pose goals
#: planned from there make RRTConnect time out intermittently. A joint-space
#: goal needs no IK, so getting here always works, and everything after it
#: starts from a sane place.
READY_JOINTS = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.57,
    "elbow_joint": 1.57,
    "wrist_1_joint": -1.57,
    "wrist_2_joint": -1.57,
    "wrist_3_joint": 0.0,
}

#: The `home` group_state from ur_macro.srdf.xacro.
HOME_JOINTS = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.5707,
    "elbow_joint": 0.0,
    "wrist_1_joint": 0.0,
    "wrist_2_joint": 0.0,
    "wrist_3_joint": 0.0,
}


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
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [float(s) for s in size]

    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = object_id
    # `operation` is a `byte` field. That is why the CLI needed the
    # `!!binary` base64 form -- a plain integer gets coerced to zeros.
    obj.operation = CollisionObject.ADD
    obj.primitives = [box]
    # Placement lives in obj.pose; the primitive sits at the object origin.
    obj.pose = _pose(position)
    obj.primitive_poses = [_pose((0.0, 0.0, 0.0))]

    scene = _new_scene_diff()
    scene.world.collision_objects = [obj]
    return _apply(node, scene)


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
    attached = AttachedCollisionObject()
    attached.link_name = link_name
    attached.touch_links = list(touch_links)
    attached.object.id = object_id
    attached.object.operation = CollisionObject.ADD  # byte field, see add_object

    scene = _new_scene_diff()
    scene.robot_state.attached_collision_objects = [attached]
    return _apply(node, scene)


def detach_object(node: Node, object_id: str, link_name: str) -> bool:
    """Detach an object from a robot link and put it back into the world.

    Args:
        node: rclpy Node used for the service call.
        object_id: name of the attached object, e.g. "cube".
        link_name: link it is currently attached to, e.g. "tool0".

    Returns:
        True if move_group accepted the scene update.
    """
    attached = AttachedCollisionObject()
    attached.link_name = link_name
    attached.object.id = object_id
    attached.object.operation = CollisionObject.REMOVE  # byte field, see add_object

    scene = _new_scene_diff()
    scene.robot_state.attached_collision_objects = [attached]
    return _apply(node, scene)



def allow_collision(node: Node, object_id: str, others) -> bool:
    """Let `object_id` touch each of `others` without it counting as a collision.

    `AttachedCollisionObject.touch_links` only covers *robot links* -- it never
    reaches the Allowed Collision Matrix, so an attached object resting on a
    world object (a cube on a table) still reports a collision and planning
    fails with START_STATE_IN_COLLISION. Permission between two named world
    objects has to go into the ACM directly.

    The current matrix is read back and extended rather than overwritten:
    sending a fresh ACM in a scene diff *replaces* the whole thing, which would
    throw away the self-collision pairs the SRDF disabled.

    Args:
        node: rclpy Node used for the service calls.
        object_id: first body, e.g. "cube".
        others: iterable of body names it may touch, e.g. ["table_a"].

    Returns:
        True if move_group accepted the scene update.
    """
    client = node.create_client(GetPlanningScene, _GET_SCENE)
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(f"{_GET_SCENE} not available")
            return False
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is None:
            node.get_logger().error(f"{_GET_SCENE} call timed out")
            return False
        acm = future.result().scene.allowed_collision_matrix
    finally:
        node.destroy_client(client)

    names = list(acm.entry_names)
    rows = [list(row.enabled) for row in acm.entry_values]
    for name in [object_id, *others]:
        if name not in names:
            names.append(name)
            for row in rows:
                row.append(False)
            rows.append([False] * len(names))

    i = names.index(object_id)
    for other in others:
        j = names.index(other)
        rows[i][j] = True
        rows[j][i] = True

    acm.entry_names = names
    acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]

    scene = _new_scene_diff()
    scene.allowed_collision_matrix = acm
    return _apply(node, scene)


# --- motion ----------------------------------------------------------------

def move_to_pose(
    node: Node,
    position,
    orientation=TOOL_DOWN,
    frame_id: str = BASE_FRAME,
    ee_link: str = EE_LINK,
    group: str = PLANNING_GROUP,
    position_tolerance: float = 0.01,
    orientation_tolerance: float = 0.1,
    velocity_scaling: float = 0.2,
    planning_time: float = 20.0,
    timeout_sec: float = 120.0,
) -> bool:
    """Plan and execute a Cartesian pose goal for `ee_link`.

    A pose goal is really two constraints: keep the link's origin inside a
    small sphere, and keep its orientation near a target quaternion. MoveIt's
    C++ API hides this behind setPoseTarget(); from rclpy we build them.

    Args:
        node: rclpy Node used for the action call.
        position: (x, y, z) target for `ee_link`'s origin, in `frame_id`.
        orientation: (x, y, z, w) target quaternion. Defaults to TOOL_DOWN.
        frame_id: frame the goal is expressed in, e.g. "base_link".
        ee_link: link being positioned, e.g. "tool0".
        group: planning group name.
        position_tolerance: radius of the goal sphere, in metres.
        orientation_tolerance: per-axis angular tolerance, in radians.
            Tight values make the goal sampler work harder for no benefit here.
        velocity_scaling: fraction of max joint speed/acceleration, 0..1.
        planning_time: seconds the planner may spend per attempt.
        timeout_sec: give up if planning *and* execution take longer.

    Returns:
        True if the motion planned and executed successfully.
    """
    region = SolidPrimitive()
    region.type = SolidPrimitive.SPHERE
    region.dimensions = [float(position_tolerance)]

    pos = PositionConstraint()
    pos.header.frame_id = frame_id
    pos.link_name = ee_link
    pos.constraint_region = BoundingVolume(
        primitives=[region], primitive_poses=[_pose(position)]
    )
    pos.weight = 1.0

    orient = OrientationConstraint()
    orient.header.frame_id = frame_id
    orient.link_name = ee_link
    (
        orient.orientation.x,
        orient.orientation.y,
        orient.orientation.z,
        orient.orientation.w,
    ) = (float(v) for v in orientation)
    orient.absolute_x_axis_tolerance = float(orientation_tolerance)
    orient.absolute_y_axis_tolerance = float(orientation_tolerance)
    orient.absolute_z_axis_tolerance = float(orientation_tolerance)
    orient.weight = 1.0

    request = _new_request(group, velocity_scaling, planning_time)
    request.goal_constraints = [
        Constraints(position_constraints=[pos], orientation_constraints=[orient])
    ]
    return _run(node, request, timeout_sec)


def move_to_joints(
    node: Node,
    joints,
    group: str = PLANNING_GROUP,
    tolerance: float = 0.01,
    velocity_scaling: float = 0.2,
    planning_time: float = 20.0,
    timeout_sec: float = 120.0,
) -> bool:
    """Plan and execute a joint-space goal.

    Use this for named configurations such as HOME_JOINTS. Joint goals need no
    IK, so they succeed in places a pose goal cannot reach -- including the
    singular `home` pose, which has no well-conditioned Cartesian equivalent.

    Args:
        node: rclpy Node used for the action call.
        joints: mapping of joint name -> target position in radians.
        group: planning group name.
        tolerance: per-joint tolerance in radians.
        velocity_scaling: fraction of max joint speed/acceleration, 0..1.
        planning_time: seconds the planner may spend per attempt.
        timeout_sec: give up if planning *and* execution take longer.

    Returns:
        True if the motion planned and executed successfully.
    """
    constraints = Constraints()
    for name, value in joints.items():
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(value)
        jc.tolerance_above = float(tolerance)
        jc.tolerance_below = float(tolerance)
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)

    request = _new_request(group, velocity_scaling, planning_time)
    request.goal_constraints = [constraints]
    return _run(node, request, timeout_sec)


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


def _new_request(group: str, velocity_scaling: float, planning_time: float) -> MotionPlanRequest:
    """A MotionPlanRequest with everything filled in except the goal."""
    request = MotionPlanRequest()
    request.group_name = group
    request.num_planning_attempts = 10
    request.allowed_planning_time = float(planning_time)
    request.max_velocity_scaling_factor = float(velocity_scaling)
    request.max_acceleration_scaling_factor = float(velocity_scaling)
    # Leaving the workspace at its all-zero default gives the planner a
    # zero-volume box to sample in, which some planners refuse outright.
    request.workspace_parameters.header.frame_id = BASE_FRAME
    request.workspace_parameters.min_corner.x = -1.5
    request.workspace_parameters.min_corner.y = -1.5
    request.workspace_parameters.min_corner.z = -1.5
    request.workspace_parameters.max_corner.x = 1.5
    request.workspace_parameters.max_corner.y = 1.5
    request.workspace_parameters.max_corner.z = 1.5
    return request


def _run(node: Node, request: MotionPlanRequest, timeout_sec: float) -> bool:
    """Send a MotionPlanRequest to /move_action and wait for execution."""
    client = ActionClient(node, MoveGroup, _MOVE_ACTION)
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error(f"{_MOVE_ACTION} not available -- is move_group running?")
            return False

        goal = MoveGroup.Goal()
        goal.request = request
        # plan_only=False means move_group also hands the trajectory to the
        # controller. That needs scaled_joint_trajectory_controller active.
        goal.planning_options.plan_only = False
        # Plan against the live scene rather than replacing it with an empty one.
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=timeout_sec)
        handle = sent.result()
        if handle is None:
            node.get_logger().error("timed out handing the goal to move_group")
            return False
        if not handle.accepted:
            node.get_logger().error("move_group rejected the goal")
            return False

        completed = handle.get_result_async()
        rclpy.spin_until_future_complete(node, completed, timeout_sec=timeout_sec)
        outcome = completed.result()
        if outcome is None:
            node.get_logger().error("timed out waiting for the motion to finish")
            return False

        code = outcome.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            node.get_logger().error(f"motion failed, MoveItErrorCodes.val={code}")
            return False
        return True
    finally:
        client.destroy()
