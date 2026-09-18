# Follow along: glossary of commands

## Environment Bring up

```bash
docker compose start
docker compose exec workshop bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
```

## UR Bringup

```bash
ros2 run tf2_ros tf2_echo base_link tool0
```

```bash
ros2 run tf2_tools view_frames
```

```bash
ros2 control list_hardware_interfaces
```

```bash
ros2 control list_controllers
```

```bash
ros2 topic pub --once -w 1 /scaled_joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
    joint_names: [
      shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
      wrist_1_joint, wrist_2_joint, wrist_3_joint
    ],
    points: [
      {
        positions: [0.5, -1.2, 1.0, -1.4, -1.57, 0.0],
        time_from_start: {sec: 4, nanosec: 0}
      }
    ]
  }"
```

```bash
ros2 topic pub --once -w 1 /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [-0.5, -1.5, 1.4, -1.5, -1.57, 0.0]}"
```

```bash
ros2 run controller_manager spawner forward_position_controller --inactive -c /controller_manager
```

```bash
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_position_controller
```

Try again:

```bash
ros2 topic pub --once -w 1 /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [-0.5, -1.5, 1.4, -1.5, -1.57, 0.0]}"
```

```bash
ros2 control switch_controllers \
  --deactivate forward_position_controller \
  --activate scaled_joint_trajectory_controller
```

```bash
ros2 topic pub --once -w 1 /scaled_joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
    joint_names: [
      shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
      wrist_1_joint, wrist_2_joint, wrist_3_joint
    ],
    points: [
      {
        positions: [-1.0, -1.5, 1.4, -1.5, -1.57, 0.0],
        time_from_start: {sec: 4, nanosec: 0}
      }
    ]
  }"
```

## Moveit Architecture

1. In `ur_macro.srdf.xacro`, add:

```xml
<group name="ur_wrist">
  <chain base_link="base_link" tip_link="wrist_3_link"/>
</group>
```

2. In `kinematics.yaml`, add:

```yaml
ur_wrist:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

3. Build again:

```bash
cbnt
```

## Planning Scene and Collisions
