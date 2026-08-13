# slam_d1_bridge

`slam_d1_bridge` converts SLAM/navigation `geometry_msgs/msg/Twist` commands into
the D1 robot's `ddt_msgs/msg/UserCommand` interface. It also subscribes to the
SLAM pose for future closed-loop use, but phase one does not use pose data to
control motion.

## Phase-one behavior

- Maps `linear.x` and `angular.z` into `UserCommand.twist`.
- Clamps both supported axes to configurable limits.
- Forces lateral and all other unsupported velocity components to zero.
- Rejects non-finite input and publishes zero velocity.
- Publishes at a fixed period and continuously publishes zero velocity after a
  command timeout.
- Caches and validates `/slam/pose` without republishing it or using it as a
  motion interlock.
- Sends a neutral D1 body pose (`orientation.w = 1.0`).

The package does not enable D1 SDK mode or perform the `transform_up -> loco`
sequence. Prepare and recover the robot using the separately validated
operational procedure. Do not run `http_ros_gateway` at the same time because
both nodes publish `command/user_command`.

## Build

```bash
cd /home/nvidia/Desktop/lunar_slam
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths project 3parts/D1-ROS2-SDK-Demo/ddt_msgs \
  --packages-up-to slam_d1_bridge
source install/setup.bash
```

## Run

All three NX devices should use the field-verified D1 domain:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch slam_d1_bridge slam_d1_bridge.launch.py \
  namespace:=d15041873
```

The launch file keeps `/cmd_vel` and `/slam/pose` global while resolving the
relative output topic to `/d15041873/command/user_command`.

For supervised physical testing, use the packaged start and stop procedures
instead of launching the bridge directly:

```bash
ros2 run slam_d1_bridge start_slam_d1_bridge.sh
ros2 run slam_d1_bridge stop_slam_d1_bridge.sh
```

Both scripts require an interactive safety confirmation. `--yes` is available
for an already supervised automation environment. The start script enables SDK
control, sends `transform_up`, switches to `loco`, and starts the bridge in the
background. The stop script stops the bridge, publishes zero velocity, sends
`transform_down`, and releases SDK control.

## Parameters

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `input_topic` | `cmd_vel` | Velocity input before launch remapping |
| `slam_pose_topic` | `slam/pose` | SLAM pose input before launch remapping |
| `output_topic` | `command/user_command` | D1 command output |
| `publish_period_ms` | `50` | Output period (20 Hz) |
| `command_timeout_ms` | `300` | Maximum age of a usable velocity command |
| `max_linear_x` | `0.5` | Absolute forward/reverse limit in m/s |
| `max_angular_z` | `0.5` | Absolute yaw-rate limit in rad/s |
| `lateral_velocity_tolerance` | `0.001` | Warning threshold for unsupported `linear.y` |
| `fsm_mode` | empty | Optional D1 FSM field; phase one does not manage FSM |

Before commanding motion, verify the physical emergency stop, SDK control mode,
robot state, clear operating area, and zero-velocity behavior.
