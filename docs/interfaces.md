# ROS interfaces and hardware comparison

## Simulation data flow

```mermaid
flowchart LR
    Keyboard[Keyboard or joystick] -->|/cmd_vel · Twist| Control[Ackermann controller]
    Control --> Rear[Rear wheel velocity commands]
    Control --> Front[Front steering position commands]
    Gazebo[Gazebo physics] --> Joints[Joint feedback]
    Joints --> Control
    Control --> Odom[/odom]
    Control --> TF[/tf: odom to base_footprint]
    Joints --> RSP[robot_state_publisher]
    RSP --> BodyTF[/tf and /tf_static: robot links]
    Gazebo --> Sensors[LiDAR · RGB-D camera · IMU]
    Gazebo --> Truth[/ground_truth/odom: evaluation only]
```

The controller owns the planar `odom → base_footprint` transform.
`robot_state_publisher` owns the remaining robot transforms, including
`base_footprint → base_link`. Only one node should publish each transform.

Gazebo's P3D plugin reports the pose of `base_footprint` in `world` on
`/ground_truth/odom`. It does not publish a `world → odom` transform. Align the
initial poses before comparing ground truth with wheel odometry, especially
when using a nonzero spawn position or heading. Ground-truth orientation and
height include chassis motion; wheel odometry is a planar estimate.

## Command contract

- Topic: `/cmd_vel`.
- Type: `geometry_msgs/msg/Twist`.
- `linear.x`: longitudinal speed at the rear axle, in m/s.
- `angular.z`: body yaw rate, in rad/s; **not** steering angle.
- Other components are unused by the Ackermann controller.
- Timeout: 0.5 seconds in simulation time after the last received command.

Front spin joints `FL_Joint` and `FR_Joint` have state interfaces only.
`FL_steering_joint` and `FR_steering_joint` accept position commands.
`RL_Joint` and `RR_Joint` accept velocity commands.

The controller configuration uses **right, then left** joint ordering for each
axle. Both steering angles are calculated from the requested body motion.
The robot cannot execute a differential-drive turn on the spot. Command
limits and controller behavior must be checked before using an external
navigation stack.

Internal remappings in `urdf/ros2_control.xacro` expose:

| Internal controller topic | Public topic |
| --- | --- |
| `/steering_cont/reference_unstamped` | `/cmd_vel` |
| `/steering_cont/odometry` | `/odom` |
| `/steering_cont/tf_odometry` | `/tf` |

See the [Humble steering controller documentation](https://control.ros.org/humble/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html)
for the upstream controller interface. This repository does not launch a
serial driver or connect to the real robot.

## Sensor frames

| Sensor | Frame | Notes |
| --- | --- | --- |
| Planar LiDAR | `livox_frame` | Historical name; current model is a planar ray sensor |
| IMU | `imu_link` | Mounted in the chassis |
| RGB-D camera | `camera_optical_frame` | Optical convention: z forward, x right, y down |

The RGB-D sensor uses one generic optical model for RGB and depth. It does not
model a measured RGB/depth baseline, lens distortion or the complete noise
characteristics of a physical Orbbec camera. Camera point clouds can be large
and are generated when subscribed to.

## Wheeltec hardware comparison

The laboratory reported the following topic names after running:

```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

That observation contains names only. The real robot's message types, frame
IDs, QoS and units have **not yet been inspected**. Matching a topic name is
not proof of full driver compatibility.

| Reported hardware topic | Simulation status |
| --- | --- |
| `/cmd_vel` | Implemented as `geometry_msgs/msg/Twist` |
| `/odom` | Implemented as `nav_msgs/msg/Odometry`, from wheel/steering feedback |
| `/mobile_base/sensors/imu_data` | Implemented as `sensor_msgs/msg/Imu` |
| `/joint_states` | Implemented; simulated joint names may differ from hardware |
| `/robot_description` | Implemented; simplified simulated URDF |
| `/tf`, `/tf_static` | Implemented; verify hardware frame names separately |
| `/rosout`, `/parameter_events` | Standard ROS topics |
| `/ackermann_cmd` | Not implemented; message type and steering convention need confirmation |
| `/PowerVoltage` | Not implemented; no battery/electrical model |
| `/diagnostics` | No vendor-equivalent diagnostics publisher |
| `/odom_combined`, `/set_pose` | Not implemented; no localization filter is launched |
| `/robotpose`, `/robotvel` | Not implemented; message types and semantics need confirmation |

The additional `/scan`, `/camera/*`, `/clock` and `/ground_truth/odom` interfaces
belong to the simulation. They remain enabled independently of the reported
hardware topic list. No constant battery value or duplicate odometry is
published to imitate unavailable hardware interfaces.

At the next laboratory session, capture the following **read-only** information
before implementing more compatibility adapters:

```bash
ros2 topic list -t
ros2 topic info /cmd_vel --verbose
ros2 topic info /ackermann_cmd --verbose
ros2 topic info /odom --verbose
ros2 topic info /mobile_base/sensors/imu_data --verbose
ros2 topic info /robotpose --verbose
ros2 topic info /robotvel --verbose
```

Also record frame IDs, the exact Wheeltec driver revision, and whether an
Ackermann message represents wheel angle or body yaw rate. Keep simulation
experiments in a separate ROS domain when the real robot is on the same network.
