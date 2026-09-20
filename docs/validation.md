# Validation status

This report describes checks performed on **20 September 2026**, using Ubuntu
22.04, ROS 2 Humble and Gazebo Classic 11.10.2. The package was built into an
isolated temporary workspace and launched with `ROS_DOMAIN_ID=91`, local-only
ROS discovery, and a separate Gazebo master. No real robot was controlled.

## Checks completed

| Check | Result |
| --- | --- |
| Package build | Passed with colcon |
| Description tests | 9 passed: XML/ROS parsing, plugins, joints, payload, sensors, interfaces |
| Launch and controllers | Robot spawned; `joint_broad` and `steering_cont` activated |
| Velocity input | `geometry_msgs/msg/Twist` on `/cmd_vel`; short straight and left-turn commands exercised |
| Odometry separation | One publisher on `/odom`, frame `odom`; separate P3D publisher on `/ground_truth/odom`, frame `world` |
| TF | `odom → base_footprint` observed on `/tf`, along with moving robot joints |
| IMU | Messages received on `/mobile_base/sensors/imu_data` |
| LiDAR | 400 samples per scan; 250 finite returns in the obstacle scene in one observed scan |
| RGB and depth | 640 × 480 images; `rgb8` and `32FC1`; `camera_optical_frame` |
| RViz | Launched with OpenGL after removing inherited Snap GTK variables for that process |

During the interface probe, about 15 simulated seconds of messages were
observed. These observations verify topic delivery and basic controller
response; they do not establish calibrated sensor accuracy or complete
controller behavior over the operating envelope.

## Motion observations

A separate ten-second simulated sequence used an initial stop, two seconds at
0.2 m/s, a stop, two seconds at 0.2 m/s with 0.1 rad/s left yaw rate, then a stop.
Ground truth advanced along positive x. During the left turn, the measured
front steering positions were approximately 0.227 rad on the right and
0.301 rad on the left, consistent with the inside wheel turning further.
The final heading was approximately 0.17 rad left of the initial heading.

These are smoke-test observations, not tracking-error specifications. A later media recording also exercised reverse motion. Right turns,
full steering travel, timeout behavior under publisher loss,
payload variation and obstacle traversal need systematic measurement.

## Open physical-model issues

**The suspension is not yet validated for research conclusions.** In the idle
probe, all four suspension positions were around +0.0300 m, at the upper travel
limit, instead of the expected unloaded spring sag. This requires investigation
of ODE spring/damper behavior and the assembled joint model before using
suspension travel to infer payload or terrain capability.

A small residual horizontal drift was also observed. From approximately 10.178
to 24.178 simulated seconds, x moved from −0.019414 m to −0.023569 m: about
4.15 mm over 14 seconds, after the initial spawn transient. This interval did
not show the large rotations and jumping reported earlier, but it does not
prove zero drift or stability for all loads/worlds.

The real robot's wheelbase, steering limits, inertias, suspension rates and
sensor extrinsics have not been measured here. Match topic names and control
signs first; calibrate physical and optical parameters before sim-to-real work.

## Scope and reproducibility

- The checked world was `demo_obstacles.sdf`, zero added payload and default spawn.
- Both LiDAR and RGB-D camera were enabled.
- Physics used ODE, 0.2 ms steps and 100 solver iterations.
- The source test suite for the legacy elevation-map module was not executed
  successfully: this environment lacks `grid_map_msgs`. It is separate from
  the nine passing description tests.
- GUI capture through the desktop returned black images under this Wayland
  environment. Those failed captures are not included in the documentation.
  Demonstration media uses actual Gazebo camera output instead.
- No hardware comparison was possible; only the Wheeltec topic names supplied
  by the laboratory were available.

For a repeatable evaluation, use the [experiment protocol](experiments.md) and
record the exact revision and configuration alongside results.

## Demonstration media

`media/gazebo-demo.gif` and `media/gazebo-demo.webm` show a separate 14-second
simulated sequence (forward, left turn, reverse, stop), captured by a temporary
fixed observer camera at pose `2.7 -3.2 1.8 0 0.39 2.32`. The 49 sampled frames
play at 5 fps, so playback lasts 9.8 seconds. GIF encoding merges identical
frames while preserving their duration. Use recorded ROS timestamps, not this
illustrative video's timing, for physical measurements.

`media/camera-demo.png` is an actual message from the robot's RGB camera in the
obstacle scene. `media/gazebo-demo.png` is the first overview frame. No desktop
content, manufacturer photographs or generated imagery is included.
