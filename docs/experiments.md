# Reproducible experiments and media

## Record the setup

Use the same ROS domain in all terminals for one experiment. If a real robot
shares the network, use a separate simulation domain, for example:

```bash
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
```

Save the repository revision if available, ROS and Gazebo versions, world,
spawn pose, payload mass and position, sensor flags, physics parameters and
controller configuration. State whether time intervals refer to simulated or
wall-clock seconds. A 0.2 ms physics step with camera rendering may run slower
than real time.

The obstacle world is intended for sensor demonstrations, not as a calibrated
terrain benchmark. Use W0 for baseline checks, then change one condition at a
time for comparative experiments.

## Baseline protocol

1. Start `sim.launch.py` without command publishers. Wait for both controllers
   to become active and for the spawn transient to settle.
2. Record at least 30 seconds of **simulation time** at rest. Measure horizontal
   displacement, roll/pitch and suspension travel from ground truth and joint
   states. Choose acceptance thresholds before comparing runs.
3. Send a short, slow straight command, then stop. Check wheel direction,
   displacement sign, odometry and command timeout.
4. Repeat with a gentle left and right turn, then reverse. Check steering signs
   and compare wheel odometry with the aligned ground-truth trajectory.
5. Repeat the sequence with the intended payload and world. Archive failures as
   well as successful trials.

A basic straight-motion input, stopped using `Ctrl+C`, is:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

Then publish zero explicitly:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

These commands target whatever robot subscribes in the selected ROS domain.
For simulation experiments, use the isolated domain described above.

## Record a ROS bag

Keep bags outside the source package. Choose a new output directory for each run:

```bash
mkdir -p ~/dev_ws/recordings
ros2 bag record -o ~/dev_ws/recordings/baseline_001 \
  /clock /cmd_vel /odom /ground_truth/odom \
  /mobile_base/sensors/imu_data /joint_states /tf /tf_static /scan
```

Add `/camera/image_raw`, `/camera/depth/image_raw` and their `camera_info` topics
when required; image recording increases storage and CPU use. Stop the recorder
with `Ctrl+C` and inspect `ros2 bag info ~/dev_ws/recordings/baseline_001`.
The bag is the measurement artifact; a demonstration video is not a substitute.

For playback, stop the simulation first and launch visualization only with
simulation time enabled. Playing a bag alongside live publishers can mix
clocks, transforms and odometry.

## Screenshots

Launch the obstacle demonstration from the README with `gui:=true rviz:=true`.
For an informative screenshot:

- Show the entire robot, front steering, camera and forward LiDAR mount.
- Include at least one obstacle and the corresponding scan points in RViz.
- Use a readable camera distance and keep the robot near the image centre.
- Capture the application window rather than the whole desktop.

The images in [media/](media/) are captured from this simulation, not generated
illustrations or manufacturer CAD. See [validation.md](validation.md) for their
capture context. Keep labels explicit when a view shows sensor output rather
than the full robot.

## Short demonstration video

A useful 20–30 second sequence contains: an idle view, a slow straight segment,
a gentle turn showing the front wheels, and the RGB/laser displays. Use the
Ubuntu screen recorder or another installed recorder to capture the simulation
window. Label the video with the world, payload and software revision.

If FFmpeg is installed, convert an existing capture to a browser-friendly MP4:

```bash
ffmpeg -i capture.webm -vf 'scale=1280:-2' \
  -c:v libx264 -crf 23 -pix_fmt yuv420p -an demo.mp4
```

Upload large recordings to a project release or lab storage and link them from
the README. Only add a link after the file exists. The README embeds a GIF and links a WebM demonstration rendered from a
temporary observer camera in Gazebo. Both show the same recorded sequence;
`media/capture.json` describes the capture.

## Results to report

| Quantity | Source / method |
| --- | --- |
| Idle drift | Ground-truth displacement over a stated interval after settling |
| Odometry error | Align initial poses, then compare `/odom` with ground truth |
| Steering response | Front joint positions and commanded velocity/yaw rate |
| Suspension response | Four suspension positions/velocities under known loads |
| Sensor availability | Message counts, simulated timestamps, frame IDs and QoS |
| Runtime performance | Simulated duration divided by wall-clock duration |

Do not use the current suspension as a validated load estimator or claim a
step-climbing capability before addressing the limitations in the validation
report and comparing against measurements from the laboratory robot.
