"""Launch the ROSbot simulation in Gazebo Classic."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _continue_on_success(event, context, next_actions):
    """Do not leave the launch waiting on a process which has failed."""
    if context.is_shutdown:
        return []
    if event.returncode == 0:
        return next_actions
    reason = (
        f'{event.process_name} failed (exit code {event.returncode}). '
        'See the preceding Gazebo/controller error.'
    )
    return [LogInfo(msg=reason), Shutdown(reason=reason)]


def generate_launch_description():
    """
    Launch the rear-drive Ackermann model using Gazebo Classic.

    Launches:
      1. Robot State Publisher (xacro -> robot_description)
      2. Gazebo Classic with a world file (default: W0, flat ground)
      3. Spawn the robot from the robot_description topic
      4. ros2_control controller spawners (steering_cont + joint_broad)
    """
    package_name = 'rosbot_plus_description'
    pkg_share = get_package_share_directory(package_name)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'world', 'w0_flat.sdf'),
        description='Path to the Gazebo Classic world file',
    )
    world = LaunchConfiguration('world')

    # Headless by default: a heavy sim campaign (§5.2 world sweeps) runs
    # without the Gazebo client.
    # Pass gui:=true on a machine with a real display to get the 3D view.
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch the Gazebo Classic client and server',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Open RViz with the simulation displays and clock',
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'simulation.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    # Payload is forwarded straight through to the xacro, so a whole section
    # 5.1 sweep is just this launch file called with a different mass.
    payload_args = [
        DeclareLaunchArgument(
            'payload_mass', default_value='0.0',
            description='Payload mass in kg (0 = unladen, datasheet max 22)'),
        DeclareLaunchArgument(
            'payload_x', default_value='0.265',
            description='Payload x position in base_link frame'),
        DeclareLaunchArgument(
            'payload_z', default_value='0.469',
            description='Payload z position in base_link frame '
                        '(raises the CoM)'),
    ]

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true',
            'enable_lidar': LaunchConfiguration('enable_lidar'),
            'enable_camera': LaunchConfiguration('enable_camera'),
            'imu_update_rate': LaunchConfiguration('imu_update_rate'),
            'payload_mass': LaunchConfiguration('payload_mass'),
            'payload_x': LaunchConfiguration('payload_x'),
            'payload_z': LaunchConfiguration('payload_z'),
            'lidar_tilt_deg': LaunchConfiguration('lidar_tilt_deg'),
            'lidar_riser_m': LaunchConfiguration('lidar_riser_m'),
            'lidar_x_offset_m': LaunchConfiguration('lidar_x_offset_m'),
        }.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true',
            'server_required': 'true',
        }.items(),
    )

    # Spawn pose must be settable. It used to be hard-coded at the origin,
    # which silently contradicted the world metadata: a perception world says
    # to start 4.5 m before the step precisely so the near-side ground clears
    # the sensor's 2.84 m blind radius, and spawning at the origin instead
    # meant that ground was never observed. The elevation map then held only
    # the far side of the step and the C4 detector had no height contrast to
    # find - with no error anywhere to say why.
    spawn_args = [
        DeclareLaunchArgument(
            'enable_camera', default_value='true',
            description='Enable the simulated RGB-D camera'),
        DeclareLaunchArgument(
            'enable_lidar', default_value='true',
            description='Enable the simulated 2D laser'),
        DeclareLaunchArgument(
            'imu_update_rate', default_value='200',
            description='Simulated IMU update rate in Hz'),
        DeclareLaunchArgument(
            'lidar_tilt_deg', default_value='0',
            description='Downward lidar tilt in degrees; keep 0 for a '
                        'horizontal 2D scan.'),
        DeclareLaunchArgument(
            'lidar_riser_m', default_value='0.025',
            description='Additional lidar mount height in metres'),
        DeclareLaunchArgument(
            'lidar_x_offset_m', default_value='0.20',
            description='Lidar forward offset from the chassis centre, m'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.3'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
    ]

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rosbot_plus',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen',
    )

    # Gazebo Classic plugins publish ROS messages directly. In particular,
    # /clock, /scan and /mobile_base/sensors/imu_data do not need a bridge.

    # Humble's spawner loads and activates these in order. Bound the wait
    # and stop launch on failure instead of leaving the GUI and spawners up.
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_broad', 'steering_cont',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--switch-timeout', '30',
        ],
        output='screen',
    )

    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=lambda event, context: _continue_on_success(
                event, context, [controller_spawner]),
        )
    )

    check_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_spawner,
            on_exit=lambda event, context: _continue_on_success(
                event, context, []),
        )
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        rviz_arg,
        *payload_args,
        *spawn_args,
        delayed_controllers,
        check_controllers,
        rsp,
        gazebo,
        spawn_entity,
        rviz,
    ])
