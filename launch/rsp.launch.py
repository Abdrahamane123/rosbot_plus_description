"""Publish the ROSbot description and its fixed transforms."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from rosbot_plus_description.description import generate_robot_description


def _robot_state_publisher(context):
    """Resolve arguments before expanding xacro, without a shell command."""
    pkg_share = get_package_share_directory('rosbot_plus_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rosbot_plus.xacro')
    mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            'use_ros2_control', 'payload_mass', 'payload_x', 'payload_z',
            'enable_lidar', 'enable_camera', 'imu_update_rate', 'lidar_tilt_deg',
            'lidar_riser_m', 'lidar_x_offset_m',
        )
    }
    mappings['sim_mode'] = LaunchConfiguration('use_sim_time').perform(context)
    robot_description = generate_robot_description(xacro_file, mappings)
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description, value_type=str),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }]
    )]


def generate_launch_description():
    """Launch Robot State Publisher for ROSbot Plus 4WD-IS."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        # Payload: the primary swept variable of the doc's section 5.1
        # experiment. Both mass and height are exposed, because a heavy low
        # load and a heavy high load fail in different ways (traction loss vs
        # tipping). Datasheet maximum is 22 kg.
        DeclareLaunchArgument(
            'payload_mass',
            default_value='0.0',
            description='Payload mass in kg (0 = unladen, datasheet max 22)'),
        DeclareLaunchArgument(
            'payload_x',
            default_value='0.265',
            description='Payload x position in base_link frame'),
        DeclareLaunchArgument(
            'payload_z',
            default_value='0.469',
            description='Payload z position in base_link frame '
                        '(raises the CoM)'),
        DeclareLaunchArgument(
            'enable_camera', default_value='true',
            description='Enable the simulated RGB-D camera'),
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='true',
            description='false disables the simulated 2D laser'),
        DeclareLaunchArgument(
            'imu_update_rate',
            default_value='200',
            description='Hz. 200 = real Mid-360 IMU. Raise to ~1000 only for '
                        'impact-severity campaigns; see params.xacro'),
        DeclareLaunchArgument(
            'lidar_tilt_deg',
            default_value='0',
            description='Downward lidar tilt in degrees; keep 0 for a '
                        'horizontal 2D scan.'),
        # Tilt, riser and forward offset are ONE mount, and passing a tilt
        # without the other two reproduces the failure the bracket exists to
        # fix: the beam goes into the robot's own roof. The printed bracket
        # (hardware/lidar_mount.py) is tilt 25, riser 0.040, offset 0.331.
        DeclareLaunchArgument(
            'lidar_riser_m',
            default_value='0.025',
            description='Extra lidar height above the chassis roof, m.'),
        DeclareLaunchArgument(
            'lidar_x_offset_m',
            default_value='0.20',
            description='Forward offset of the lidar from the chassis centre, '
                        'm. 0.331 puts it at the front edge, which is what '
                        'actually removes the self-occlusion.'),
        OpaqueFunction(function=_robot_state_publisher),
    ])
