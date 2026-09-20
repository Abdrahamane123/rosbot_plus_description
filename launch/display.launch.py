import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rosbot_plus_description.description import generate_robot_description


def generate_launch_description():
    """RViz display launch file for ROSbot Plus 4WD-IS (no simulation)."""

    pkg_share = get_package_share_directory('rosbot_plus_description')

    xacro_file = os.path.join(pkg_share, 'urdf', 'rosbot_plus.xacro')
    robot_urdf = generate_robot_description(
        xacro_file, mappings={'use_ros2_control': 'false', 'sim_mode': 'false'})

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Show joint_state_publisher_gui',
    )
    show_gui = LaunchConfiguration('gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
