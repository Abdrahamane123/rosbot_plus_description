"""Run FAST-LIO2 against the simulated Livox Mid-360.

    ros2 launch rosbot_plus_description sim.launch.py          # terminal 1
    ros2 launch rosbot_plus_description fastlio_sim.launch.py  # terminal 2

Thin wrapper around FAST_LIO's own mapping.launch.py, pointed at
config/fastlio_sim.yaml in THIS package. FAST_LIO stays an unmodified upstream
checkout, so it can be updated with a plain `git pull` and nothing of ours is
lost in the merge.

RViz is off by default because simulation campaigns normally run headless.
Pass rviz:=true from a terminal with a display to watch the map build.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('rosbot_plus_description')
    fast_lio_share = get_package_share_directory('fast_lio')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz. Needs a display; run it from a normal '
                        'terminal, not the VS Code snap one.'),
        DeclareLaunchArgument(
            'config_file', default_value='fastlio_sim.yaml',
            description='Config in this package\'s config/ directory. Use '
                        'the real robot config when running on hardware.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')),
            launch_arguments={
                # Sim time is not optional here: FAST-LIO fuses lidar and IMU
                # on timestamps, and mixing wall clock with the Gazebo clock makes
                # every scan look arbitrarily stale.
                'use_sim_time': 'true',
                'config_path': os.path.join(pkg_share, 'config'),
                'config_file': LaunchConfiguration('config_file'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
