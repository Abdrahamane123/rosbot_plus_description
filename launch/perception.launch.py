"""Full perception stack on the simulated Mid-360, in one launch.

    ros2 launch rosbot_plus_description sim.launch.py world:=<world with landmarks>
    ros2 launch rosbot_plus_description perception.launch.py

Chain:  /mid360/points -> cloud_sanitize -> /mid360/points_clean
                       -> FAST-LIO2      -> /Odometry, /cloud_registered
                       -> elevation_map  -> /elevation_map (grid_map_msgs)

Starting the three nodes together rather than by hand matters more than it
looks. Doing it in separate shells needs each one to wait for the previous
one's topic, and polling with `ros2 topic list` in a loop is what wedged the
ros2 CLI during bring-up - the DDS graph stayed perfectly healthy while every
`ros2 topic list` hung. Launch handles the ordering without polling.

Remember the world needs landmarks (`generate_world ... --landmarks 16`): on
bare discontinuity worlds FAST-LIO does not track at all. See the perception
section of discontinuity_worlds/README.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rosbot_plus_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz (needs a display, and a non-snap terminal)'),
        DeclareLaunchArgument(
            'detector', default_value='true',
            description='Run the C4 discontinuity detector'),
        # Which pose the elevation map accumulates against.
        #
        # /Odometry (FAST-LIO) is what the real robot will have, so it is the
        # default. But for a SENSOR-GEOMETRY experiment - sweeping the mount
        # tilt, say - LIO drift is a confound, not the subject: each run drifts
        # differently, which moves the map, changes which cells fill, and drags
        # landmarks into the corridor being measured. A tilt sweep run this way
        # produced non-monotonic coverage and detections that appeared and
        # vanished between angles, measuring the drift rather than the mount.
        # Point this at /ground_truth/odom to remove that term.
        DeclareLaunchArgument(
            'map_odom_topic', default_value='/Odometry',
            description='pose source for the elevation map. Use '
                        '/ground_truth/odom for sensor-geometry experiments, '
                        'where LIO drift would otherwise dominate the result.'),
        DeclareLaunchArgument(
            'resolution', default_value='0.05',
            description='Elevation map cell size in m. 0.05 puts ~2 cells '
                        'across the narrowest groove that matters (0.12 m).'),

        Node(
            package='rosbot_plus_description', executable='cloud_sanitize',
            name='cloud_sanitize', output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'fastlio_sim.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
        ),

        Node(
            package='rosbot_plus_description', executable='elevation_map',
            name='elevation_map', output='screen',
            parameters=[{
                'use_sim_time': True,
                'resolution': LaunchConfiguration('resolution'),
                'odom_topic': LaunchConfiguration('map_odom_topic'),
                # Registering from a pose needs the SENSOR-frame cloud; the
                # pre-registered one already carries the drift being removed.
                'register_from_pose': PythonExpression(
                    ["'", LaunchConfiguration('map_odom_topic'),
                     "' != '/Odometry'"]),
                'cloud_topic': PythonExpression(
                    ["'/mid360/points_clean' if '",
                     LaunchConfiguration('map_odom_topic'),
                     "' != '/Odometry' else '/cloud_registered'"]),
            }],
        ),

        # C4. Publishes /discontinuities and /discontinuity_markers.
        # Note it does not yet recover the W1 step - see the STATUS section at
        # the top of discontinuity_detection/detector.py. It does classify the
        # landmark pillars correctly, which is the evidence that the blocker is
        # the elevation map rather than the classifier.
        Node(
            package='discontinuity_detection', executable='detector',
            name='discontinuity_detector', output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('detector')),
        ),
    ])
