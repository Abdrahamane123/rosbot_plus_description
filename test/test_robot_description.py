"""Check the simulated URDF and its passage through ROS parameter parsing."""

import xml.etree.ElementTree as ET
from pathlib import Path

import ament_index_python.packages

import pytest

from rclpy.utilities import remove_ros_args
from rclpy.validate_node_name import validate_node_name

from rosbot_plus_description.description import generate_robot_description

import yaml


@pytest.fixture
def describe_robot(monkeypatch):
    """Resolve this source checkout without requiring a colcon installation."""
    package_root = Path(__file__).resolve().parents[1]
    original_lookup = ament_index_python.packages.get_package_share_directory

    def package_share(package_name):
        if package_name == 'rosbot_plus_description':
            return str(package_root)
        return original_lookup(package_name)

    monkeypatch.setattr(
        ament_index_python.packages,
        'get_package_share_directory', package_share)

    def describe(**mappings):
        return generate_robot_description(
            str(package_root / 'urdf' / 'rosbot_plus.xacro'),
            mappings={'sim_mode': 'true', **mappings},
        )

    return describe


def test_description_survives_ros_parameter_parsing(describe_robot):
    """Gazebo Classic forwards the entire URDF as an RCL parameter override."""
    description = describe_robot()
    assert yaml.safe_load(description) == description
    assert remove_ros_args([
        'gazebo_ros2_control', '--ros-args',
        '-p', 'robot_description:=' + description,
    ]) == ['gazebo_ros2_control']
    assert ET.fromstring(description).tag == 'robot'


def test_gazebo_plugins_have_valid_ros_node_names(describe_robot):
    """Each Classic plugin name must also be accepted as a ROS node name."""
    robot = ET.fromstring(describe_robot())
    plugins = robot.findall('.//plugin[@filename]')
    assert {plugin.get('filename') for plugin in plugins} == {
        'libgazebo_ros2_control.so',
        'libgazebo_ros_ray_sensor.so',
        'libgazebo_ros_imu_sensor.so',
        'libgazebo_ros_p3d.so',
        'libgazebo_ros_camera.so',
    }
    for plugin in plugins:
        validate_node_name(plugin.get('name'))


def test_control_interfaces_match_drive_and_passive_joints(describe_robot):
    """Bind wheel commands and passive suspension states to existing joints."""
    robot = ET.fromstring(describe_robot())
    control = robot.find('ros2_control')
    assert control.findtext('hardware/plugin') == (
        'gazebo_ros2_control/GazeboSystem')

    drive_names = {'RL_Joint', 'RR_Joint'}
    passive_names = {
        f'{corner}_suspension_joint' for corner in ('FL', 'FR', 'RL', 'RR')
    }
    passive_names |= {'FL_Joint', 'FR_Joint'}
    joints = {joint.get('name'): joint for joint in control.findall('joint')}
    steering_names = {'FL_steering_joint', 'FR_steering_joint'}
    assert set(joints) == drive_names | passive_names | steering_names
    urdf_joint_names = {joint.get('name') for joint in robot.findall('joint')}
    assert set(joints) <= urdf_joint_names
    for name, joint in joints.items():
        commands = {
            entry.get('name') for entry in joint.findall('command_interface')
        }
        assert commands == (
            {'velocity'} if name in drive_names else
            {'position'} if name in steering_names else set())
        states = {
            entry.get('name') for entry in joint.findall('state_interface')
        }
        assert states == {'position', 'velocity', 'effort'}


def test_lidar_can_be_disabled_without_removing_imu_or_control(describe_robot):
    """Keep the scanner planar and limit its disable flag to that sensor."""
    enabled = ET.fromstring(describe_robot())
    laser = enabled.find('.//sensor[@name="laser_2d"]')
    assert laser.get('type') == 'ray'
    assert laser.findtext('plugin/output_type') == 'sensor_msgs/LaserScan'
    assert laser.findtext('plugin/ros/remapping') == '~/out:=/scan'
    assert laser.find('ray/scan/vertical') is None

    disabled = ET.fromstring(describe_robot(enable_lidar='false'))
    assert disabled.find('.//sensor[@name="laser_2d"]') is None
    assert disabled.find('.//sensor[@type="imu"]') is not None
    assert disabled.find('ros2_control') is not None


@pytest.mark.parametrize('mass', ['0.0', '22.0'])
def test_payload_arguments_reach_the_urdf(describe_robot, mass):
    """Preserve optional payload mass and pose during serialization."""
    robot = ET.fromstring(describe_robot(
        payload_mass=mass, payload_x='0.3', payload_z='0.5'))
    payload = robot.find('link[@name="payload_link"]')
    joint = robot.find('joint[@name="payload_joint"]')
    if float(mass) == 0.0:
        assert payload is None
        assert joint is None
    else:
        assert float(payload.find('inertial/mass').get('value')) == float(mass)
        xyz = [
            float(value) for value in joint.find('origin').get('xyz').split()
        ]
        assert xyz == [0.3, 0.0, 0.5]


def test_front_sensors_and_camera_optical_frame(describe_robot):
    """Keep the camera ahead of the lidar, both ahead of the chassis centre."""
    robot = ET.fromstring(describe_robot())
    def origin(name):
        return [float(v) for v in robot.find(
            f"joint[@name='{name}']/origin").get('xyz').split()]
    camera = origin('camera_joint')
    lidar = origin('livox_joint')
    assert camera[0] > lidar[0] > 0.265
    assert lidar[2] > camera[2]
    optical = robot.find("joint[@name='camera_optical_joint']")
    assert optical.find('child').get('link') == 'camera_optical_frame'
    sensor = robot.find(".//sensor[@name='depth_camera']")
    assert sensor.get('type') == 'depth'
    assert sensor.findtext('plugin/frame_name') == 'camera_optical_frame'
    disabled = ET.fromstring(describe_robot(enable_camera='false'))
    assert disabled.find(".//sensor[@name='depth_camera']") is None
    assert disabled.find("link[@name='camera_link']") is not None


def test_ackermann_geometry_and_controller_agree(describe_robot):
    robot = ET.fromstring(describe_robot())
    config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                             'config/my_controllers.yaml').read_text())
    params = config['steering_cont']['ros__parameters']
    assert params['front_wheels_names'] == ['FR_steering_joint', 'FL_steering_joint']
    assert params['rear_wheels_names'] == ['RR_Joint', 'RL_Joint']
    for corner in ('FR', 'FL'):
        steering = robot.find(f"joint[@name='{corner}_steering_joint']")
        assert steering.get('type') == 'revolute'
        assert steering.find('axis').get('xyz') == '0 0 1'
        spin = robot.find(f"joint[@name='{corner}_Joint']")
        assert spin.find('parent').get('link') == f'{corner}_steering_link'
        suspension = robot.find(f"joint[@name='{corner}_suspension_joint']")
        xyz = list(map(float, suspension.find('origin').get('xyz').split()))
        assert xyz[0] == params['wheelbase']
        assert abs(xyz[1])*2 == params['front_wheel_track']
    total_mass = sum(float(m.get('value')) for m in robot.findall('link/inertial/mass'))
    assert total_mass == pytest.approx(35.16)


def test_public_interfaces_keep_odometry_and_ground_truth_separate(describe_robot):
    """Do not mix a world pose into the controller's odom stream."""
    robot = ET.fromstring(describe_robot())
    control = robot.find(".//plugin[@name='gazebo_ros2_control']")
    remaps = {entry.text for entry in control.findall('ros/remapping')}
    assert '/steering_cont/reference_unstamped:=/cmd_vel' in remaps
    assert '/steering_cont/odometry:=/odom' in remaps
    assert '/steering_cont/tf_odometry:=/tf' in remaps
    truth = robot.find(".//plugin[@name='gazebo_ros_ground_truth']")
    assert truth.findtext('ros/remapping') == 'odom:=/ground_truth/odom'
    assert truth.findtext('frame_name') == 'world'
    imu = robot.find(".//plugin[@name='gazebo_ros_imu']")
    assert imu.findtext('ros/remapping') == (
        '~/out:=/mobile_base/sensors/imu_data')
    camera = robot.find(".//plugin[@name='gazebo_ros_camera']")
    assert camera.findtext('camera_name') == 'camera'
    assert camera.findtext('ros/namespace', default='/') == '/'
