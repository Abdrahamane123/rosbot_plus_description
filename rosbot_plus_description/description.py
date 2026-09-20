"""Generate URDF that Gazebo Classic can forward as a ROS parameter."""

import xml.etree.ElementTree as ET

import xacro


def generate_robot_description(xacro_file, mappings=None):
    """Expand xacro and discard comments and formatting-only whitespace.

    gazebo_ros2_control passes robot_description through rcl's parameter
    argument parser. XML comments containing ': ' can be interpreted as YAML
    mappings there, even when robot_state_publisher received a string.
    ElementTree's default parser drops comments without changing the model.
    """
    document = xacro.process_file(str(xacro_file), mappings=mappings or {})
    root = ET.fromstring(document.toxml())
    for element in root.iter():
        if element.text is not None and not element.text.strip():
            element.text = None
        if element.tail is not None and not element.tail.strip():
            element.tail = None
    return ET.tostring(root, encoding='unicode')
