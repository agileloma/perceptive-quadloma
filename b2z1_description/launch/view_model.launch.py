"""
Launch file for visualizing the robot model in RViz2.

This script initializes and launches the following nodes:
1. joint_state_publisher_gui - For publishing joint states interactively.
2. robot_state_publisher - For publishing the robot's state based on th URDF.
3. rviz2 - For visualizing the robot model in RViz2.

Author: Jun Li (junli@hit.edu.cn)
Date:   June 14, 2025
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Constants
PACKAGE_NAME = "b2z1_description"
URDF_FILE_NAME = "b2z1.urdf"
RVIZ_CONFIG_FILE_NAME = "view_model.rviz"


def generate_launch_description():
  """
  Generate a launch description for visualizing the robot model.

  Returns:
    LaunchDescription: A description of the launch configuration.
  """
  # Resolve paths manually for validation
  package_share_dir = FindPackageShare(PACKAGE_NAME).find(PACKAGE_NAME)
  urdf_file_path = os.path.join(
      package_share_dir, "urdf", URDF_FILE_NAME)
  rviz_config_file_path = os.path.join(
      package_share_dir, "rviz", RVIZ_CONFIG_FILE_NAME)

  # Check if the required files exist
  if not Path(urdf_file_path).is_file():
    raise FileNotFoundError(f"URDF file not found: {urdf_file_path}")
  if not Path(rviz_config_file_path).is_file():
    raise FileNotFoundError(f"RViz config file not found: {rviz_config_file_path}")

  # Use PathJoinSubstitution for ROS2 launch-time resolution
  urdf_file = PathJoinSubstitution(
    [FindPackageShare(PACKAGE_NAME), 'urdf', URDF_FILE_NAME]
  )
  rviz_config_file = PathJoinSubstitution(
    [FindPackageShare(PACKAGE_NAME), 'rviz', RVIZ_CONFIG_FILE_NAME]
  )

  # Node: Joint State Publisher GUI
  joint_state_publisher_node = Node(
    name="joint_state_publisher_node", 
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
    output="screen",
    arguments=[urdf_file]
  )

  # Node: Robot State Publisher
  robot_state_publisher_node = Node(
    name="robot_state_publisher_node",
    package="robot_state_publisher", 
    executable="robot_state_publisher",
    output="screen",
    arguments=[urdf_file]
  )

  # Node: RViz2
  rviz_node = Node(
    package="rviz2", 
    executable="rviz2", 
    name="rviz2",
    output="screen",
    arguments=["-d", rviz_config_file],
  )

  # Nodes to start
  node_to_start = [
    joint_state_publisher_node,
    robot_state_publisher_node, 
    rviz_node
  ]

  return LaunchDescription(node_to_start)
