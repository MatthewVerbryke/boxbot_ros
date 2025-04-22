#!/usr/bin/env python3

"""
  Rviz2 launch file for Boxbot
  
  Copyright 2018-2025 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
	LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	
	# Declare launch arguments
	dof_arg = DeclareLaunchArgument(
		name='dof',
		default_value='6',
		choices=['5', '6'],
		description='Select WidowX arm model by degrees of freedom'
	)
	jsp_gui_arg = DeclareLaunchArgument(
		name='use_jsp_gui',
		default_value='true',
		choices=['true', 'false'],
		description='Flag to enable joint_state_publisher_gui'
	)
	
	# Construct robot file name
	file_name = PythonExpression(
		['"boxbot_{}dof.urdf"', '.format(',	LaunchConfiguration("dof"),	')']
	)
	
	# Construct required paths
	model_path = PathJoinSubstitution(['robots', file_name])
	rviz_config_path = PathJoinSubstitution(
		[FindPackageShare('boxbot_description'), 'config', 'boxbot.rviz']
	)
	urdf_launch_path = PathJoinSubstitution(
		[FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']
	)
	
	# Configure Nodes
	virtual_joint_broadcaster_node = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		arguments=[
			'--x', '0', '--y', '0', '--z', '0',
			'--yaw', '0', '--pitch', '0', '--roll', '0',
			'--frame-id', 'world', '--child-frame-id', 'base_link'
		]
	)
	rviz_node = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(urdf_launch_path),
		launch_arguments={
			'urdf_package': 'boxbot_description',
			'urdf_package_path': model_path,
			'rviz_config': rviz_config_path,
			'jsp_gui': LaunchConfiguration('use_jsp_gui')
		}.items()
	)
	
	return LaunchDescription([
		dof_arg,
		jsp_gui_arg,
		virtual_joint_broadcaster_node,
		rviz_node
	])
