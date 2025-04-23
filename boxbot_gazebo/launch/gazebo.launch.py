#!/usr/bin/env python3

"""
  Gazebo simulation launch file for Boxbot.
  
  Copyright 2018-2025 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
	DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
	LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def retrieve_top_directory():
	"""
	Starting from the 'boxbot_gazebo' share directory, retrieve the path
	to the top level boxbot_ros directory within the ROS2 workspace.
	
	NOTE: this is needed in order to provide Gazebo with a proper path 
	      to the meshes stored in the 'boxbot_description' package, as
	      search for these does not work as before in Gazebo classic.
	"""
	
	# Get path to share directory
	cur_path = get_package_share_directory('boxbot_gazebo')
	
	# Get path to ROS2 workspace
	at_ws = False
	while not at_ws:
		cur_path, cur_dir = os.path.split(cur_path)
		if cur_dir == 'install':
			ws_path = cur_path
			at_ws = True
	
	# Construct path to boxbot top level directory
	top_dir = os.path.join(ws_path, 'src/boxbot_ros')
	
	return top_dir


def generate_launch_description():
	
	# Provide path to find specific meshes/resources
	top_dir = retrieve_top_directory()
	
	# Declare launch arguments
	world_arg = DeclareLaunchArgument(
		name='world',
		default_value='empty.world',
		description='Sim world file to use from the "boxbot_gazebo/world" folder'
	)
	dof_arg = DeclareLaunchArgument(
		name='dof',
		default_value='6',
		choices=['5', '6'],
		description='Select WidowX arm model by degrees of freedom'
	)
	
	# Construct model file name
	file_name = PythonExpression(
		['"boxbot_{}dof.urdf"', '.format(',	LaunchConfiguration("dof"),	')']
	)
	
	# Construct required paths
	gz_launch_path = PathJoinSubstitution(
		[FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
	)
	world_path = PathJoinSubstitution(
		[FindPackageShare('boxbot_gazebo'), 'world', LaunchConfiguration('world')]
	)
	model_path = PathJoinSubstitution(
		[FindPackageShare('boxbot_description'), 'robots', file_name]
	)
	
	# Configure nodes
	resource_path_var = SetEnvironmentVariable(
		'GZ_SIM_RESOURCE_PATH', top_dir
	)
	gz_launch_include = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(gz_launch_path),
		launch_arguments={
			'gz_args': [world_path],
			'on_exit_shutdown': 'True'
		}.items()
	)
	spawn_entity_node = Node(
		package='ros_gz_sim',
		executable='create',
		output='screen',
		arguments=['-file', model_path,
				   '-x', '0.0',
				   '-y', '0.0',
				   '-z', '0.0',
				   '-R', '0.0',
				   '-P', '0.0',
				   '-Y', '0.0',
				   '-name', 'boxbot',
				   '-allow_renaming', 'false']
	)
	
	return LaunchDescription([
		world_arg,
		dof_arg,
		resource_path_var,
		gz_launch_include,
		spawn_entity_node
	])
