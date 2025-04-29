#!/usr/bin/env python3

"""
  Control launch file for Boxbot
  
  Copyright 2018-2025 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
	Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
	PythonExpression
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
	gazebo_arg = DeclareLaunchArgument(
		name='use_gazebo',
		default_value='true',
		choices=['true','false'],
		description='Configure control for gazebo or actual robot hardware'
	)
	
	# Construct model file name
	desc_file = PythonExpression(
		['"boxbot_{}dof.urdf"', '.format(',	LaunchConfiguration('dof'),	')']
	)
	
	# Construct other names
	left_control_name = PythonExpression(
		['"left_arm_{}dof_controller"', '.format(', LaunchConfiguration('dof'), ')']
	)
	right_control_name = PythonExpression(
		['"right_arm_{}dof_controller"', '.format(', LaunchConfiguration('dof'), ')']
	)	
	
	# Construct required paths
	controller_path = PathJoinSubstitution(
		[FindPackageShare('boxbot_control'), 'config', 'boxbot_control.yaml']
	)
	desc_path = PathJoinSubstitution(
		[FindPackageShare('boxbot_description'), 'robots', desc_file]
	)
	
	# Retreive robot description
	desc_content = Command([
	    PathJoinSubstitution([FindExecutable(name='xacro')]),
		' ', desc_path, ' ', 'use_gazebo:=',
		LaunchConfiguration('use_gazebo')
	])
	
	# Configure nodes
	# control_node = Node(
		# package='controller_manager',
		# executable='ros2_control_node',
		# output="both",
		# parameters=[controller_path]
	# )
	robot_state_pub_spawner = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': desc_content}]
	)
	joint_state_pub_spawner = Node(
		package='controller_manager',
		executable='spawner',
		arguments=['joint_state_broadcaster']
	)
	left_arm_control_spawner = Node(
		package='controller_manager',
		executable='spawner',
		arguments=[left_control_name, '--param-file', controller_path]
	)
	right_arm_control_spawner = Node(
		package='controller_manager',
		executable='spawner',
		arguments=[right_control_name, '--param-file', controller_path]
	)
	head_control_spawner = Node(
		package='controller_manager',
		executable='spawner',
		arguments=['head_controller', '--param-file', controller_path]
	)
	
	return LaunchDescription([
		dof_arg,
		gazebo_arg,
		#control_node,
		robot_state_pub_spawner,
		joint_state_pub_spawner,
		left_arm_control_spawner,
		right_arm_control_spawner,
		head_control_spawner
	])
		
