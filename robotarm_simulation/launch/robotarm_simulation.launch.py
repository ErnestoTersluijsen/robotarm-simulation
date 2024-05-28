from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
	ld = LaunchDescription()

	robotarm_sim_path = FindPackageShare('robotarm_simulation')
	default_robotarm_model_path = PathJoinSubstitution([robotarm_sim_path, 'urdf', 'lynxmotion_arm.urdf'])
	default_mug_model_path = PathJoinSubstitution([robotarm_sim_path, 'urdf', 'mug.urdf'])
	default_rviz_config_path = PathJoinSubstitution([robotarm_sim_path, 'rviz', 'urdf_config.rviz'])

	ld.add_action(DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
									 description='Absolute path to rviz config file'))

	ld.add_action(DeclareLaunchArgument(name='robotarm_model', default_value=default_robotarm_model_path,
										description='Path to robot urdf file relative to urdf_tutorial package'))

	ld.add_action(DeclareLaunchArgument(name='mug_model', default_value=default_mug_model_path,
										description='Path to robot urdf file relative to urdf_tutorial package'))


	ld.add_action(Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_robotarm',
		output='screen',
		arguments=[LaunchConfiguration('robotarm_model')]
	))

	ld.add_action(Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_mug',
		output='screen',
		arguments=[LaunchConfiguration('mug_model')],
		remappings=[('/robot_description', '/mug_robot_description')]
	))
	
	ld.add_action(Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', LaunchConfiguration('rvizconfig')]
	))

	ld.add_action(Node(
		package='robotarm_simulation',
		executable='robotarm'
	))

	# ld.add_action(Node(
	# 	package='robotarm_simulation',
	# 	executable='mug',
	# 	parameters=[{'x': 0.35, 'y': 0.0, 'z': 0.0}]
	# ))

	return ld