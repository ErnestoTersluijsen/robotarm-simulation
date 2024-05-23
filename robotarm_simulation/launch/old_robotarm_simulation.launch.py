from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robotarm_sim_path = FindPackageShare('robotarm_simulation')
    default_robotarm_model_path = PathJoinSubstitution(['urdf', 'lynxmotion_arm.urdf'])
    default_rviz_config_path = PathJoinSubstitution([robotarm_sim_path, 'rviz', 'urdf_config.rviz'])
    default_mug_model_path = PathJoinSubstitution([robotarm_sim_path, 'urdf', 'mug.urdf'])

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_robotarm_model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'robotarm_simulation',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))

    # ld.add_action(Node(
    # 	    package="robotarm_simulation",
    #     	executable="robotarm_simulation",
    #         parameters=[]
	# 	)
    # )

    return ld