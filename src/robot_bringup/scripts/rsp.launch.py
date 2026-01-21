import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_dir = get_package_share_directory('my_robot_description')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'amr_robot.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        rsp_node
    ])
