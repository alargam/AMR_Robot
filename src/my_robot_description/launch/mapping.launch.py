import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_description'
    pkg_dir = get_package_share_directory(pkg_name)
    
    gazebo_launch = os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    slam_config = os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml')

    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'mapping.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': slam_config
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])