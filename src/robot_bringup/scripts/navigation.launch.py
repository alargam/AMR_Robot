import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument # <--- أضفنا DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration # <--- أضفنا LaunchConfiguration

def generate_launch_description():
    pkg_name = 'my_robot_description'
    pkg_dir = get_package_share_directory(pkg_name)
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    default_map_file = os.path.join(pkg_dir, 'maps', 'my_map.yaml')
    
    params_file = os.path.join(pkg_dir, 'config', 'my_nav_params.yaml')
    gazebo_launch = os.path.join(pkg_dir, 'launch', 'gazebo.launch.py') 
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'nav2_config.rviz')

    map_arg = LaunchConfiguration('map')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map file to load'
    )

    return LaunchDescription([
        declare_map_cmd,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_arg,
                'params_file': params_file,
                'use_sim_time': 'true',
                'use_rviz': 'false' 
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])