import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():

    package_name = 'my_robot_description'
    urdf_file_name = 'amr_robot.urdf'

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)
    
    world_file_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    robot_desc_param = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc_param, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', 
                   '-topic', 'robot_description',
                   '-x', '-10.0', 
                   '-y', '-0.5', 
                   '-z', '0.05'], 
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])