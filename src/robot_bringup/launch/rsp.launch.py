import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

def generate_launch_description():

    pkg_dir = get_package_share_directory('my_robot_description')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'amr_robot.urdf')
    joint_yaml_path = os.path.join(pkg_dir, 'config', 'wheels_robot_joint.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Load joint names from YAML
    with open(joint_yaml_path, 'r') as f:
        joint_data = yaml.safe_load(f)
    joint_list = joint_data.get('controller_joint_names', [])

    # URDF
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # joint_state_publisher (only wheels)
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': joint_list,  # use only the wheel joints
            'publish_rate': 50.0
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        rsp_node,
        jsp_node
    ])
