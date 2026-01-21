import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    
    # =========================================================================
    # 1. PATH CONFIGURATION
    # Locate the directories for your robot's description, lidar driver, 
    # and the main bringup package.
    # =========================================================================
    description_pkg = get_package_share_directory('my_robot_description')
    lidar_pkg = get_package_share_directory('ldlidar_ros2')
    bringup_pkg = get_package_share_directory('robot_bringup')

    # Path to the URDF model exported from SolidWorks
    urdf_model_path = os.path.join(description_pkg, 'urdf', 'amr_robot.urdf')

    # =========================================================================
    # 2. ROBOT STATE PUBLISHER
    # This node parses the URDF and publishes the static TF tree.
    # It tells ROS exactly where the 'lidar_link' and wheels are located 
    # relative to 'base_link'.
    # =========================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_model_path])}]
    )

    # =========================================================================
    # 3. LIDAR DRIVER (LD06)
    # Includes the launch file for the LDLiDAR. It will start rotating 
    # and publishing /scan data.
    # =========================================================================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'ld06.launch.py')
        )
    )

    # =========================================================================
    # 4. MICRO-ROS AGENT
    # This node is the bridge to your ESP32. We run it as a 'Node' with 
    # arguments to match the manual terminal command that worked.
    # -b 115200: Matches the baud rate in your Arduino code.
    # --dev /dev/esp32: Matches your custom udev rule.
    # =========================================================================
    microros_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/esp32', '-b', '921600']
    )

    # =========================================================================
    # 5. EKF FILTER (ROBOT LOCALIZATION)
    # The brain for robot positioning. It listens to the raw /odom from 
    # the ESP32 and provides a smooth, filtered transform between 
    # 'odom' and 'base_link'.
    # =========================================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(bringup_pkg, 'config', 'ekf.yaml')]
    )

    # =========================================================================
    # 6. ASSEMBLE ALL COMPONENTS
    # =========================================================================
    return LaunchDescription([
        robot_state_publisher,
        lidar_launch,
        microros_node,
        ekf_node
    ])