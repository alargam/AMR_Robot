#!/usr/bin/env python3
"""
AMR Robot Main Launch File - ROS 2 Jazzy

This launch file starts all essential nodes for the AMR robot:
1. Robot State Publisher (URDF + TF)
2. Micro-ROS Agent (ESP32 data)
3. LiDAR Node (LDLiDAR LD06)
4. EKF Node (Encoders + IMU fusion)
5. SLAM Toolbox (mapping & localization)
6. Optional: Rosbridge + ROSAPI (Foxglove Studio visualization)

Notes:
- YAML config files provide Node parameters only.
- Static transforms must be passed directly as arguments in the Launch file.
- SLAM startup is delayed to ensure TF and LiDAR are stable.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # -------------------------------
    # Package directory
    # -------------------------------
    pkg_dir = get_package_share_directory('my_robot_description')

    # -------------------------------
    # Config files
    # -------------------------------
    lidar_config = os.path.join(pkg_dir, 'config', 'lidar_config.yaml')
    ekf_config   = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    slam_config  = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    # -------------------------------
    # Robot State Publisher (URDF + TF)
    # -------------------------------
    # Publishes TF frames for all robot links
    # Essential for LiDAR, SLAM, Navigation, RViz visualization
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # -------------------------------
    # Micro-ROS Agent (ESP32 connectivity)
    # -------------------------------
    # Reads ESP32 serial data (Encoders, IMU, ToF) and publishes ROS2 topics
    micro_ros = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0'],
        output='screen'
    )

    # -------------------------------
    # LiDAR Node (LDLiDAR LD06)
    # -------------------------------
    # Publishes LaserScan and 2D PointCloud topics for mapping and navigation
    ldlidar_node = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ldlidar_publisher_ld06',
        parameters=[lidar_config],
        output='screen'
    )

    # -------------------------------
    # a Static Transform Publisher (LiDAR -> Base Link)
    # -------------------------------
    # TF from robot base to LiDAR frame
    lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_lidar_link',
        arguments=[
            '0', '0', '0.18',    # x, y, z offset in meters
            '0', '0', '0',       # roll, pitch, yaw
            'base_link',          # parent frame
            'lidar_link'          # child frame
        ]
    )

    # -------------------------------
    # EKF Node (robot_localization)
    # -------------------------------
    # Fuses Encoder and IMU data to provide a filtered pose
    # Output: /odom_filtered, used by SLAM and Navigation
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen'
    )

    # -------------------------------
    # SLAM Toolbox (mapping & localization)
    # -------------------------------
    # Builds real-time map using LiDAR and EKF pose
    # Delayed startup to ensure LiDAR and TF are ready
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen'
    )

    delayed_slam = TimerAction(
        period=3.0,  # Delay 3 seconds
        actions=[slam_node]
    )

    # -------------------------------
    # Foxglove / Web Visualization (Optional)
    # -------------------------------
    # rosbridge_websocket provides WebSocket access for Foxglove Studio
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090, 'address': '0.0.0.0'}]
    )

    # rosapi_node provides ROS system info to Foxglove
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi'
    )

    # -------------------------------
    # Launch all nodes in proper order
    # -------------------------------
    return LaunchDescription([
        rsp,                 # 1️⃣ URDF + TF
        micro_ros,           # 2️⃣ ESP32 connectivity
        ldlidar_node,        # 3️⃣ LiDAR
        lidar_tf_node,       # 3️⃣a Static TF for LiDAR
        ekf_node,            # 4️⃣ EKF Pose Filter
        delayed_slam,        # 5️⃣ SLAM Toolbox
        rosbridge_node,      # 6️⃣ Web visualization
        rosapi_node          # 6️⃣ System API
    ])
