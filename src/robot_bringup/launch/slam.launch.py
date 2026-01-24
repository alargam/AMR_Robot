"""
Falcon AMR Full Robot Launch with SLAM
========================================
This launch file starts all nodes needed for SLAM mapping:
- Robot State Publisher (static TF: base_link -> lidar_link)
- LiDAR Driver (/scan)
- micro-ROS Agent (/odom/wheel from ESP32)
- EKF Filter (odom -> base_footprint)
- SLAM Toolbox (map -> odom)

CRITICAL: Node ordering ensures proper TF tree construction
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # ============================================
    # CRITICAL: Proper Node Ordering for TF Tree
    # ============================================
    # TF Tree: map -> odom -> base_footprint -> base_link -> lidar_link
    # 
    # Publishing order:
    # 1. Robot State Publisher: base_link -> lidar_link (static)
    # 2. EKF: odom -> base_footprint (from wheel odometry)
    # 3. SLAM: map -> odom (from scan matching)
    
    # 1. Package paths
    description_pkg = get_package_share_directory('my_robot_description')
    lidar_pkg = get_package_share_directory('ldlidar_ros2')
    bringup_pkg = get_package_share_directory('robot_bringup')
    urdf_model_path = os.path.join(description_pkg, 'urdf', 'amr_robot.urdf')

    # 2. Robot State Publisher (MUST START FIRST)
    # Publishes: base_footprint -> base_link -> lidar_link (from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_model_path]),
            'use_sim_time': False
        }]
    )

    # 3. LiDAR Driver (LD06)
    # Publishes: /scan (sensor_msgs/LaserScan)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'ld06.launch.py')
        )
    )

    # 4. micro-ROS Agent
    # Connects to ESP32 and publishes: /odom/wheel (nav_msgs/Odometry)
    # CRITICAL: Wait a moment for serial connection to establish
    microros_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/esp32', '-b', '115200']
    )

    # 5. EKF Filter Node
    # Subscribes: /odom/wheel
    # Publishes: /odom (filtered) and TF: odom -> base_footprint
    # CRITICAL: Start after micro-ROS agent has connected (delay 2 seconds)
    ekf_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[os.path.join(bringup_pkg, 'config', 'ekf.yaml')]
            )
        ]
    )

    # 6. SLAM Toolbox Node
    # Subscribes: /scan, /odom (or uses TF: odom -> base_footprint)
    # Publishes: /map (nav_msgs/OccupancyGrid) and TF: map -> odom
    # CRITICAL: Start after EKF is publishing odom -> base_footprint (delay 3 seconds)
    slam_toolbox_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(bringup_pkg, 'config', 'slam_params.yaml'),
                    {'use_sim_time': False}
                ]
            )
        ]
    )

    return LaunchDescription([
        # CRITICAL: Launch order ensures TF tree is built correctly
        robot_state_publisher,  # 1. Static transforms (base_link -> lidar_link)
        lidar_launch,           # 2. LiDAR driver (/scan)
        microros_node,          # 3. micro-ROS agent (waits for ESP32 connection)
        ekf_node,               # 4. EKF (waits 2s, publishes odom -> base_footprint)
        slam_toolbox_node,      # 5. SLAM (waits 3s, publishes map -> odom)
    ])
