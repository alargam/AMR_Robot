import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ========================================================================
    # 1. Define Paths to Packages and Config Files
    # ========================================================================
   
    pkg_bringup = get_package_share_directory('robot_bringup')
    ekf_config_file = os.path.join(pkg_bringup, 'config', 'ekf.yaml')

    # ========================================================================
    # 2. Include Robot State Publisher (RSP)
    # ========================================================================
   
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_bringup, 'launch', 'rsp.launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # ========================================================================
    # 3. Micro-ROS Agent (ESP32 Interface)
    # ========================================================================
    # This node acts as the bridge between the Raspberry Pi and the ESP32.
    # It receives Odometry (position) and sends Velocity commands (cmd_vel).
    # NOTE: We use the persistent name '/dev/esp32' defined in udev rules
    # to avoid confusion with other USB devices.
    
    micro_ros = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/esp32', '-b', '115200']
    )

    # ========================================================================
    # 4. LiDAR Driver (LD06)
    # ========================================================================
    # This node drives the LD06 LiDAR sensor.
    # NOTE: We use '/dev/ldlidar' persistent name.
    # CRITICAL FIX: 'laser_scan_dir' must be a Boolean (True), not a string.
    
    lidar = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ldlidar_publisher_ld06',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD06',
            'laser_scan_topic_name': 'scan',
            'point_cloud_2d_topic_name': 'pointcloud',
            'frame_id': 'lidar_link',
            'port_name': '/dev/ldlidar',   # Fixed path from udev rules
            'serial_baudrate': 230400,     # Standard baudrate for LD06
            'laser_scan_dir': True,        # True = Counterclockwise
            'enable_angle_crop_func': False
        }]
    )

    # ========================================================================
    # 5. Extended Kalman Filter (EKF) - WITH DELAY
    # ========================================================================
    # The EKF node fuses the raw odometry from ESP32 to provide a smoother
    # and more accurate 'odom' -> 'base_footprint' transform.
    #
    # WHY THE DELAY?
    # We wait 3.0 seconds before starting EKF to ensure that:
    # 1. Micro-ROS has established a connection.
    # 2. ESP32 has started publishing /odom/wheel data.
    # Without this delay, EKF might start too early and complain about missing data.
    
    ekf = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config_file, {'use_sim_time': False}]
            )
        ]
    )

    return LaunchDescription([
        rsp,        # 1. Start Robot Model & TF
        micro_ros,  # 2. Start Connection to ESP32
        lidar,      # 3. Start LiDAR
        ekf         # 4. Start Sensor Fusion (after 3s delay)
    ])