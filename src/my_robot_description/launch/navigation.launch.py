import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument # <--- أضفنا DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration # <--- أضفنا LaunchConfiguration

def generate_launch_description():
    # 1. تحديد المسارات الأساسية
    pkg_name = 'my_robot_description'
    pkg_dir = get_package_share_directory(pkg_name)
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    # تحديد المسار الافتراضي (Default) للخريطة في حال لم تكتب شيئاً في التيرمينال
    default_map_file = os.path.join(pkg_dir, 'maps', 'my_map.yaml')
    
    params_file = os.path.join(pkg_dir, 'config', 'my_nav_params.yaml')
    gazebo_launch = os.path.join(pkg_dir, 'launch', 'gazebo.launch.py') 
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'nav2_config.rviz')

    # 2. إعداد متغير التكوين (Launch Configuration)
    # هذا المتغير هو الذي سيلتقط القيمة التي تكتبها في map:=...
    map_arg = LaunchConfiguration('map')

    # 3. الإعلان عن الحجة (Argument Declaration)
    # هذا يخبر ROS أن هناك معامل اسمه 'map' قد يأتي من الخارج
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map file to load'
    )

    return LaunchDescription([
        # يجب تفعيل الإعلان عن الحجة أولاً
        declare_map_cmd,

        # 4. تشغيل Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        # 5. تشغيل نظام الملاحة Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                # هنا التغيير الجوهري: نستخدم المتغير map_arg بدلاً من المسار الثابت
                'map': map_arg,
                'params_file': params_file,
                'use_sim_time': 'true',
                'use_rviz': 'false' 
            }.items(),
        ),

        # 6. تشغيل RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])