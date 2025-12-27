import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue # ضروري لتجنب الأخطاء

def generate_launch_description():

    package_name = 'my_robot_description'
    # تأكدنا من الاسم من صورتك السابقة
    urdf_file_name = 'amr_robot.SLDASM.urdf'

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)

    # التحقق من وجود الملف
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    # ملف إعدادات Rviz (اختياري، إذا لم يوجد سيفتح Rviz فارغاً)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    # قراءة وصف الروبوت وتجهيزه (الحل السحري ParameterValue)
    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    # 1. عقدة Robot State Publisher (تنشر شكل الروبوت)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. عقدة Joint State Publisher GUI (نافذة منزلقة لتحريك العجلات يدوياً)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 3. تشغيل Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])