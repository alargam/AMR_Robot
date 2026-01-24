🤖 AMR Autonomous Mobile Robot (AMR) – ROS 2 Jazzy
This repository contains the complete hardware bringup, system integration, and software architecture for an Autonomous Mobile Robot (AMR) developed as a Mechatronics Engineering Graduation Project (2026).

The project is carefully structured to separate hardware drivers, robot description, and system logic, following professional ROS 2 Jazzy best practices.

🏗️ Project Architecture Overview
The source code (src) is organized to ensure scalability, maintainability, and clear responsibility separation:

hardware/ Contains all low-level hardware drivers and communication bridges.

my_robot_description/ Defines the physical structure of the robot using URDF, meshes, and TF relationships.

robot_bringup/ The system integration layer ("robot brain") that unifies sensors, localization, and launch logic.

🚀 Quick Start Aliases
To simplify field operation and debugging, the following aliases are configured in .bashrc:

1️⃣ Launch Commands
Alias	Description	Launch File
start_robot	Launch the complete robot system	robot_bringup/full_robot.launch.py
start_lidar	Start LD06 LiDAR driver	ldlidar_ros2/ld06.launch.py
start_camera	Start Orbbec Astra Pro camera	astra_camera/astra_pro.launch.xml
start_esp	Start micro-ROS agent (ESP32)	Serial via /dev/esp32
2️⃣ Verification Commands
check_topics – Lists all active ROS 2 topics
check_odom – Monitors filtered odometry output from EKF
🧭 Sensor Fusion & Localization
The robot uses the robot_localization package to fuse wheel encoder odometry and IMU data.

Key Features
Extended Kalman Filter (EKF) for accurate state estimation
Drift reduction and smooth pose output
REP-105 compliant TF tree:
map → odom → base_link
Compatible with ROS 2 Jazzy using the rolling-devel branch
📂 Repository Structure
AMR_Robot/
├── src/
│   ├── hardware/               # Drivers & Communication
│   │   ├── ldlidar_ros2/        # LD06 LiDAR Driver
│   │   ├── micro-ROS-Agent/     # ESP32 micro-ROS Bridge
│   │   └── ros2_astra_camera/   # Orbbec Astra Pro Driver
│   ├── my_robot_description/   # Robot Physical Model
│   │   ├── urdf/               # URDF & TF Definitions
│   │   └── mesh/               # 3D Meshes
│   └── robot_bringup/          # System Integration
│       ├── config/             # EKF & Navigation Parameters
│       ├── scripts/            # Utility & Odometry Scripts
│       └── launch/             # Master Launch Files
🛠️ Build & Installation
1️⃣ Device Rules Setup
Ensure persistent device naming by running:

bash robot_bringup/scripts/setup_rules.sh
2️⃣ Build the Workspace
colcon build --symlink-install
source install/setup.bash
3️⃣ Start the Robot
start_robot
🛰️ Hardware Components
🔴 LD06 2D LiDAR – 360° planar environment scanning
📷 Orbbec Astra Pro – RGB + Depth perception
🟢 ESP32 (micro-ROS) – Motor control, encoders, and IMU bridge
🍓 Raspberry Pi 5 – Main ROS 2 computation unit
🔗 References & Official Packages
🛠️ Middleware & Communication
micro-ROS Agent https://github.com/micro-ROS/micro-ROS-Agent

micro-ROS Arduino Client https://github.com/micro-ROS/micro_ros_arduino

🛰️ Sensors & Perception
LD06 LiDAR Driver https://github.com/ldrobotSensorTeam/ldlidar_ros2

Orbbec Astra Camera Driver https://github.com/orbbec/ros2_astra_camera

🧭 Localization & Navigation
Robot Localization (EKF) https://github.com/cra-ros-pkg/robot_localization

SLAM Toolbox https://github.com/SteveMacenski/slam_toolbox

Navigation2 (Nav2) https://github.com/ros-navigation/navigation2

🔮 Future Work
Full Navigation2 (Nav2) integration
Autonomous waypoint navigation
Fault detection & sensor redundancy
Real-time monitoring dashboard
Built with ❤️ for ROS 2 Jazzy and Autonomous Robotics AMR Graduation Project – Mechatronics Engineering (2026)