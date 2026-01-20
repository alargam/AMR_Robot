# 🤖 AMR Autonomous Mobile Robot – ROS 2 Jazzy (2026)

This repository contains the **hardware bringup and full software integration** for an **Autonomous Mobile Robot (AMR)** developed as a **Mechatronics Engineering Graduation Project (2026)**.

The system runs **ROS 2 Jazzy** on a **Raspberry Pi 5** and integrates multiple sensors and controllers to form a complete **SLAM, Localization, and Navigation stack**.

---

## 🧩 System Overview

The AMR platform integrates the following hardware components:

* 🟦 **Orbbec Astra Pro** – RGB-D depth camera
* 🔴 **LD06** – 2D LiDAR (360° scanning)
* 🟢 **ESP32** – Low-level motor control & sensor interface via **micro-ROS**

These components are fused in ROS 2 to enable:

* Real-time perception
* Accurate odometry & localization
* Mapping (SLAM)
* Autonomous navigation (Nav2-ready)

---

## 🚀 Quick Start (Shell Aliases)

To simplify operation and field testing, the following custom aliases are configured:

| Command        | Action                                     |
| -------------- | ------------------------------------------ |
| `start_camera` | Launch Orbbec Astra Pro (RGB + Depth)      |
| `start_lidar`  | Launch LD06 LiDAR (LaserScan)              |
| `start_esp`    | Start micro-ROS Agent (ESP32 bridge)       |
| `check_topics` | List all active ROS 2 topics               |
| `start_all`    | Launch Camera + LiDAR + micro-ROS together |

---

## 🏗️ System Architecture

### 🔁 Data Flow (High Level)

Encoders + IMU → **micro-ROS (ESP32)** → `/odom_raw`
LiDAR → `/scan`
Camera → RGB / Depth Topics

All sensor data is fused using **EKF (robot_localization)** to produce `/odometry/filtered`, which is then used by **SLAM Toolbox** and **Navigation2**.

---

## 🧱 Core ROS 2 Nodes

### 1️⃣ Robot State Publisher (RSP)

**Purpose**

* Publishes the robot TF tree from URDF

**Function**

* Defines all rigid-body relationships
* Example frames:

  * `base_link → lidar_link`
  * `base_link → camera_link`
  * `base_link → imu_link`

---

### 2️⃣ Orbbec Astra Pro Camera Node

**Purpose**

* RGB-D perception using structured light depth sensing

**Data Streams**

* RGB Image
* Depth Image
* Infrared Image

**Key Topics**

* `/camera/color/image_raw`
* `/camera/depth/image_raw`
* `/camera/depth/points`

---

### 3️⃣ micro-ROS Agent (ESP32 Bridge)

**Purpose**

* Real-time communication between ESP32 and ROS 2

**Responsibilities**

* Wheel encoder publishing
* IMU data publishing
* Motor command reception (`cmd_vel`)
* PWM-based motor control

**Transport**

* Serial (USB)

---

### 4️⃣ LD06 LiDAR Node

**Purpose**

* 2D laser scanning for perception and SLAM

**Function**

* Publishes `/scan`
* 360° planar environment sensing

---

### 5️⃣ EKF Node – robot_localization

**Purpose**

* Sensor fusion and state estimation

**Function**

* Fuses wheel encoders and IMU data
* Outputs `/odometry/filtered`

**Standards**

* REP-105 compliant
* TF chain: `map → odom → base_link`

### 6️⃣ SLAM Toolbox

**Purpose**

* 2D mapping and localization

**Function**

* Uses LiDAR scans and EKF-filtered odometry

---

## 📂 Repository Structure

```
AMR_Robot/
├── src/
│   ├── hardware/
│   │   ├── ldlidar_ros2/
│   │   ├── micro-ROS-Agent/
│   │   └── ros2_astra_camera/
│   ├── my_robot_description/
│   │   ├── urdf/
│   │   └── mesh/
│   └── robot_bringup/
│       ├── launch/
│       ├── config/
│       └── scripts/
```

---

## 🛠️ Build & Installation

```bash
cd ~/AMR_Robot
colcon build --symlink-install
source install/setup.bash
```

### Permissions

```bash
sudo usermod -a -G dialout $USER
```

---

## 🔌 Hardware Setup & Udev Rules

### Orbbec Astra Camera

```bash
cd src/hardware/ros2_astra_camera/astra_camera/scripts
sudo bash install.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### LD06 LiDAR

```bash
cd src/hardware/ldlidar_ros2/scripts
sudo bash create_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Creates a fixed device name such as:

```
/dev/ttyUSB_lidar
```

---

## ⏱️ Recommended Startup Workflow

1. Robot State Publisher
2. micro-ROS Agent
3. LiDAR & Camera
4. EKF (robot_localization)
5. SLAM Toolbox
6. Navigation2 (optional)

---

## 🔗 References

* micro-ROS Agent: [https://github.com/micro-ROS/micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent)
* micro-ROS Arduino: [https://github.com/micro-ROS/micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino)
* LD06 LiDAR: [https://github.com/ldrobotSensorTeam/ldlidar_ros2](https://github.com/ldrobotSensorTeam/ldlidar_ros2)
* Orbbec Astra: [https://github.com/orbbec/ros2_astra_camera](https://github.com/orbbec/ros2_astra_camera)
* Navigation2: [https://github.com/ros-navigation/navigation2](https://github.com/ros-navigation/navigation2)
* SLAM Toolbox: [https://github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
* Robot Localization: [https://github.com/cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization)

---

## 🧠 Future Work

* Full Navigation2 integration
* Autonomous waypoint navigation
* Sensor redundancy & diagnostics
* Visualization dashboard

---

> Built with ❤️ for ROS 2 Jazzy & Autonomous Robotics
> AMR Graduation Project – Mechatronics Engineering (2026)
