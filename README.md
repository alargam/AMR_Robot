# 🤖 AMR Autonomous Mobile Robot - ROS 2 Jazzy (2026)

This repository contains the **hardware bringup and software integration** for an **Autonomous Mobile Robot (AMR)** running **ROS 2 Jazzy** on a **Raspberry Pi 5**.

The system integrates:

* 🟦 Orbbec Astra Pro depth camera
* 🔴 LD06 2D LiDAR
* 🟢 ESP32 (via micro-ROS)

to form a complete **SLAM, Localization, and Navigation stack**.

---

## 🚀 Quick Start (Aliases)

To simplify operation, the following custom aliases are configured:

| Command        | Action                                             |
| -------------- | -------------------------------------------------- |
| `start_camera` | Launches the Orbbec Astra Pro camera (RGB + Depth) |
| `start_lidar`  | Launches the LD06 LiDAR for 360° scanning          |
| `start_esp`    | Starts the micro-ROS Agent (ESP32 communication)   |
| `check_topics` | Lists all active ROS 2 topics                      |
| `start_all`  | Launches Camera + LiDAR + micro-ROS together       |

---

## 🏗️ System Architecture

### Core Nodes

#### 1. Robot State Publisher (RSP)

**Purpose:**

* Reads the URDF file and publishes the TF tree

**Function:**

* Defines physical relationships between frames
* Example: `base_link → lidar_link`, `base_link → camera_link`

---

#### 2. Astra Camera Node (Triple Lens System)

**Purpose:**

* ROS 2 driver for the **Orbbec Astra Pro** depth camera

**Data Streams:**

* **RGB Lens:** Color video for perception
* **Depth Lens:** Distance measurement via structured light
* **IR Lens:** Low-light depth operation

**Key Topics:**

* `/camera/color/image_raw`
* `/camera/depth/image_raw`
* `/camera/depth/points` *(PointCloud2)*

---

#### 3. micro-ROS Agent

**Purpose:**

* Bridge between Raspberry Pi 5 and ESP32

**Function:**

* Encoder data (Odometry)
* IMU data
* Motor control commands (PWM)

---

#### 4. LDLiDAR Node

**Purpose:**

* Processes raw laser scan data from the **LD06 LiDAR**

**Function:**

* Publishes `/scan` topic
* Provides 360° environment perception

---

#### 5. EKF Node (robot_localization)

**Purpose:**

* Sensor fusion

**Function:**

* Fuses encoder + IMU data
* Outputs filtered, drift-reduced pose estimate

---

#### 6. SLAM Toolbox

**Purpose:**

* Mapping & Localization

**Function:**

* Real-time SLAM using LiDAR scans and EKF pose

---

## 📂 Repository Structure

```
AMR_Robot/
├── src/
│   ├── ros2_astra_camera/      # Orbbec Astra Pro driver
│   ├── ldlidar_ros2/           # LD06 LiDAR driver
│   ├── micro-ROS-Agent/        # micro-ROS serial agent
│   ├── my_robot_description/   # URDF, meshes, launch files
│   └── navigation/             # SLAM & localization configs
```

---

## 🛠️ Build & Installation

### Clone & Build

```bash
cd ~/AMR_Robot
colcon build --symlink-install
source install/setup.bash
```

### Permissions

Allow access to serial and USB devices:

```bash
sudo usermod -a -G dialout $USER
```

> 🔄 Reboot or log out/in after this step.

---

## 🔌 Hardware Setup & Udev Rules

### Orbbec Astra Camera

```bash
cd src/ros2_astra_camera/astra_camera/scripts
sudo bash install.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### LD06 LiDAR

```bash
cd src/ldlidar_ros2/scripts
sudo bash create_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Creates a fixed device symlink such as:

```
/dev/ttyUSB_lidar
```

---

## ⏱️ Recommended Startup Workflow

1. Start Robot State Publisher (URDF & TF)
2. Start micro-ROS Agent (ESP32)
3. Start LiDAR and Camera
4. Launch EKF (sensor fusion)
5. Launch SLAM Toolbox (after TF stabilizes)

---
## 🔗 References & Official Drivers

This project utilizes several **official ROS 2 drivers and stacks** to enable autonomous navigation and sensor fusion.  
Below are the key repositories and documentation sources used in this project.

---

### 🛠️ Middleware & Communication

- 🔌 **micro-ROS Agent**  
  Bridge between the ESP32-S3 and the ROS 2 workspace  
  👉 https://github.com/micro-ROS/micro-ROS-Agent

- 🔌 **micro-ROS Arduino (Client)**  
  micro-ROS client library for Arduino-based microcontrollers  
  👉 https://github.com/micro-ROS/micro_ros_arduino

---

### 🛰️ Perception & Sensors

- 🔴 **LD06 LiDAR Driver**  
  Official ROS 2 driver for the LDROBOT LD06 LiDAR sensor  
  👉 https://github.com/ldrobotSensorTeam/ldlidar_ros2

- 📷 **Orbbec Astra Camera**  
  ROS 2 driver for Astra Series depth cameras  
  👉 https://github.com/orbbec/ros2_astra_camera

---

### 🧭 Navigation & Localization

- 🗺️ **Navigation 2 (Nav2) Stack**  
  Professional-grade navigation framework for ROS 2  
  👉 https://github.com/ros-navigation/navigation2

- 🧭 **SLAM Toolbox**  
  Advanced 2D SLAM for map building and localization  
  👉 https://github.com/SteveMacenski/slam_toolbox

- 📐 **Robot Localization**  
  Extended Kalman Filter (EKF) for fusing odometry and sensor data  
  👉 https://github.com/cra-ros-pkg/robot_localization

---

## 🧠 Future Work

* Navigation2 integration
* Autonomous waypoint navigation
* Sensor redundancy & fault detection
* Visualization dashboard

---

> Built with ❤️ for ROS 2 Jazzy & Autonomous Robotics
