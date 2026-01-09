# AMR Robot Bringup - ROS 2 Jazzy

This repository contains the ROS 2 Jazzy bringup setup for the AMR Robot.  
It integrates LiDAR, ESP32 (Encoders + IMU + ToF), and provides SLAM, EKF, and visualization via Foxglove.

---

## **Project Structure**

- **launch/**: ROS 2 launch files
  - `main_launch.py` → Starts all nodes (RSP, LiDAR, micro-ROS, EKF, SLAM, rosbridge, rosapi)
  - `rsp.launch.py` → Launches Robot State Publisher (URDF + TF frames)
- **config/**: YAML configuration files
  - `lidar_config.yaml` → LiDAR node parameters
  - `ekf.yaml` → EKF node parameters
  - `slam_params.yaml` → SLAM toolbox parameters
- **urdf/**: Robot URDF files

> **Note:** YAML files contain **Node parameters only**. Static transforms **must be passed directly in Launch files**.

---

## **Nodes Overview**

### **1. Robot State Publisher (`rsp`)**
- **Package:** `robot_state_publisher`
- **Purpose:** Reads URDF and publishes TF frames for all robot links.
- **Inputs:** URDF
- **Outputs:** TF frames (e.g., `base_link`, `lidar_link`, `imu_link`)
- **Used by:** LiDAR visualization, SLAM, Navigation, RViz

---

### **2. micro-ROS Agent (`micro_ros_agent`)**
- **Package:** `micro_ros_agent`
- **Purpose:** Connects ESP32 (or MCU) sensors to ROS 2 via Serial/USB.
- **Inputs:** Serial data from ESP32
  - Encoders → `/odom`
  - IMU → `/imu/data`
  - ToF → `/tof/scan` (if used)
- **Outputs:** ROS 2 topics
- **Used by:** EKF, TF, SLAM, Navigation

---

### **3. LiDAR Node (`ldlidar_node`)**
- **Package:** `ldlidar_ros2`
- **Purpose:** Reads LDLiDAR LD06 and publishes LaserScan + PointCloud2.
- **Inputs:** LiDAR data
- **Outputs:** `/scan`, `/pointcloud2d`
- **Used by:** SLAM, Navigation, RViz
- **Parameters:** `port_name`, `baudrate`, `frame_id`, `topic names`, `scan direction`, `angle crop`

---

### **4. EKF Node (`ekf_filter_node`)**
- **Package:** `robot_localization`
- **Purpose:** Fuses Encoders + IMU to provide a filtered pose estimate.
- **Inputs:** `/odom`, `/imu/data`
- **Outputs:** `/odom_filtered` (pose: x, y, z, yaw, etc.)
- **Used by:** SLAM, Navigation, Motion Control

---

### **5. SLAM Toolbox (`slam_toolbox`)**
- **Package:** `slam_toolbox`
- **Purpose:** Builds a real-time map and localizes the robot.
- **Inputs:** `/scan` from LiDAR, EKF pose
- **Outputs:** `/map`, TF frames
- **Used by:** Navigation, RViz

---

### **6. ROSBridge (`rosbridge_websocket`)**
- **Package:** `rosbridge_server`
- **Purpose:** Provides WebSocket interface to ROS 2.
- **Inputs:** ROS topics
- **Outputs:** WebSocket server (default port 9090)
- **Used by:** Foxglove Studio, Web visualization tools

---

### **7. ROSAPI (`rosapi_node`)**
- **Package:** `rosapi`
- **Purpose:** Provides information about ROS 2 system for rosbridge clients.
- **Inputs:** ROS system info
- **Outputs:** Service interface for topics, parameters, services
- **Used by:** Foxglove Studio, Web clients

---

## **Notes on Launch vs Config**

- **YAML config files** contain Node parameters only. Example: LiDAR port, baudrate, topic names, frame_id.
- **Static transforms** (e.g., `base_link` → `lidar_link`) **must be passed in Launch files directly**. YAML does **not** work for static_transform_publisher.
- Launch files are responsible for:
  - Starting nodes
  - Passing YAML parameters
  - Passing static transforms as arguments
  - Handling timing (e.g., delayed SLAM startup)

---

## **Recommended Workflow**

1. **Start RSP** (URDF + TF)
2. **Start micro-ROS Agent** → ensures ESP32 topics are available
3. **Start LiDAR Node** → publishes `/scan`
4. **Start EKF** → fuses `/odom` + `/imu/data`
5. **Start SLAM Toolbox** → after a small delay (~3 sec) to ensure stable TF
6. **Optional:** Start `rosbridge_websocket` + `rosapi` for visualization in Foxglove Studio

---

## **References**

- ROS 2 Documentation: [https://docs.ros.org](https://docs.ros.org)
- LDLiDAR ROS2 Driver: [https://github.com/LDLidar/ldlidar_ros2](https://github.com/LDLidar/ldlidar_ros2)
- robot_localization Package: [https://github.com/cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- SLAM Toolbox: [https://github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

---

