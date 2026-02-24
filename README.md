# 🤖 Autonomous Mobile Robot (AMR)
### ROS 2 Jazzy | Real-Time Perception | Embedded Control | Edge AI

An end-to-end **Autonomous Mobile Robot (AMR)** platform developed as a Mechatronics Engineering Graduation Project (2026).

This project integrates real-time perception, sensor fusion, embedded motor control, SLAM, and autonomous navigation into a complete robotic system designed using ROS 2 Jazzy best practices.

---

You can visit the official project website for more details: [https://delivero.site/](https://delivero.site/)

---

## 🎥 Demonstration

Mapping Demo: [link](https://www.youtube.com/shorts/1S7ldnf2kuI)

Autonomous Navigation Demo: [link](https://youtu.be/pdu4tVg_QxQ?si=NQ7DHxntDtQeoHpF)

Real-World Robot Demo [link](https://www.youtube.com/shorts/1S7ldnf2kuI)

---

# 🏗️ System Architecture


| Sensor / Input        | Processing Node / Algorithm      | Output Topic / Interface                                |
|----------------------|---------------------------------|--------------------------------------------------------|
| Camera               | Perception Node (TensorRT)      | `/detections`                                         |
| LiDAR                | SLAM Toolbox                    | `/map`                                                |
| Encoders + IMU       | EKF (Extended Kalman Filter)    | `/odom`                                               |
| Navigation (Nav2)    | Path Planner / Controller       | `/cmd_vel` → micro-ROS (ESP32) → Motor Driver → Wheels |

The system is modular and scalable, separating hardware drivers, robot description, and system integration layers.

---

# 🧠 Real-Time Perception Pipeline

The robot integrates an optimized deep learning perception system:

- Model: YOLOv11n
- Framework: PyTorch → ONNX → TensorRT
- Precision: INT8, FP32
- Deployment: Custom ROS 2 inference node
- Output Topic: `/detections`

## 📊 Inference Performance (Edge Deployment)

| Mode  | FPS | Latency | Hardware |
|-------|-----|----------|----------|
| FP32  | 4–6  FPS  | 160–250 ms | Raspberry Pi 5 |
| INT8  | 12–18 FPS | 55–85 ms   | Raspberry Pi 5 |

Measured on CPU-only deployment, actual FPS may improve with GPU/Edge Accelerator

---

# 🧭 Localization & Sensor Fusion

Localization is implemented using `robot_localization` (EKF):

- IMU + Wheel Encoder fusion
- REP-105 compliant TF tree:


map → odom → base_link


## 📊 EKF & Control Performance

- EKF update rate: <XX Hz>
- Control loop frequency: <XX Hz>
- Odometry drift: <XX% over XX meters>
- Maximum linear velocity: <XX m/s>
- Maximum angular velocity: <XX rad/s>

---

# 🛰️ Navigation Stack

- SLAM Toolbox for real-time mapping
- Navigation2 (Nav2) for path planning
- Dynamic obstacle avoidance
- Waypoint navigation support

## Autonomous Capabilities

- ✅ Mapping
- ✅ Localization
- ✅ Path planning
- ✅ Obstacle avoidance

---

# 🔌 Embedded & Low-Level Control

Low-level control handled via:

- ESP32 running micro-ROS
- Serial communication (`/dev/esp32`)
- Encoder feedback at <XX Hz>
- Closed-loop motor control

## Communication Metrics

- micro-ROS latency: <XX ms average>
- Serial baud rate: <XXXXXX>
- Motor response delay: <XX ms>

---

# 🛠️ Hardware Stack

| Component | Model |
|------------|--------|
| LiDAR | LD06 |
| Camera | Orbbec Astra Pro |
| MCU | ESP32 S3 |
| Main Compute | Raspberry Pi 5 |
| Motor Driver | BTS7960 |
| Battery | 24V |

---

# 📊 System Resource Usage

Measured during full operation:

| Metric | Value |
|--------|--------|
| CPU Usage | 70–85% average |
| RAM Usage | 1.5–2.0 GB|
| Power Consumption | 12–15 W |
| Runtime per charge | 10–15 hours |

---

# 📂 Repository Structure


AMR_Robot/
├── src/
│ ├── hardware/ # Drivers & Communication
│ ├── my_robot_description/ # URDF & TF definitions
│ └── robot_bringup/ # System integration & launch


The architecture follows a clear separation of concerns to ensure maintainability and scalability.

---

# 🚀 Build & Deployment

## 1️⃣ Setup Device Rules

```bash
bash robot_bringup/scripts/setup_rules.sh
```

## 2️⃣ Build Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

3️⃣ Launch Robot
```bash
start_robot
```

## 🧠 Engineering Highlights

```
This project demonstrates:

End-to-end robotics system integration

Real-time AI deployment on edge hardware

ROS 2 modular architecture design

Embedded systems integration via micro-ROS

Performance optimization under hardware constraints

Production-oriented robotics engineering practices
```
---
