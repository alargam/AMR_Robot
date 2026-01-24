# 🛠️ Falcon AMR - Complete Hardware Specifications

This document provides a technical overview of the Falcon AMR hardware architecture, electrical connections, and physical configuration.

---

## 🏗️ 1. Mechanical Structure
The robot is based on a 4-wheel drive (4WD) chassis designed for indoor autonomous navigation.

| Feature | Specification |
| :--- | :--- |
| **Chassis Type** | 4-Wheel Differential Drive |
| **Ground Clearance** | `0.02 m` (2 cm) - Ground to bottom of chassis |
| **Wheel Diameter** | `0.16 m` (16 cm) |
| **Wheel Radius** | `0.07 m` (7 cm) |
| **Wheel Track (Separation)** | `0.44 m` (44 cm) |
| **Total Weight** | Approx. `9.0 kg` |

---

## ⚡ 2. Electrical & Control System
The system uses a hierarchical control strategy: **Raspberry Pi 5** for high-level processing and **ESP32** for low-level motor control.

### **Core Components**
* **High-Level Controller:** Raspberry Pi 5 (8GB RAM) running ROS 2 Jazzy.
* **Low-Level Controller:** ESP32 (connected via Micro-ROS).
* **Power Source:** [Insert Battery Type, e.g., 12V Li-Po]
* **Motor Drivers:** [Insert Driver Type, e.g., BTS7960 or L298N]

---

## 📡 3. Sensors & Connectivity
The robot is equipped with a suite of sensors for environment perception.

| Sensor | Purpose | Connection Type | Baudrate / Port |
| :--- | :--- | :--- | :--- |
| **LiDAR LD06** | 2D SLAM & Obstacle Avoidance | Serial Over USB | `230400` / `/dev/ldlidar` |
| **ESP32 (Odom)** | Encoder Data & Motor PWM | Micro-ROS (Serial) | `115200` / `/dev/esp32` |
| **Astra Pro Max** | 3D Depth & RGB Vision | USB 3.0 | N/A |
| **GPS Module** | Outdoor Global Positioning | Serial | N/A |

---

## 📏 4. Sensor Mounting Heights (TF Offsets)
All heights are measured from the **Ground Surface** to the center of the sensor's lens/scanning plane.



* **LiDAR Scanning Plane:** `0.38 m` (38 cm)
* **Camera Optical Center:** `0.22 m` (22 cm)
* **GPS Antenna:** `0.31 m` (31 cm)

---

## 📝 5. Hardware Implementation Checklist
- [x] **Persistent USB Rules:** Created udev rules for `/dev/ldlidar` and `/dev/esp32` to prevent port swapping.
- [x] **LiDAR Configuration:** Serial baudrate confirmed at `230400`.
- [x] **Micro-ROS Connectivity:** Agent baudrate confirmed at `115200`.
- [x] **URDF Accuracy:** Updated `base_joint` with `0.02m` ground clearance and verified sensor Z-offsets.
- [ ] **Motor Polarity:** Verified that positive `cmd_vel` moves the robot forward.
- [ ] **Encoder Phase:** Verified that forward movement increases the tick count in `/odom`.