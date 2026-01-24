# 🛠️ Falcon AMR – Debugging & Solutions Log

This document records the critical hardware and software issues encountered during the development of **Falcon AMR** and the precise solutions applied to resolve them.

---

## 1. LiDAR Node Crash (Invalid Topic Name)

### 🚨 The Problem

The LiDAR node (`ldlidar_ros2_node`) would start, connect to the serial port successfully, and then immediately crash with the error:

```
Invalid topic name: topic name must not be empty string
```

### ✅ The Solution

The launch file was passing a parameter named `topic_name`, but the LD06 driver expects `laser_scan_topic_name`.

**Fix:** Updated `hardware.launch.py` to use the correct parameter key:

```python
parameters=[{
    'laser_scan_topic_name': 'scan',
    'point_cloud_2d_topic_name': 'pointcloud',
}]
```

---

## 2. Odometry Drift (Incorrect Wheel Diameter)

### 🚨 The Problem

The robot was reporting incorrect distances in `/odom`. Moving the robot 1 meter physically resulted in a different distance in RViz.

### ✅ The Solution

The URDF and Gazebo plugins were configured with a wheel diameter of **0.16 m** (radius 0.08 m), while the physical measurement confirmed the wheels are **0.14 m**.

**Fix:**

1. Updated URDF collision radius to **0.07 m**
2. Updated `nav2_params.yaml` and Gazebo plugin `wheel_diameter` to **0.14 m**

---

## 3. TF Tree Disconnect (base_link vs base_footprint)

### 🚨 The Problem

Nav2 and SLAM Toolbox were reporting `LookupTransform` errors. The robot model in RViz appeared to jump or was not connected correctly to the map.

### ✅ The Solution

The EKF (Robot Localization) was publishing `odom → base_link`, while Nav2 and SLAM were expecting `base_footprint`.

**Fix:** Unified the entire system to use `base_link` as the primary robot frame in:

* `ekf.yaml`
* `nav2_params.yaml`
* `slam_toolbox.yaml`

**Final TF Chain:**

```
map → odom → base_link → sensors
```

---

## 4. ESP32 Time Synchronization (TF_OLD_DATA)

### 🚨 The Problem

ROS 2 rejected messages from the ESP32 with:

```
Message contains older data than latest transform
```

This occurred because the ESP32 clock (`millis()`) was not synchronized with the Raspberry Pi Unix time.

### ✅ The Solution

Implemented **micro-ROS time synchronization** on the ESP32.

**Fix:**

* Used `rmw_uros_sync_session(1000)` in `setup()`
* Timestamped messages using `rmw_uros_epoch_millis()`

```cpp
odom_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
```

---

## 5. Serial Port Permission Denied

### 🚨 The Problem

ROS nodes could not open `/dev/ttyUSB0` or `/dev/ttyACM0` without `sudo`, and device names would randomly swap (e.g., LiDAR becoming `ttyUSB1`).

### ✅ The Solution

Created **persistent udev rules** to lock device names and grant permanent permissions.

**Fix:** Created `/etc/udev/rules.d/99-robot-serial.rules`

```bash
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", SYMLINK+="ldlidar", MODE="0666"
KERNEL=="ttyACM*", ATTRS{idVendor}=="303a", SYMLINK+="esp32", MODE="0666"
```

---

## 6. micro-ROS Compilation Errors (Undeclared Objects)

### 🚨 The Problem

Compilation failed with:

```
'odom_msg' was not declared in this scope
```

### ✅ The Solution

In C++, variables must be declared **before** the functions that use them.

**Fix:** Reordered the `.ino` file so all micro-ROS objects (`rcl_publisher_t`, messages, etc.) are declared at the top of the file, before `publishOdometry()`.

---

## 7. EKF NaN / Inf State Detection

### 🚨 The Problem

The EKF node crashed with:

```
Critical Error: NaNs were detected in the output state
```

### ✅ The Solution

The ESP32 was sending invalid `dt` values (0 or NaN) during the first loop iteration, causing division by zero in velocity calculations.

**Fix:** Added a safety check in the ESP32 code:

```cpp
if (dt <= 0.0 || isnan(dt)) return; // Skip invalid calculation
```

---

## 8. LiDAR Static Transform Jitter

### 🚨 The Problem

The LiDAR scan in RViz was shaking or misaligned relative to the robot body.

### ✅ The Solution

There was a conflict between:

* `static_transform_publisher` in the launch file
* The LiDAR joint defined in the URDF

**Fix:** Removed the manual static transform from the launch file and relied exclusively on `robot_state_publisher` to publish the sensor transform from the URDF.

---

## 📌 Next Step

Would you like help pushing this README to your GitHub repository (including commit and push commands)?
