#!/bin/bash

# --- AMR Project: Micro-ROS Agent Starter ---
# This script launches the Micro-ROS Agent specifically for the ESP32 connection.
# The ESP32 is connected to the Raspberry Pi 5 via USB port /dev/ttyUSB1

# Define connection parameters
PORT="/dev/ttyUSB0"
BAUD="115200"

echo "-----------------------------------------------"
echo "Initializing AMR Micro-ROS Agent..."
echo "Target Port: $PORT"
echo "Baud Rate: $BAUD"
echo "-----------------------------------------------"

# Ensure the port is accessible
sudo chmod 666 $PORT

# Run the Micro-ROS Agent
# Using the serial transport protocol
ros2 run micro_ros_agent micro_ros_agent serial --dev $PORT -b $BAUD