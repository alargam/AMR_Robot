#!/bin/bash

echo "Configuring AMR Hardware Rules..."
RULES_FILE="99-robot-serial.rules"

# Copy the rules to the system directory
sudo cp ./$RULES_FILE /etc/udev/rules.d/

# Reload the udev rules
echo "Reloading system rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Configuration complete! Please reconnect your LiDAR and ESP32."