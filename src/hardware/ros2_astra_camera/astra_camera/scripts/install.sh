#!/bin/bash

# 1. Check if user is root/running with sudo
if [ "$(id -u)" -ne 0 ]; then
    echo "Error: Please run this script with sudo."
    exit 1
fi

# 2. Resolve script directory paths
SCRIPT_PATH=$(cd "$(dirname "$0")" && pwd)
RULES_FILE="56-orbbec-usb.rules"
TARGET_DIR="/etc/udev/rules.d"

# 3. Check for OS compatibility
if [ "$(uname -s)" == "Darwin" ]; then
    echo "UDEV rules are not required for macOS."
    exit 0
fi

# 4. Verify the rules file exists before copying
if [ ! -f "${SCRIPT_PATH}/${RULES_FILE}" ]; then
    echo "Error: ${RULES_FILE} not found in ${SCRIPT_PATH}."
    exit 1
fi

# 5. Install UDEV rules
cp "${SCRIPT_PATH}/${RULES_FILE}" "${TARGET_DIR}/"
echo "USB rules installed at ${TARGET_DIR}/${RULES_FILE}"

# 6. RELOAD UDEV SERVICE (Critical for AMR immediate use)
echo "Reloading UDEV rules..."
udevadm control --reload-rules
udevadm trigger

echo "Setup complete. Please unplug and replug your Astra camera."