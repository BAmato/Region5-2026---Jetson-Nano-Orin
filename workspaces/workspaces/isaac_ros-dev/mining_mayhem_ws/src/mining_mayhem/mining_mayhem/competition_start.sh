#!/bin/bash
# ===========================================================================
# competition_start.sh
# Runs INSIDE the Isaac ROS container on boot.
# Starts the full ROS stack automatically.
#
# Called by: mining_mayhem.service via docker run
# ===========================================================================

set -e

WORKSPACE="/workspaces/isaac_ros-dev/mining_mayhem_ws"
LOG_DIR="/home/mrohms/logs"
LOG_FILE="$LOG_DIR/competition_$(date +%Y%m%d_%H%M%S).log"

mkdir -p "$LOG_DIR"

echo "=== Mining Mayhem Competition Start ===" | tee "$LOG_FILE"
echo "$(date)" | tee -a "$LOG_FILE"

# Source ROS
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=42

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" | tee -a "$LOG_FILE"

# Wait for roboRIO Ethernet to be reachable
ROBORIO_IP="10.0.67.2"
MAX_WAIT=30
waited=0

echo "Waiting for roboRIO at $ROBORIO_IP..." | tee -a "$LOG_FILE"
while ! ping -c 1 -W 1 "$ROBORIO_IP" > /dev/null 2>&1; do
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge "$MAX_WAIT" ]; then
        echo "WARNING: roboRIO not reachable after ${MAX_WAIT}s — launching anyway" | tee -a "$LOG_FILE"
        break
    fi
done

if ping -c 1 -W 1 "$ROBORIO_IP" > /dev/null 2>&1; then
    echo "roboRIO reachable after ${waited}s" | tee -a "$LOG_FILE"
fi

echo "Launching jetson_bringup.launch.py..." | tee -a "$LOG_FILE"

# Launch everything — redirect to log
exec ros2 launch mining_mayhem jetson_bringup.launch.py 2>&1 | tee -a "$LOG_FILE"
