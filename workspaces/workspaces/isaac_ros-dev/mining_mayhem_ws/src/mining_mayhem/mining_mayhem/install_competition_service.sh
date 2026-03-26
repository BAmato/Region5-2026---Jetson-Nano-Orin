#!/bin/bash
# ===========================================================================
# install_competition_service.sh
# Run this ONCE on the Jetson host to set up auto-start on boot.
# Run as the mrohms user (not root, not inside container)
# ===========================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="$SCRIPT_DIR/mining_mayhem.service"
START_SCRIPT="$SCRIPT_DIR/competition_start.sh"

echo "=== Installing Mining Mayhem Competition Service ==="
echo ""

# Verify files exist
if [ ! -f "$SERVICE_FILE" ]; then
    echo "ERROR: mining_mayhem.service not found at $SCRIPT_DIR"
    exit 1
fi
if [ ! -f "$START_SCRIPT" ]; then
    echo "ERROR: competition_start.sh not found at $SCRIPT_DIR"
    exit 1
fi

# Verify Docker image exists
echo "Checking Docker image..."
if ! docker images | grep -q "isaac_ros_dev-aarch64"; then
    echo "WARNING: isaac_ros_dev-aarch64 image not found."
    echo "Run the container at least once first:"
    echo "  cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common"
    echo "  ./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev"
    echo ""
    echo "Then re-run this install script."
    exit 1
fi
echo "Docker image found: $(docker images | grep isaac_ros_dev-aarch64 | head -1)"

# Make competition_start.sh executable
chmod +x "$START_SCRIPT"
echo "Made competition_start.sh executable"

# Install systemd service
sudo cp "$SERVICE_FILE" /etc/systemd/system/mining_mayhem.service
sudo systemctl daemon-reload
sudo systemctl enable mining_mayhem.service
echo "Service installed and enabled"

echo ""
echo "=== Installation complete ==="
echo ""
echo "Commands:"
echo "  Start now:    sudo systemctl start mining_mayhem"
echo "  Stop:         sudo systemctl stop mining_mayhem"
echo "  Status:       sudo systemctl status mining_mayhem"
echo "  Live logs:    journalctl -fu mining_mayhem"
echo "  Disable boot: sudo systemctl disable mining_mayhem"
echo ""
echo "To TEST before competition (simulates boot):"
echo "  sudo systemctl start mining_mayhem"
echo "  journalctl -fu mining_mayhem"
echo ""
echo "To SWITCH BACK to dev mode:"
echo "  sudo systemctl stop mining_mayhem"
echo "  sudo systemctl disable mining_mayhem"
echo "  # Then run container manually as usual"
