#!/bin/bash
# ===========================================================================
# setup_jetson_network.sh
# Configures Jetson Ethernet for direct roboRIO connection.
# Safe to run multiple times.
#
# Result:
#   Jetson:  10.0.67.5/24
#   roboRIO: 10.0.67.2  (already set on roboRIO web dashboard)
# ===========================================================================

set -e

JETSON_IP="10.0.67.5"
ROBORIO_IP="10.0.67.2"
SUBNET="24"
CONNECTION_NAME="roborio-direct"
ETH_IF="enP8p1s0"

echo "=== Jetson Network Setup ==="
echo "Ethernet interface: $ETH_IF"
echo "Jetson IP:  $JETSON_IP/$SUBNET"
echo "roboRIO IP: $ROBORIO_IP"
echo ""

# ---------------------------------------------------------------------------
# Verify interface exists
# ---------------------------------------------------------------------------
if ! ip link show "$ETH_IF" > /dev/null 2>&1; then
    echo "ERROR: Interface $ETH_IF not found."
    echo "Available interfaces:"
    ip link show | grep -E '^[0-9]+:' | awk '{print $2}' | tr -d ':'
    exit 1
fi

echo "Interface $ETH_IF found  ✓"

# ---------------------------------------------------------------------------
# Remove existing connection if present (idempotent)
# ---------------------------------------------------------------------------
if nmcli connection show "$CONNECTION_NAME" &>/dev/null; then
    echo "Removing existing connection '$CONNECTION_NAME'..."
    nmcli connection delete "$CONNECTION_NAME"
fi

# ---------------------------------------------------------------------------
# Create new static IP connection
# ---------------------------------------------------------------------------
echo "Creating static IP connection..."
nmcli connection add \
    type ethernet \
    con-name "$CONNECTION_NAME" \
    ifname "$ETH_IF" \
    ipv4.method manual \
    ipv4.addresses "$JETSON_IP/$SUBNET" \
    ipv4.gateway "" \
    ipv4.dns "" \
    ipv6.method disabled

# ---------------------------------------------------------------------------
# Bring connection up
# ---------------------------------------------------------------------------
echo "Bringing connection up..."
nmcli connection up "$CONNECTION_NAME"

# Give interface a moment to come up
sleep 2

# ---------------------------------------------------------------------------
# Verify IP was assigned
# ---------------------------------------------------------------------------
ASSIGNED=$(ip addr show "$ETH_IF" | grep "inet $JETSON_IP" | awk '{print $2}')
if [ -z "$ASSIGNED" ]; then
    echo "WARNING: IP $JETSON_IP does not appear to be assigned yet."
    echo "Current addresses on $ETH_IF:"
    ip addr show "$ETH_IF"
else
    echo "IP assigned: $ASSIGNED  ✓"
fi

echo ""

# ---------------------------------------------------------------------------
# Ping roboRIO
# ---------------------------------------------------------------------------
echo "Pinging roboRIO at $ROBORIO_IP..."
if ping -c 3 -W 2 "$ROBORIO_IP" > /dev/null 2>&1; then
    echo "roboRIO reachable  ✓"
else
    echo "roboRIO NOT reachable — check:"
    echo "  1. Cat6 cable is plugged into both roboRIO Ethernet and Jetson Ethernet"
    echo "  2. roboRIO is powered on"
    echo "  3. roboRIO static IP is set to $ROBORIO_IP"
fi

echo ""
echo "=== Setup complete ==="
echo ""
echo "Before launching ROS nodes, ensure this is set:"
echo "  export ROS_DOMAIN_ID=42"
echo ""
echo "To make ROS_DOMAIN_ID permanent:"
echo "  echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc"
