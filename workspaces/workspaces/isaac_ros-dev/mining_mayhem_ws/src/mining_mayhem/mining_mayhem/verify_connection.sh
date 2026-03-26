#!/bin/bash
# ===========================================================================
# verify_connection.sh
# Pre-match / pre-test connectivity and environment verification.
# Run this before launching ROS nodes.
#
# Exit code 0 = all checks passed
# Exit code 1 = one or more checks failed
# ===========================================================================

ROBORIO_IP="10.0.67.2"
JETSON_IP="10.0.67.5"
RX_PORT=5800
TX_PORT=5801
EXPECTED_DOMAIN_ID=42

PASS="[  PASS  ]"
FAIL="[  FAIL  ]"
WARN="[  WARN  ]"

overall=0

pad() {
    printf "%-52s" "$1"
}

check() {
    local label="$1"
    local result="$2"   # 0=pass, 1=fail, 2=warn
    local detail="$3"

    pad "$label"
    if   [ "$result" -eq 0 ]; then echo "$PASS  $detail"
    elif [ "$result" -eq 2 ]; then echo "$WARN  $detail"
    else                           echo "$FAIL  $detail"; overall=1
    fi
}

echo ""
echo "========================================================"
echo "  Mining Mayhem — Pre-Launch Verification"
echo "  $(date)"
echo "========================================================"
echo ""

# 1. Ethernet interface up
ETH_IF=""
for iface in $(ls /sys/class/net/); do
    if [[ "$iface" == lo* || "$iface" == wl* || "$iface" == docker* || "$iface" == veth* || "$iface" == br-* ]]; then continue; fi
    if [ -e "/sys/class/net/$iface/device" ]; then ETH_IF="$iface"; break; fi
done

if [ -z "$ETH_IF" ]; then
    check "Ethernet interface detected" 1 "no physical interface found"
else
    state=$(cat /sys/class/net/$ETH_IF/operstate 2>/dev/null)
    if [ "$state" == "up" ]; then
        check "Ethernet interface ($ETH_IF) up" 0 "state=$state"
    else
        check "Ethernet interface ($ETH_IF) up" 1 "state=$state — is cable plugged in?"
    fi
fi

# 2. Jetson has correct IP
if ip addr show 2>/dev/null | grep -q "inet $JETSON_IP"; then
    check "Jetson IP $JETSON_IP assigned" 0 ""
else
    check "Jetson IP $JETSON_IP assigned" 1 "run setup_jetson_network.sh"
fi

# 3. roboRIO pingable
if ping -c 3 -W 1 "$ROBORIO_IP" > /dev/null 2>&1; then
    rtt=$(ping -c 3 -W 1 "$ROBORIO_IP" 2>/dev/null | tail -1 | awk -F'/' '{print $5}' 2>/dev/null)
    check "roboRIO ($ROBORIO_IP) pingable" 0 "avg RTT=${rtt}ms"
else
    check "roboRIO ($ROBORIO_IP) pingable" 1 "check cable, roboRIO power, and IP"
fi

# 4. UDP RX port 5800 free
if ss -ulnp 2>/dev/null | grep -q ":$RX_PORT "; then
    pid=$(ss -ulnp 2>/dev/null | grep ":$RX_PORT " | grep -oP 'pid=\K[0-9]+' | head -1)
    check "UDP port $RX_PORT (RX) available" 1 "already in use by PID $pid — kill existing ROS nodes"
else
    check "UDP port $RX_PORT (RX) available" 0 ""
fi

# 5. UDP TX port 5801 free on roboRIO side (we can only check local)
if ss -ulnp 2>/dev/null | grep -q ":$TX_PORT "; then
    check "UDP port $TX_PORT (TX) available" 2 "in use locally — may be fine if it's the bridge node"
else
    check "UDP port $TX_PORT (TX) available" 0 ""
fi

# 6. ROS_DOMAIN_ID
if [ "$ROS_DOMAIN_ID" == "$EXPECTED_DOMAIN_ID" ]; then
    check "ROS_DOMAIN_ID=$EXPECTED_DOMAIN_ID" 0 ""
else
    check "ROS_DOMAIN_ID=$EXPECTED_DOMAIN_ID" 1 "currently='${ROS_DOMAIN_ID:-unset}' — run: export ROS_DOMAIN_ID=42"
fi

# 7. No conflicting /dev/ttyACM*
if ls /dev/ttyACM* 2>/dev/null | grep -q .; then
    devs=$(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ')
    check "No /dev/ttyACM* serial devices" 2 "found: $devs (old serial config?)"
else
    check "No /dev/ttyACM* serial devices" 0 ""
fi

# 8. Docker running (for competition mode)
if docker info > /dev/null 2>&1; then
    check "Docker daemon running" 0 ""
else
    check "Docker daemon running" 2 "not running — start with: sudo systemctl start docker"
fi

# 9. ROS 2 sourced
if command -v ros2 > /dev/null 2>&1; then
    ros2_ver=$(ros2 --version 2>/dev/null | head -1)
    check "ROS 2 available (ros2 command)" 0 "$ros2_ver"
else
    check "ROS 2 available (ros2 command)" 1 "source /opt/ros/humble/setup.bash"
fi

echo ""
echo "========================================================"
if [ "$overall" -eq 0 ]; then
    echo "  Result: ALL CHECKS PASSED — ready to launch"
else
    echo "  Result: ONE OR MORE CHECKS FAILED — resolve before launching"
fi
echo "========================================================"
echo ""

exit $overall
