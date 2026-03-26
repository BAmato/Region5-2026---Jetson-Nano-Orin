#!/bin/bash
# ===========================================================================
# run_bench.sh
# One-command bench test launcher. Run this on the Jetson HOST (not inside
# the container). It handles starting the full ROS stack + bench test for you.
#
# Usage:
#   ./run_bench.sh              # full system checks + interactive run prompt
#   ./run_bench.sh --checks     # checks only, no auto-run prompt
#   ./run_bench.sh --autorun    # skip checks, go straight to start-light wait
#   ./run_bench.sh --stop       # stop the running stack
#
# What it does:
#   1. Starts jetson_bringup.launch.py inside the Isaac ROS container
#      (with enable_led_camera=true, enable_path_planner=false)
#   2. Waits for the stack to be ready
#   3. Runs bench_test.py inside the same container
#
# Logs:
#   ROS stack log: ~/logs/bench_<timestamp>.log
#   Live tail:     journalctl -fu mining_mayhem  (if using service)
#                  or screen at the top of this terminal
# ===========================================================================

set -e

# ---------------------------------------------------------------------------
# Config — update these if paths differ
# ---------------------------------------------------------------------------
WORKSPACE="/workspaces/isaac_ros-dev/mining_mayhem_ws"
HOST_WORKSPACE="$HOME/workspaces/isaac_ros-dev"
DOCKER_IMAGE="isaac_ros_dev-aarch64"
CONTAINER_NAME="mining_mayhem_bench"
LOG_DIR="$HOME/logs"
BENCH_SCRIPT="$WORKSPACE/src/mining_mayhem/mining_mayhem/bench_test.py"
ROS_DOMAIN_ID=42

# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------
GREEN='\033[92m'
RED='\033[91m'
YELLOW='\033[93m'
BLUE='\033[94m'
RESET='\033[0m'
BOLD='\033[1m'

info()  { echo -e "  ${BLUE}[INFO]${RESET}  $*"; }
ok()    { echo -e "  ${GREEN}[OK]${RESET}    $*"; }
warn()  { echo -e "  ${YELLOW}[WARN]${RESET}  $*"; }
fail()  { echo -e "  ${RED}[FAIL]${RESET}  $*"; }

# ---------------------------------------------------------------------------
# --stop flag
# ---------------------------------------------------------------------------
if [[ "$1" == "--stop" ]]; then
    info "Stopping bench container..."
    docker stop "$CONTAINER_NAME" 2>/dev/null && ok "Container stopped" \
        || warn "Container was not running"
    exit 0
fi

BENCH_ARGS="${*}"

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------
echo ""
echo -e "${BOLD}${GREEN}Mining Mayhem — Bench Test Launcher${RESET}"
echo "════════════════════════════════════════"
echo ""

# Docker running?
if ! docker info > /dev/null 2>&1; then
    fail "Docker is not running. Start it first."
    exit 1
fi
ok "Docker is running"

# Image exists?
if ! docker images | grep -q "$DOCKER_IMAGE"; then
    fail "Docker image '$DOCKER_IMAGE' not found."
    info "Run the container manually once first:"
    info "  cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common"
    info "  ./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev"
    exit 1
fi
ok "Docker image $DOCKER_IMAGE found"

# Workspace exists on host?
if [ ! -d "$HOST_WORKSPACE" ]; then
    fail "Workspace not found at $HOST_WORKSPACE"
    exit 1
fi
ok "Workspace found at $HOST_WORKSPACE"

# Stop any existing bench container (not the competition one)
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    warn "Stopping existing bench container..."
    docker stop "$CONTAINER_NAME" > /dev/null 2>&1 || true
    sleep 1
fi

mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/bench_$(date +%Y%m%d_%H%M%S).log"
info "Log file: $LOG_FILE"
echo ""

# ---------------------------------------------------------------------------
# Build the startup script that runs inside the container
# ---------------------------------------------------------------------------
# This heredoc is passed to docker run as the command.
# It sources ROS, starts the launch file in the background, waits for
# topics to appear, then runs bench_test.py interactively.

INNER_SCRIPT=$(cat <<'INNEREOF'
#!/bin/bash
set -e

WORKSPACE="/workspaces/isaac_ros-dev/mining_mayhem_ws"
LOG_FILE_INNER="${LOG_FILE_INNER:-/home/mrohms/logs/bench_inner.log}"

echo "=== Bench test inner startup ===" | tee "$LOG_FILE_INNER"
echo "$(date)" | tee -a "$LOG_FILE_INNER"

# Source ROS
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=42

# Wait for roboRIO
ROBORIO_IP="10.0.67.2"
echo "Pinging roboRIO at $ROBORIO_IP..." | tee -a "$LOG_FILE_INNER"
waited=0
while ! ping -c 1 -W 1 "$ROBORIO_IP" > /dev/null 2>&1; do
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge 15 ]; then
        echo "WARNING: roboRIO not reachable after 15s — continuing anyway" \
            | tee -a "$LOG_FILE_INNER"
        break
    fi
done
if ping -c 1 -W 1 "$ROBORIO_IP" > /dev/null 2>&1; then
    echo "roboRIO reachable after ${waited}s" | tee -a "$LOG_FILE_INNER"
fi

# Start the ROS stack in the background
echo "Starting jetson_bringup.launch.py in background..." | tee -a "$LOG_FILE_INNER"
ros2 launch mining_mayhem jetson_bringup.launch.py \
    enable_led_camera:=true \
    enable_path_planner:=false \
    >> "$LOG_FILE_INNER" 2>&1 &

ROS_PID=$!
echo "ROS stack PID: $ROS_PID" | tee -a "$LOG_FILE_INNER"

# Wait for topics to appear (up to 30s)
echo "Waiting for ROS topics to appear..." | tee -a "$LOG_FILE_INNER"
waited=0
while [ $waited -lt 30 ]; do
    topic_count=$(ros2 topic list 2>/dev/null | wc -l)
    if [ "$topic_count" -gt 5 ]; then
        echo "Topics available after ${waited}s (count=$topic_count)" \
            | tee -a "$LOG_FILE_INNER"
        break
    fi
    sleep 1
    waited=$((waited + 1))
done
if [ $waited -ge 30 ]; then
    echo "WARNING: topics still not visible after 30s — proceeding anyway" \
        | tee -a "$LOG_FILE_INNER"
fi

# Small extra settle time
sleep 2

# Run the bench test (interactive — needs TTY)
echo "Launching bench_test.py..." | tee -a "$LOG_FILE_INNER"
python3 /workspaces/isaac_ros-dev/mining_mayhem_ws/src/mining_mayhem/mining_mayhem/bench_test.py $BENCH_ARGS_INNER

# When bench_test exits, also kill the ROS stack
echo "Bench test complete — stopping ROS stack..." | tee -a "$LOG_FILE_INNER"
kill $ROS_PID 2>/dev/null || true
wait $ROS_PID 2>/dev/null || true
echo "Done." | tee -a "$LOG_FILE_INNER"
INNEREOF
)

# ---------------------------------------------------------------------------
# Run the container interactively (needs TTY for the bench_test.py prompt)
# ---------------------------------------------------------------------------
echo -e "${BOLD}Starting bench container...${RESET}"
echo ""

docker run \
    --rm \
    --name "$CONTAINER_NAME" \
    --privileged \
    --network=host \
    --interactive \
    --tty \
    -v "$HOST_WORKSPACE:/workspaces/isaac_ros-dev" \
    -v "/dev:/dev" \
    -v "$LOG_DIR:/home/mrohms/logs" \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -e HOME=/root \
    -e BENCH_ARGS_INNER="$BENCH_ARGS" \
    -e LOG_FILE_INNER="/home/mrohms/logs/bench_$(date +%Y%m%d_%H%M%S)_inner.log" \
    "$DOCKER_IMAGE" \
    /bin/bash -c "$INNER_SCRIPT"

EXIT_CODE=$?
echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "  ${GREEN}${BOLD}Bench session complete.${RESET}"
else
    echo -e "  ${YELLOW}Bench session exited with code $EXIT_CODE.${RESET}"
fi
echo ""
echo -e "  Log saved to: ${BLUE}$LOG_FILE${RESET}"
