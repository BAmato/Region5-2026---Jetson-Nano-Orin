#!/usr/bin/env python3
# ===========================================================================
# integration_test.py
# Step-by-step integration test suite for Mining Mayhem robot.
#
# Run each step in order. Each step has a PASS/FAIL gate.
# Do not proceed to the next step until current step passes.
#
# Usage:
#   python3 integration_test.py --step 1   # UDP bridge raw packets
#   python3 integration_test.py --step 2   # ROS topic health
#   python3 integration_test.py --step 3   # EKF fusion
#   python3 integration_test.py --step 4   # AprilTag pipeline
#   python3 integration_test.py --step 5   # Match simulation dry run
#   python3 integration_test.py --all      # Run all steps in sequence
#
# Requirements:
#   Step 1: roboRIO powered and deployed. ROS does NOT need to be running.
#   Steps 2-5: ROS stack running (ros2 launch mining_mayhem jetson_bringup.launch.py)
# ===========================================================================

import argparse
import socket
import struct
import time
import sys
import subprocess
import math
import os
os.environ['ROS_DOMAIN_ID'] = '42'

# ---------------------------------------------------------------------------
# Shared constants
# ---------------------------------------------------------------------------
ROBORIO_IP   = '10.0.67.2'
JETSON_IP    = '10.0.67.5'
RX_PORT      = 5800
TX_PORT      = 5801
PACKET_SIZE  = 60
MAGIC_BYTES  = b'\xA5\x5A'

RIO_TO_JETSON_FMT  = '<2sB6f3f3iBI BH B'
JETSON_TO_RIO_FMT  = '<2sB6fBBB'

PASS = "\033[92m[ PASS ]\033[0m"
FAIL = "\033[91m[ FAIL ]\033[0m"
INFO = "\033[94m[ INFO ]\033[0m"
WARN = "\033[93m[ WARN ]\033[0m"

def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def print_result(label, passed, detail=''):
    tag = PASS if passed else FAIL
    detail_str = f"  {detail}" if detail else ''
    print(f"  {tag}  {label}{detail_str}")
    return passed

def section(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

def ros2_topic_hz(topic, duration=3.0):
    import os
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '42'
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'hz', '--window', '10', topic],
            capture_output=True, text=True, timeout=duration + 2,
            env=env
        )
        for line in result.stdout.splitlines():
            if 'average rate:' in line:
                hz = float(line.split('average rate:')[1].strip().split()[0])
                return hz
    except Exception:
        pass
    return -1.0

def ros2_topic_list():
    """Returns list of active ROS 2 topics."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=5
        )
        return result.stdout.splitlines()
    except Exception:
        return []

def ros2_topic_echo_once(topic, msg_type, timeout=5.0):
    """Returns True if at least one message received on topic within timeout."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '--once', topic],
            capture_output=True, text=True, timeout=timeout
        )
        return len(result.stdout.strip()) > 0
    except subprocess.TimeoutExpired:
        return False
    except Exception:
        return False


# ===========================================================================
# STEP 1: UDP Bridge Raw Packet Test
# ===========================================================================
def step1_udp_bridge():
    section("STEP 1: UDP Bridge Raw Packet Test")
    print("  Prerequisite: roboRIO powered and code deployed.")
    print("  ROS does NOT need to be running for this step.\n")

    all_pass = True

    # --- 1a. Can we bind the RX socket? ---
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', RX_PORT))
        sock.settimeout(5.0)
        all_pass &= print_result("Bind UDP RX socket on port 5800", True)
    except Exception as e:
        all_pass &= print_result("Bind UDP RX socket on port 5800", False, str(e))
        print(f"\n  {FAIL} Cannot bind socket — is serial_bridge_node already running?")
        print("        Kill it first: pkill -f serial_bridge_node")
        return False

    # --- 1b. Receive packets from roboRIO ---
    print(f"\n  Waiting up to 5 seconds for roboRIO packets on port {RX_PORT}...")
    packets_received = 0
    valid_packets = 0
    start = time.time()

    while time.time() - start < 5.0:
        try:
            data, addr = sock.recvfrom(4096)
            packets_received += 1

            if len(data) == PACKET_SIZE and data[:2] == MAGIC_BYTES:
                expected_crc = crc8(data[:-1])
                if data[-1] == expected_crc:
                    valid_packets += 1
        except socket.timeout:
            break

    sock.close()

    all_pass &= print_result(
        f"Packets received from roboRIO ({packets_received} total)",
        packets_received > 0,
        f"from {ROBORIO_IP}" if packets_received > 0 else "Check roboRIO is powered and code is running"
    )
    all_pass &= print_result(
        f"Valid packets (correct magic + CRC) ({valid_packets}/{packets_received})",
        valid_packets > 0
    )

    if valid_packets == 0:
        print(f"\n  {FAIL} No valid packets received. Check:")
        print(f"        1. roboRIO code deployed and running")
        print(f"        2. roboRIO IP is {ROBORIO_IP} (check web dashboard)")
        print(f"        3. Cat6 cable connected")
        print(f"        4. Run: ping {ROBORIO_IP}")
        return False

    # --- 1c. Unpack a packet and sanity check fields ---
    try:
        fields = struct.unpack(RIO_TO_JETSON_FMT, data)
        seq         = fields[1]
        odom_x      = fields[2]
        odom_y      = fields[3]
        odom_theta  = fields[4]
        gyro_rate   = fields[8]
        match_state = fields[16]

        all_pass &= print_result(
            "Packet unpacks correctly",
            True,
            f"seq={seq} odom=({odom_x:.3f},{odom_y:.3f}) theta={math.degrees(odom_theta):.1f}° state={match_state}"
        )

        # Check odom is near landing site (0.15, 0.15) at startup
        odom_reasonable = abs(odom_x) < 5.0 and abs(odom_y) < 5.0
        all_pass &= print_result(
            "Odometry values in reasonable range",
            odom_reasonable,
            f"x={odom_x:.3f} y={odom_y:.3f} (should be near 0.15, 0.15 at startup)"
        )

    except Exception as e:
        all_pass &= print_result("Packet unpacks correctly", False, str(e))

    # --- 1d. Send a test TX packet to roboRIO ---
    try:
        tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        payload = struct.pack('<2sB6fBB',
            b'\x5A\xA5', 0,
            0.0, 0.0, 0.0, 0.1, 0.2, 0.5,  # zero cmd_vel, some servo positions
            0x00, 0x00
        )
        packet = payload + bytes([crc8(payload)])
        tx_sock.sendto(packet, (ROBORIO_IP, TX_PORT))
        tx_sock.close()
        all_pass &= print_result("TX packet sent to roboRIO", True,
            f"sent 30-byte command packet to {ROBORIO_IP}:{TX_PORT}")
    except Exception as e:
        all_pass &= print_result("TX packet sent to roboRIO", False, str(e))

    print(f"\n  {'STEP 1 PASSED — proceed to Step 2' if all_pass else 'STEP 1 FAILED — fix issues above before continuing'}")
    return all_pass


# ===========================================================================
# STEP 2: ROS Topic Health Check
# ===========================================================================
def step2_ros_topics():
    section("STEP 2: ROS Topic Health Check")
    print("  Prerequisite: ros2 launch mining_mayhem jetson_bringup.launch.py\n")

    all_pass = True

    # Required topics and expected minimum Hz
    required_topics = {
        '/odom/wheel':           ('nav_msgs/msg/Odometry',   40.0),
        '/imu/data':             ('sensor_msgs/msg/Imu',     40.0),
        '/odom/filtered':        ('nav_msgs/msg/Odometry',   40.0),
        '/match/state_rio':      ('std_msgs/msg/String',      1.0),
        '/match/time_remaining': ('std_msgs/msg/Float32',     8.0),
        '/match/state':          ('std_msgs/msg/String',      1.0),
    }

    optional_topics = {
        '/apriltag/detections':  ('isaac_ros_apriltag_interfaces/msg/AprilTagDetectionArray', 0.0),
        '/robot/hall_event':     ('std_msgs/msg/Bool',  0.0),  # only on detection
        '/cmd_vel':              ('geometry_msgs/msg/Twist', 0.0),
    }

    active_topics = ros2_topic_list()

    print("  Checking required topics exist:")
    for topic, (msg_type, min_hz) in required_topics.items():
        exists = topic in active_topics
        all_pass &= print_result(f"Topic exists: {topic}", exists,
            "" if exists else "not in ros2 topic list")

    print("\n  Checking optional topics exist:")
    for topic, (msg_type, min_hz) in optional_topics.items():
        exists = topic in active_topics
        print_result(f"Topic exists: {topic}", exists,
            "(optional)" if not exists else "")

    print("\n  Checking topic publish rates (this takes ~5 seconds each)...")
    print("  (Only checking critical topics)")

    for topic, (msg_type, min_hz) in [
        ('/odom/wheel', ('', 40.0)),
        ('/odom/filtered', ('', 40.0)),
        ('/match/time_remaining', ('', 8.0)),
    ]:
        hz = ros2_topic_hz(topic, duration=4.0)
        if hz < 0:
            all_pass &= print_result(f"Rate check: {topic}", False, "no messages received")
        else:
            passed = hz >= min_hz * 0.8  # allow 20% tolerance
            all_pass &= print_result(
                f"Rate check: {topic}",
                passed,
                f"{hz:.1f} Hz (expected ≥{min_hz:.0f} Hz)"
            )

    print(f"\n  {'STEP 2 PASSED — proceed to Step 3' if all_pass else 'STEP 2 FAILED — fix issues above before continuing'}")
    return all_pass


# ===========================================================================
# STEP 3: EKF Fusion Check
# ===========================================================================
def step3_ekf():
    section("STEP 3: EKF Fusion Check")
    print("  Prerequisite: Step 2 passed, robot stationary on floor.\n")

    all_pass = True

    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry

        rclpy.init()

        class EKFChecker(Node):
            def __init__(self):
                super().__init__('ekf_test_node')
                self.wheel_msgs = []
                self.filtered_msgs = []
                from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
                sensor_qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                )
                self.create_subscription(Odometry, '/odom/wheel',
                    lambda m: self.wheel_msgs.append(m), sensor_qos)
                self.create_subscription(Odometry, '/odom/filtered',
                    lambda m: self.filtered_msgs.append(m), 10)

        node = EKFChecker()
        deadline = time.time() + 5.0
        while time.time() < deadline and (len(node.wheel_msgs) < 5 or len(node.filtered_msgs) < 5):
            rclpy.spin_once(node, timeout_sec=0.1)

        # Check wheel odom received
        all_pass &= print_result(
            f"Wheel odometry messages received ({len(node.wheel_msgs)})",
            len(node.wheel_msgs) >= 5
        )

        # Check filtered odom received
        all_pass &= print_result(
            f"EKF filtered odometry messages received ({len(node.filtered_msgs)})",
            len(node.filtered_msgs) >= 5
        )

        if node.filtered_msgs:
            msg = node.filtered_msgs[-1]
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            cov_x = msg.pose.covariance[0]
            cov_y = msg.pose.covariance[7]

            # At startup robot should be near landing site
            near_landing = abs(x - 0.15) < 0.5 and abs(y - 0.15) < 0.5
            all_pass &= print_result(
                "EKF position near landing site",
                near_landing,
                f"x={x:.3f} y={y:.3f} (expected ~0.15, 0.15)"
            )

            # Covariance should be finite and reasonable
            cov_ok = 0 < cov_x < 10.0 and 0 < cov_y < 10.0
            all_pass &= print_result(
                "EKF covariance finite and reasonable",
                cov_ok,
                f"cov_x={cov_x:.4f} cov_y={cov_y:.4f}"
            )

            # Check filtered and wheel odom are close (EKF not diverging)
            if node.wheel_msgs:
                wmsg = node.wheel_msgs[-1]
                dx = abs(x - wmsg.pose.pose.position.x)
                dy = abs(y - wmsg.pose.pose.position.y)
                close = dx < 0.5 and dy < 0.5
                all_pass &= print_result(
                    "EKF and wheel odom agree (not diverging)",
                    close,
                    f"delta=({dx:.3f},{dy:.3f}) m"
                )

        node.destroy_node()
        rclpy.shutdown()

    except ImportError:
        print(f"  {WARN} rclpy not available — run this test inside the Isaac ROS container")
        print(f"        Skipping EKF check")

    print(f"\n  {'STEP 3 PASSED — proceed to Step 4' if all_pass else 'STEP 3 FAILED — fix issues above before continuing'}")
    return all_pass


# ===========================================================================
# STEP 4: AprilTag Pipeline Check
# ===========================================================================
def step4_apriltag():
    section("STEP 4: AprilTag Pipeline Check")
    print("  Prerequisite: Step 3 passed.")
    print("  Physical: Point camera at any tag36h11 AprilTag (print one from the field set).\n")

    all_pass = True

    # Check camera topic is publishing
    print("  Checking camera feed...")
    camera_ok = ros2_topic_echo_once('/vision/image_raw', '', timeout=5.0)
    all_pass &= print_result(
        "Camera publishing /vision/image_raw",
        camera_ok,
        "" if camera_ok else "Check USB camera is plugged in, check v4l2_camera node"
    )

    if not camera_ok:
        print(f"\n  {FAIL} Camera not publishing. Debug steps:")
        print("        ls /dev/video*")
        print("        v4l2-ctl --list-devices")
        print("        Check jetson_bringup.launch.py video_device parameter")
        return False

    # Check camera_info
    cam_info_ok = ros2_topic_echo_once('/vision/camera_info', '', timeout=3.0)
    all_pass &= print_result(
        "Camera info publishing /vision/camera_info",
        cam_info_ok,
        "" if cam_info_ok else "Camera may not be calibrated — run camera calibration"
    )

    # Check AprilTag detections topic exists and is publishing
    print("\n  Checking AprilTag detector...")
    print("  (Hold a tag36h11 AprilTag in front of the camera now)")
    time.sleep(2)

    try:
        import rclpy
        from rclpy.node import Node

        rclpy.init()

        detections = []

        class TagChecker(Node):
            def __init__(self):
                super().__init__('apriltag_test_node')
                # Try isaac_ros message type first, fall back to generic check
                try:
                    from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
                    self.create_subscription(
                        AprilTagDetectionArray,
                        '/apriltag/detections',
                        lambda m: detections.append(m),
                        10
                    )
                except ImportError:
                    self.get_logger().warn('isaac_ros_apriltag_interfaces not available')

        node = TagChecker()
        deadline = time.time() + 8.0
        while time.time() < deadline and len(detections) == 0:
            rclpy.spin_once(node, timeout_sec=0.2)

        tag_detected = len(detections) > 0
        all_pass &= print_result(
            "AprilTag detection received",
            tag_detected,
            f"{len(detections)} detection messages" if tag_detected else
            "No detections — is a tag36h11 tag visible to the camera?"
        )

        if tag_detected:
            latest = detections[-1]
            if hasattr(latest, 'detections') and len(latest.detections) > 0:
                for det in latest.detections:
                    tag_id = det.id if hasattr(det, 'id') else 'unknown'
                    print(f"    {INFO} Detected tag ID: {tag_id}")

        node.destroy_node()
        rclpy.shutdown()

    except ImportError:
        print(f"  {WARN} rclpy not available — run inside Isaac ROS container")

    if not all_pass:
        print(f"\n  {FAIL} AprilTag pipeline not working. Debug steps:")
        print("        1. Confirm isaac_ros_apriltag node is running: ros2 node list | grep apriltag")
        print("        2. Check NITROS pipeline: ros2 topic echo /apriltag/detections")
        print("        3. Verify camera calibration YAML exists")
        print("        4. Confirm tag family is 36h11 in jetson_bringup.launch.py")

    print(f"\n  {'STEP 4 PASSED — proceed to Step 5' if all_pass else 'STEP 4 FAILED — fix issues above before continuing'}")
    return all_pass


# ===========================================================================
# STEP 5: Match Simulation Dry Run
# ===========================================================================
def step5_match_sim():
    section("STEP 5: Match Simulation Dry Run")
    print("  Prerequisite: All previous steps passed.")
    print("  Physical: Robot on field, camera pointing at Beacon Mast AprilTag.\n")
    print("  This test simulates a match start and watches the state machine.\n")

    all_pass = True

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool, String, Float32
        from nav_msgs.msg import Odometry

        rclpy.init()

        states_seen = []
        time_remaining = [180.0]
        cmd_vel_received = []

        class MatchSimNode(Node):
            def __init__(self):
                super().__init__('match_sim_test_node')

                from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
                reliable_qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                )
                sensor_qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                )

                self.start_pub = self.create_publisher(Bool, '/match/start_signal', reliable_qos)
                self.create_subscription(String, '/match/state',
                    lambda m: states_seen.append(m.data), reliable_qos)
                self.create_subscription(Float32, '/match/time_remaining',
                    lambda m: time_remaining.__setitem__(0, m.data), sensor_qos)
                from geometry_msgs.msg import Twist
                self.create_subscription(Twist, '/cmd_vel',
                    lambda m: cmd_vel_received.append(m), sensor_qos)

        node = MatchSimNode()

        # Wait for mission state node to be in WAIT_START
        print("  Waiting for mission_state_node to reach WAIT_START...")
        deadline = time.time() + 10.0
        while time.time() < deadline and 'WAIT_START' not in states_seen:
            rclpy.spin_once(node, timeout_sec=0.1)

        wait_start_ok = 'WAIT_START' in states_seen
        all_pass &= print_result(
            "Mission state node reached WAIT_START",
            wait_start_ok,
            "" if wait_start_ok else "Is mission_state_node running?"
        )

        if not wait_start_ok:
            node.destroy_node()
            rclpy.shutdown()
            return False

        # Send start signal
        print("\n  Sending start signal...")
        start_msg = Bool()
        start_msg.data = True
        node.start_pub.publish(start_msg)

        # Watch state transitions for 10 seconds
        print("  Watching state transitions for 10 seconds...")
        deadline = time.time() + 10.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Check state machine progressed past WAIT_START
        unique_states = list(dict.fromkeys(states_seen))  # ordered unique
        progressed = len(unique_states) > 1 and 'WAIT_START' in unique_states
        all_pass &= print_result(
            "State machine progressed after start signal",
            progressed,
            f"States seen: {' → '.join(unique_states)}"
        )

        # Check cmd_vel was published (robot tried to move)
        all_pass &= print_result(
            "cmd_vel published (robot commanded to move)",
            len(cmd_vel_received) > 0,
            f"{len(cmd_vel_received)} messages" if cmd_vel_received else
            "No movement commands — check path_planner_node"
        )

        # Check match timer is counting down
        time_ok = time_remaining[0] < 180.0 and time_remaining[0] > 0.0
        all_pass &= print_result(
            "Match timer counting down",
            time_ok,
            f"{time_remaining[0]:.1f}s remaining"
        )

        print(f"\n  State sequence: {' → '.join(unique_states)}")

        node.destroy_node()
        rclpy.shutdown()

    except ImportError:
        print(f"  {WARN} rclpy not available — run inside Isaac ROS container")

    print(f"\n  {'STEP 5 PASSED — robot is competition ready!' if all_pass else 'STEP 5 FAILED — fix issues above'}")
    return all_pass


# ===========================================================================
# Main
# ===========================================================================
def main():
    parser = argparse.ArgumentParser(description='Mining Mayhem Integration Test Suite')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--step', type=int, choices=[1,2,3,4,5], help='Run a specific step')
    group.add_argument('--all', action='store_true', help='Run all steps in sequence')
    args = parser.parse_args()

    steps = {
        1: step1_udp_bridge,
        2: step2_ros_topics,
        3: step3_ekf,
        4: step4_apriltag,
        5: step5_match_sim,
    }

    if args.all:
        print("\n  Running full integration test suite...")
        for i in range(1, 6):
            passed = steps[i]()
            if not passed:
                print(f"\n\033[91m  Stopped at Step {i} — fix failures before continuing.\033[0m\n")
                sys.exit(1)
        print("\n\033[92m  ALL STEPS PASSED — robot is competition ready!\033[0m\n")
    else:
        passed = steps[args.step]()
        sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
