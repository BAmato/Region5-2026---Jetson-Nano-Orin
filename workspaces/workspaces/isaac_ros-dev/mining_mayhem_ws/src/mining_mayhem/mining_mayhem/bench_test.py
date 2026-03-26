#!/usr/bin/env python3
# ===========================================================================
# bench_test.py
# Mining Mayhem — interactive bench test + manual autonomous run launcher.
# Runs INSIDE the Isaac ROS container.
#
# Usage (from inside container):
#   python3 bench_test.py            # full check + interactive prompt
#   python3 bench_test.py --checks   # checks only, no auto run prompt
#   python3 bench_test.py --autorun  # skip checks, go straight to start-light wait
#
# What it does:
#   1. Runs all system checks (UDP bridge, ROS topics, EKF, AprilTag, start LED)
#   2. Displays a live status dashboard with PASS/FAIL/WARN for each check
#   3. Prompts: "All checks passed. Start autonomous run? [y/N]"
#   4. If yes: arms the robot (publishes software_enable=True) and waits for
#      the real start light OR a manual keyboard trigger
#   5. Once triggered: publishes /match/start_signal=True and monitors states
#
# For competition: use competition_start.sh (fully headless, no prompt).
# For bench testing: use this script (interactive, shows diagnostics).
# ===========================================================================

import argparse
import math
import os
import socket
import struct
import subprocess
import sys
import time
import threading

os.environ.setdefault('ROS_DOMAIN_ID', '42')

# ---------------------------------------------------------------------------
# Terminal colours
# ---------------------------------------------------------------------------
RESET  = '\033[0m'
GREEN  = '\033[92m'
RED    = '\033[91m'
YELLOW = '\033[93m'
BLUE   = '\033[94m'
CYAN   = '\033[96m'
BOLD   = '\033[1m'
CLEAR  = '\033[2J\033[H'

PASS_TAG = f'{GREEN}[ PASS ]{RESET}'
FAIL_TAG = f'{RED}[ FAIL ]{RESET}'
WARN_TAG = f'{YELLOW}[ WARN ]{RESET}'
INFO_TAG = f'{BLUE}[ INFO ]{RESET}'

# ---------------------------------------------------------------------------
# Network constants
# ---------------------------------------------------------------------------
ROBORIO_IP  = '10.0.67.2'
RX_PORT     = 5800
TX_PORT     = 5801
PACKET_SIZE = 60
MAGIC_RX    = b'\xA5\x5A'
MAGIC_TX    = b'\x5A\xA5'
RIO_FMT     = '<2sB6f3f3iBI BH B'
JETSON_FMT  = '<2sB6fBBB'


def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def pr(tag, label, detail=''):
    detail_str = f'  {detail}' if detail else ''
    print(f'  {tag}  {label}{detail_str}')


def section(title):
    print(f'\n{BOLD}{"=" * 62}{RESET}')
    print(f'  {BOLD}{title}{RESET}')
    print(f'{BOLD}{"=" * 62}{RESET}')


def ros2_run(cmd, timeout=6):
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '42'
    try:
        r = subprocess.run(cmd, capture_output=True, text=True,
                           timeout=timeout, env=env)
        return r.stdout, r.returncode
    except subprocess.TimeoutExpired:
        return '', -1
    except Exception:
        return '', -2


def topic_list():
    out, _ = ros2_run(['ros2', 'topic', 'list'], timeout=5)
    return out.splitlines()


def topic_hz(topic, duration=3.5):
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '42'
    try:
        r = subprocess.run(
            ['ros2', 'topic', 'hz', '--window', '10', topic],
            capture_output=True, text=True, timeout=duration + 2, env=env)
        for line in r.stdout.splitlines():
            if 'average rate:' in line:
                return float(line.split('average rate:')[1].strip().split()[0])
    except Exception:
        pass
    return -1.0


def topic_echo_once(topic, timeout=5.0):
    out, _ = ros2_run(['ros2', 'topic', 'echo', '--once', topic], timeout=timeout)
    return len(out.strip()) > 0


# ===========================================================================
# CHECK 1 — UDP bridge / roboRIO comms
# ===========================================================================

def check_udp_bridge():
    section('CHECK 1: roboRIO UDP bridge')
    ok = True
    last_data = None

    # Ping
    ping_out, ping_rc = ros2_run(['ping', '-c', '2', '-W', '1', ROBORIO_IP], timeout=5)
    ping_ok = ping_rc == 0
    pr(PASS_TAG if ping_ok else FAIL_TAG, f'roboRIO reachable at {ROBORIO_IP}',
       '' if ping_ok else 'check ethernet cable and roboRIO IP')
    ok &= ping_ok

    # Bind RX socket and listen for packets
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', RX_PORT))
        sock.settimeout(4.0)

        received = 0
        valid = 0
        t0 = time.time()
        while time.time() - t0 < 4.0:
            try:
                data, _ = sock.recvfrom(4096)
                received += 1
                if len(data) == PACKET_SIZE and data[:2] == MAGIC_RX:
                    if data[-1] == crc8(data[:-1]):
                        valid += 1
                        last_data = data
            except socket.timeout:
                break
        sock.close()

        pr(PASS_TAG if received > 0 else FAIL_TAG,
           f'roboRIO packets received ({received} in 4s)',
           'pkts/s ≈ ' + str(round(received / 4)) if received > 0 else
           'is robot code deployed and running?')
        pr(PASS_TAG if valid > 0 else FAIL_TAG,
           f'Valid packets (magic+CRC) {valid}/{received}')
        ok &= valid > 0

    except OSError as e:
        pr(FAIL_TAG, 'Cannot bind UDP RX socket', str(e))
        pr(WARN_TAG, 'Is serial_bridge_node running? If so skip this check.')
        ok = False

    # Decode last packet if we have one
    if last_data:
        try:
            fields = struct.unpack(RIO_FMT, last_data)
            ox, oy, oth = fields[2], fields[3], fields[4]
            state = fields[16]
            pr(INFO_TAG, f'Latest packet: odom=({ox:.3f},{oy:.3f}) '
                         f'theta={math.degrees(oth):.1f}° rio_state={state}')
        except Exception:
            pass

    return ok


# ===========================================================================
# CHECK 2 — ROS topic health
# ===========================================================================

def check_ros_topics():
    section('CHECK 2: ROS topic health')
    ok = True

    required = {
        '/odom/wheel':           40.0,
        '/imu/data':             40.0,
        '/odom/filtered':        40.0,
        '/match/state':           1.0,
        '/match/time_remaining':  8.0,
        '/led_cam/image_raw':     5.0,
        '/image':                 5.0,
    }
    optional = ['/apriltag/detections', '/robot/hall_event',
                '/cmd_vel', '/match/start_signal']

    active = topic_list()
    if not active:
        pr(FAIL_TAG, 'No ROS topics found — is jetson_bringup running?')
        print(f'\n  Start with:  ros2 launch mining_mayhem jetson_bringup.launch.py '
              f'enable_led_camera:=true enable_path_planner:=false')
        return False

    print('  Required topics:')
    for t, min_hz in required.items():
        exists = t in active
        pr(PASS_TAG if exists else FAIL_TAG, f'{t}',
           '' if exists else 'NOT in topic list')
        ok &= exists

    print('\n  Optional topics:')
    for t in optional:
        exists = t in active
        pr(PASS_TAG if exists else WARN_TAG, f'{t}',
           '' if exists else '(optional / not yet receiving)')

    # Rate checks on the three most critical topics
    print('\n  Rate checks (~4s each):')
    for t, min_hz in [('/odom/wheel', 40.0), ('/odom/filtered', 40.0),
                      ('/led_cam/image_raw', 5.0)]:
        if t not in active:
            pr(WARN_TAG, f'Rate {t}', 'skipped (topic absent)')
            continue
        hz = topic_hz(t, duration=3.5)
        passed = hz >= min_hz * 0.75
        pr(PASS_TAG if passed else FAIL_TAG,
           f'Rate {t}',
           f'{hz:.1f} Hz (want ≥ {min_hz:.0f})'
           if hz >= 0 else 'no messages — is source running?')
        if t != '/led_cam/image_raw':  # camera rate is soft
            ok &= passed

    return ok


# ===========================================================================
# CHECK 3 — EKF fusion
# ===========================================================================

def check_ekf():
    section('CHECK 3: EKF fusion')
    ok = True

    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        if not rclpy.ok():
            rclpy.init()

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        wheel_msgs, filt_msgs = [], []

        class EKFNode(Node):
            def __init__(self):
                super().__init__('bench_ekf_checker')
                self.create_subscription(Odometry, '/odom/wheel',
                                         lambda m: wheel_msgs.append(m), sensor_qos)
                self.create_subscription(Odometry, '/odom/filtered',
                                         lambda m: filt_msgs.append(m), 10)

        node = EKFNode()
        deadline = time.time() + 5.0
        while time.time() < deadline and (len(wheel_msgs) < 5 or len(filt_msgs) < 5):
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()

        pr(PASS_TAG if len(wheel_msgs) >= 5 else FAIL_TAG,
           f'Wheel odometry flowing ({len(wheel_msgs)} msgs in 5s)')
        pr(PASS_TAG if len(filt_msgs) >= 5 else FAIL_TAG,
           f'EKF filtered output flowing ({len(filt_msgs)} msgs in 5s)')
        ok &= len(wheel_msgs) >= 5 and len(filt_msgs) >= 5

        if filt_msgs:
            m = filt_msgs[-1]
            x, y = m.pose.pose.position.x, m.pose.pose.position.y
            cx, cy = m.pose.covariance[0], m.pose.covariance[7]
            near = abs(x - 0.15) < 0.5 and abs(y - 0.15) < 0.5
            cov_ok = 0 < cx < 10.0 and 0 < cy < 10.0
            pr(PASS_TAG if near else WARN_TAG,
               f'EKF position near landing site',
               f'x={x:.3f} y={y:.3f} (expect ≈ 0.15, 0.15)')
            pr(PASS_TAG if cov_ok else WARN_TAG,
               f'EKF covariance finite',
               f'cov_x={cx:.4f} cov_y={cy:.4f}')

    except ImportError:
        pr(WARN_TAG, 'rclpy not available — run inside Isaac ROS container')
        ok = False

    return ok


# ===========================================================================
# CHECK 4 — AprilTag pipeline
# ===========================================================================

def check_apriltag():
    section('CHECK 4: AprilTag pipeline')
    ok = True

    # Camera must be publishing
    cam_ok = topic_echo_once('/image', timeout=4.0)
    pr(PASS_TAG if cam_ok else FAIL_TAG, 'Front camera publishing /image',
       '' if cam_ok else 'check v4l2_camera node and /dev/video0')
    ok &= cam_ok

    if not cam_ok:
        return False

    # AprilTag container
    nodes_out, _ = ros2_run(['ros2', 'node', 'list'], timeout=5)
    tag_node_ok = 'apriltag' in nodes_out
    pr(PASS_TAG if tag_node_ok else FAIL_TAG, 'isaac_ros_apriltag node running',
       '' if tag_node_ok else 'check apriltag_container in launch file')

    print(f'\n  {INFO_TAG}  Hold any tag36h11 AprilTag in front of the '
          f'front camera now...')
    print('  Waiting 8 seconds for a detection...')

    try:
        import rclpy
        from rclpy.node import Node
        if not rclpy.ok():
            rclpy.init()

        detections = []

        class TagNode(Node):
            def __init__(self):
                super().__init__('bench_tag_checker')
                try:
                    from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
                    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
                    self.create_subscription(
                        AprilTagDetectionArray, '/apriltag/detections',
                        lambda m: detections.append(m),
                        QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                   history=HistoryPolicy.KEEP_LAST, depth=1))
                except ImportError:
                    pass

        node = TagNode()
        deadline = time.time() + 8.0
        while time.time() < deadline and not detections:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()

        tag_ok = len(detections) > 0
        pr(PASS_TAG if tag_ok else WARN_TAG,
           'AprilTag detection received',
           f'{len(detections)} msgs' if tag_ok else
           'no tag seen — is a tag36h11 tag visible?')
        if tag_ok:
            latest = detections[-1]
            if hasattr(latest, 'detections'):
                for d in latest.detections:
                    pr(INFO_TAG, f'  tag ID detected: {d.id}')

    except ImportError:
        pr(WARN_TAG, 'rclpy not available — skipping detection check')

    return ok


# ===========================================================================
# CHECK 5 — Start LED camera
# ===========================================================================

def check_start_led():
    section('CHECK 5: Start LED detector')
    ok = True

    led_cam_ok = topic_echo_once('/led_cam/image_raw', timeout=4.0)
    pr(PASS_TAG if led_cam_ok else FAIL_TAG,
       'LED camera publishing /led_cam/image_raw',
       '' if led_cam_ok else
       'check enable_led_camera:=true in launch and /dev/video1')
    ok &= led_cam_ok

    detector_running = False
    nodes_out, _ = ros2_run(['ros2', 'node', 'list'], timeout=5)
    detector_running = 'start_led_detector' in nodes_out
    pr(PASS_TAG if detector_running else FAIL_TAG,
       'start_led_detector_node running',
       '' if detector_running else 'check it is in jetson_bringup.launch.py')
    ok &= detector_running

    if led_cam_ok and detector_running:
        print(f'\n  {INFO_TAG}  LED camera and detector are both running.')
        print(f'  {INFO_TAG}  ROI calibration reminder:')
        print(f'           ros2 run rqt_image_view rqt_image_view → /led_cam/image_raw')
        print(f'           Note pixel coords of Start LED → update apriltag_config.yaml')

    return ok


# ===========================================================================
# CHECK 6 — Mission state machine
# ===========================================================================

def check_mission_state():
    section('CHECK 6: Mission state machine')
    ok = True

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        if not rclpy.ok():
            rclpy.init()

        states = []
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   history=HistoryPolicy.KEEP_LAST, depth=10)

        class StateNode(Node):
            def __init__(self):
                super().__init__('bench_state_checker')
                self.create_subscription(String, '/match/state',
                                         lambda m: states.append(m.data),
                                         reliable_qos)

        node = StateNode()
        deadline = time.time() + 5.0
        while time.time() < deadline and not states:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()

        got_state = len(states) > 0
        pr(PASS_TAG if got_state else FAIL_TAG,
           '/match/state publishing',
           f'current: {states[-1]}' if states else
           'mission_state_node not running?')

        if states:
            in_wait = states[-1] == 'WAIT_START'
            pr(PASS_TAG if in_wait else WARN_TAG,
               'Mission state is WAIT_START',
               states[-1] if not in_wait else '')

        ok &= got_state

    except ImportError:
        pr(WARN_TAG, 'rclpy not available')
        ok = False

    return ok


# ===========================================================================
# AUTO-RUN: arm robot, wait for start light (or key press), then run
# ===========================================================================

def wait_for_start_and_run():
    section('AUTONOMOUS RUN')

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool, String, Float32
        from geometry_msgs.msg import Twist
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        if not rclpy.ok():
            rclpy.init()

        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   history=HistoryPolicy.KEEP_LAST, depth=10)
        sensor_qos  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST, depth=1)

        states_seen     = []
        cmd_vel_count   = [0]
        time_remaining  = [180.0]
        start_received  = [False]

        class RunNode(Node):
            def __init__(self):
                super().__init__('bench_run_node')
                self.pub_enable = self.create_publisher(
                    Bool, '/bridge/software_enable', reliable_qos)
                self.pub_start  = self.create_publisher(
                    Bool, '/match/start_signal', reliable_qos)
                self.create_subscription(String, '/match/state',
                    lambda m: states_seen.append(m.data), reliable_qos)
                self.create_subscription(Twist, '/cmd_vel',
                    lambda m: cmd_vel_count.__setitem__(0, cmd_vel_count[0] + 1),
                    sensor_qos)
                self.create_subscription(Float32, '/match/time_remaining',
                    lambda m: time_remaining.__setitem__(0, m.data), sensor_qos)
                self.create_subscription(Bool, '/match/start_signal',
                    lambda m: start_received.__setitem__(0, m.data or start_received[0]),
                    reliable_qos)

        node = RunNode()

        # Arm the robot
        arm_msg = Bool()
        arm_msg.data = True
        node.pub_enable.publish(arm_msg)
        print(f'  {GREEN}Robot ARMED — software_enable=True published{RESET}')
        print()

        # Wait for WAIT_START
        print('  Waiting for mission_state_node → WAIT_START...')
        deadline = time.time() + 10.0
        while time.time() < deadline and 'WAIT_START' not in states_seen:
            rclpy.spin_once(node, timeout_sec=0.1)

        if 'WAIT_START' not in states_seen:
            print(f'  {WARN_TAG}  Mission state not yet WAIT_START — proceeding anyway')
        else:
            print(f'  {PASS_TAG}  Mission state: WAIT_START — ready')

        print()
        print(f'  {BOLD}Waiting for Start LED or manual trigger...{RESET}')
        print(f'  Real start light: place robot in Landing Site, LED will auto-trigger')
        print(f'  Manual trigger:   press {BOLD}ENTER{RESET} to simulate start signal now')
        print()

        # Start a thread to watch for ENTER key
        enter_pressed = [False]

        def watch_enter():
            try:
                input()
                enter_pressed[0] = True
            except Exception:
                pass

        t = threading.Thread(target=watch_enter, daemon=True)
        t.start()

        # Spin until start_received (from LED detector) or ENTER key
        print('  Monitoring... (Ctrl+C to abort)')
        last_status = time.time()
        while not start_received[0] and not enter_pressed[0]:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - last_status > 2.0:
                current_state = states_seen[-1] if states_seen else '?'
                print(f'\r  State: {current_state} | '
                      f'T-{time_remaining[0]:.0f}s | '
                      f'cmd_vel msgs: {cmd_vel_count[0]}    ',
                      end='', flush=True)
                last_status = time.time()

        print()

        if enter_pressed[0] and not start_received[0]:
            # Manual trigger — publish start signal ourselves
            print(f'\n  {INFO_TAG}  Manual trigger — publishing /match/start_signal=True')
            start_msg = Bool()
            start_msg.data = True
            node.pub_start.publish(start_msg)
            # Also re-publish software_enable in case it was missed
            node.pub_enable.publish(arm_msg)
        else:
            print(f'\n  {GREEN}Start LED detected — match running!{RESET}')

        # Monitor the run
        print()
        print('  Match running. Press Ctrl+C to abort.')
        print()

        run_start = time.time()
        last_print = time.time()

        try:
            while True:
                rclpy.spin_once(node, timeout_sec=0.1)

                if time.time() - last_print > 1.0:
                    current_state = states_seen[-1] if states_seen else '?'
                    elapsed = time.time() - run_start
                    tr = time_remaining[0]
                    cmds = cmd_vel_count[0]
                    status_color = GREEN if cmds > 0 else YELLOW
                    print(f'\r  {status_color}T+{elapsed:.0f}s{RESET} | '
                          f'State: {BOLD}{current_state:<20}{RESET} | '
                          f'T-{tr:.0f}s | '
                          f'cmd_vel: {cmds}    ',
                          end='', flush=True)
                    last_print = time.time()

                    if current_state == 'END_OF_MATCH':
                        print(f'\n\n  {GREEN}Match complete — END_OF_MATCH reached{RESET}')
                        break

                    if tr <= 0:
                        print(f'\n\n  {YELLOW}Match timer expired{RESET}')
                        break

        except KeyboardInterrupt:
            print(f'\n\n  {YELLOW}Aborted by user — disarming robot{RESET}')
            disarm = Bool()
            disarm.data = False
            node.pub_enable.publish(disarm)
            stop = Twist()
            from rclpy.node import Node as _N
            stop_pub = node.create_publisher(Twist, '/cmd_vel', sensor_qos)
            stop_pub.publish(stop)

        node.destroy_node()

    except ImportError:
        print(f'  {FAIL_TAG}  rclpy not available — run inside Isaac ROS container')


# ===========================================================================
# Main
# ===========================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Mining Mayhem bench test + autonomous run launcher')
    parser.add_argument('--checks',  action='store_true',
                        help='Run system checks only, no auto-run prompt')
    parser.add_argument('--autorun', action='store_true',
                        help='Skip checks, go straight to auto-run (start light wait)')
    args = parser.parse_args()

    if args.autorun:
        wait_for_start_and_run()
        return

    print(CLEAR)
    print(f'{BOLD}{CYAN}')
    print('  ╔══════════════════════════════════════════════════════╗')
    print('  ║      Mining Mayhem — Bench Test & Run Launcher      ║')
    print('  ╚══════════════════════════════════════════════════════╝')
    print(RESET)
    print(f'  {INFO_TAG}  ROS_DOMAIN_ID = {os.environ.get("ROS_DOMAIN_ID", "42")}')
    print(f'  {INFO_TAG}  roboRIO target: {ROBORIO_IP}')
    print()

    results = {}

    results['UDP bridge']       = check_udp_bridge()
    results['ROS topics']       = check_ros_topics()
    results['EKF fusion']       = check_ekf()
    results['AprilTag pipeline']= check_apriltag()
    results['Start LED camera'] = check_start_led()
    results['Mission state']    = check_mission_state()

    # Summary
    section('SYSTEM CHECK SUMMARY')
    all_pass = True
    for name, passed in results.items():
        tag = PASS_TAG if passed else FAIL_TAG
        pr(tag, name)
        all_pass &= passed

    print()
    if all_pass:
        print(f'  {GREEN}{BOLD}All checks passed.{RESET}')
    else:
        failed = [n for n, p in results.items() if not p]
        print(f'  {YELLOW}{BOLD}{len(failed)} check(s) failed: {", ".join(failed)}{RESET}')
        print(f'  {WARN_TAG}  You can still proceed but behaviour may be incorrect.')

    if args.checks:
        sys.exit(0 if all_pass else 1)

    # Prompt
    print()
    print(f'  {BOLD}{"─" * 60}{RESET}')
    try:
        answer = input(
            f'  Start autonomous run? '
            f'[{GREEN}y{RESET}=yes / {RED}n{RESET}=no / {YELLOW}f{RESET}=force even if checks failed]: '
        ).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print(f'\n  {YELLOW}Cancelled.{RESET}')
        sys.exit(0)

    if answer == 'y' and not all_pass:
        print(f'  {WARN_TAG}  Checks failed — use "f" to force, or fix issues first.')
        sys.exit(1)

    if answer in ('y', 'f'):
        wait_for_start_and_run()
    else:
        print(f'  {INFO_TAG}  Aborted. Re-run when ready.')
        sys.exit(0)


if __name__ == '__main__':
    main()
