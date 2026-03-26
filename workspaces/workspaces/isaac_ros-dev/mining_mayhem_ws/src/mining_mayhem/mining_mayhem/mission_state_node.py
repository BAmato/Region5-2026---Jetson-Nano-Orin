#!/usr/bin/env python3
# ===========================================================================
# mission_state_node.py
# Top-level match state machine for the Mining Mayhem competition robot.
#
# Implements the hierarchical state machine from the RAS Architecture v1.2
# document (Section 7).  Runs on the Jetson Orin Nano.
#
# States:
#   INIT → WAIT_START → DEPART_LANDING → READ_TELEMETRY → DEPOSIT_BEACON →
#   COLLECT_MATERIALS → LOAD_CSCS → DELIVER_CSCS → CAVE_ENTRY → END_OF_MATCH
#   (FAULT can be entered from any state)
#
# Coordinate frame: field origin at south-west corner, +X = east, +Y = north
# ===========================================================================

import math
import time
from enum import IntEnum
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int8, String


# ---------------------------------------------------------------------------
# State enumeration (values must match roboRIO match_state byte encoding)
# ---------------------------------------------------------------------------
class MatchState(IntEnum):
    INIT = 0
    WAIT_START = 1
    DEPART_LANDING = 2
    READ_TELEMETRY = 3
    DEPOSIT_BEACON = 4
    COLLECT_MATERIALS = 5
    LOAD_CSCS = 6
    DELIVER_CSCS = 7
    CAVE_ENTRY = 8
    END_OF_MATCH = 9
    FAULT = 10


# ---------------------------------------------------------------------------
# Beacon deposit sub-states
# ---------------------------------------------------------------------------
class BeaconSubState(IntEnum):
    APPROACH_MAST = 0
    ALIGN_TO_MAST = 1
    EXTEND_ARM = 2
    DEPOSIT = 3
    RETRACT_ARM = 4
    DONE = 5


# ---------------------------------------------------------------------------
# Helper: Euclidean distance
# ---------------------------------------------------------------------------
def dist(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ===========================================================================
# Mission State Node
# ===========================================================================
class MissionStateNode(Node):

    def __init__(self):
        super().__init__('mission_state_node')

        # -------------------------------------------------------------------
        # Parameters
        # -------------------------------------------------------------------
        self.declare_parameter('landing_x', 0.15)
        self.declare_parameter('landing_y', 0.15)
        self.declare_parameter('beacon_mast_x', 0.05)
        self.declare_parameter('beacon_mast_y', 0.61)
        self.declare_parameter('field_length_m', 2.4384)  # 96 inches
        self.declare_parameter('field_width_m', 1.2192)   # 48 inches
        self.declare_parameter('intake_width_m', 0.20)

        self._landing_x = self.get_parameter('landing_x').value
        self._landing_y = self.get_parameter('landing_y').value
        self._beacon_mast_x = self.get_parameter('beacon_mast_x').value
        self._beacon_mast_y = self.get_parameter('beacon_mast_y').value
        self._field_length = self.get_parameter('field_length_m').value
        self._field_width = self.get_parameter('field_width_m').value
        self._intake_width = self.get_parameter('intake_width_m').value

        # -------------------------------------------------------------------
        # Match timing
        # -------------------------------------------------------------------
        self._match_duration_s = 180.0
        self._match_start_time: Optional[float] = None
        self._time_remaining_s = self._match_duration_s

        # Time budgets for each phase (seconds) – Strategy A conservative
        # CALIBRATE: Adjust based on testing.  These define when to force
        # transition to the next phase to stay within the match clock.
        self._budget_depart = 3.0          # 3s for landing bonus
        self._budget_read_telemetry = 8.0  # 8s to navigate + read tag
        self._budget_deposit_beacon = 15.0 # 15s for full beacon sequence
        self._budget_collect = 60.0        # 60s for sweep
        self._budget_load = 15.0           # 15s to load CSCs
        self._budget_deliver = 40.0        # 40s to deliver both CSCs
        self._budget_cave = 20.0           # 20s for cave entry
        self._min_time_for_cave = 25.0     # don't attempt cave with < 25s

        # -------------------------------------------------------------------
        # State machine
        # -------------------------------------------------------------------
        self._state = MatchState.INIT
        self._beacon_sub = BeaconSubState.APPROACH_MAST
        self._phase_start_time: Optional[float] = None

        # Navigation waypoints
        self._current_waypoint_idx = 0
        self._waypoints: List[Tuple[float, float, float]] = []  # (x, y, heading)

        # Boustrophedon sweep pattern (generated in COLLECT_MATERIALS)
        self._sweep_waypoints: List[Tuple[float, float, float]] = []

        # Target pad from telemetry tag
        self._target_pad = -1  # -1 = not yet decoded

        # Robot pose (from EKF filtered odometry)
        self._robot_x = self._landing_x
        self._robot_y = self._landing_y
        self._robot_theta = 0.0

        # Hall event counter
        self._geodinium_count = 0
        self._nebulite_count = 0
        self._total_collected = 0

        # Beacon deposit sub-state timer
        self._sub_state_timer: Optional[float] = None

        # -------------------------------------------------------------------
        # Known field positions (in field frame, origin = SW corner)
        # All measurements in meters.
        # CALIBRATE: Verify all positions on the mock field.
        # -------------------------------------------------------------------
        # Rendezvous Pads (west side, 5 pads numbered 0-4 from south to north)
        # Pad spacing: approximately 8.5" apart, starting from south
        # CALIBRATE: measure exact pad center positions
        pad_y_start = 0.127   # ~5 inches from south wall (CALIBRATE)
        pad_spacing = 0.216   # ~8.5 inches between pad centers (CALIBRATE)
        self._pad_positions = []
        for i in range(5):
            # Pads are on the west side, x ≈ 6.5" = 0.165m from west wall
            self._pad_positions.append((
                0.165,  # CALIBRATE: x position of pad centers
                pad_y_start + i * pad_spacing,
            ))

        # CSC starting positions
        # CALIBRATE: measure on mock field
        self._geodinium_csc_pos = (1.50, 0.30)   # (CALIBRATE)
        self._nebulite_csc_pos = (1.50, 0.90)     # (CALIBRATE)

        # Depart waypoint: just north of landing site
        self._depart_waypoint = (0.35, 0.15, 0.0)  # (CALIBRATE)

        # Telemetry reading position: facing beacon mast
        self._telemetry_read_pos = (
            self._beacon_mast_x + 0.30,  # 30cm east of mast
            self._beacon_mast_y,
            math.pi,  # facing west toward mast
        )

        # Beacon approach standoff position
        self._beacon_approach_pos = (
            self._beacon_mast_x + 0.30,  # 12" = 30cm standoff
            self._beacon_mast_y,
            math.pi,  # facing west
        )

        # Cave entrance position
        # CALIBRATE: measure cave entrance coordinates on mock field
        self._cave_entrance_pos = (1.0, 0.80, math.pi / 2)  # (CALIBRATE)

        # Beacon arm positions
        # CALIBRATE: servo values for beacon deposit sequence
        self._beacon_arm_home = 0.1     # retracted
        self._beacon_arm_extended = 0.8  # deployed into mast hole

        # Container arm positions
        # CALIBRATE: servo values for CSC latch/release
        self._container_arm_release = 0.2
        self._container_arm_latch = 0.8

        # Sort gate positions (not directly controlled by state machine,
        # but needed for initial position)
        self._sort_gate_default = 0.5

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -------------------------------------------------------------------
        # Subscribers
        # -------------------------------------------------------------------
        self.sub_start = self.create_subscription(
            Bool, '/match/start_signal', self._cb_start_signal, reliable_qos)
        self.sub_target_pad = self.create_subscription(
            Int8, '/match/target_pad', self._cb_target_pad, reliable_qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom/filtered', self._cb_odom, sensor_qos)
        self.sub_hall = self.create_subscription(
            Bool, '/robot/hall_event', self._cb_hall_event, sensor_qos)

        # -------------------------------------------------------------------
        # Publishers
        # -------------------------------------------------------------------
        self.pub_state = self.create_publisher(
            String, '/match/state', reliable_qos)
        self.pub_cmd_vel = self.create_publisher(
            Twist, '/cmd_vel', sensor_qos)
        self.pub_beacon_arm = self.create_publisher(
            Float32, '/actuator/beacon_arm', sensor_qos)
        self.pub_container_arm = self.create_publisher(
            Float32, '/actuator/container_arm', sensor_qos)
        self.pub_sort_gate = self.create_publisher(
            Float32, '/actuator/sort_gate', sensor_qos)
        self.pub_time_remaining = self.create_publisher(
            Float32, '/match/time_remaining', sensor_qos)

        # -------------------------------------------------------------------
        # Timers
        # -------------------------------------------------------------------
        # Main state machine runs at 50 Hz (matches control loop)
        self._timer = self.create_timer(0.02, self._tick)

        # Time remaining publisher at 10 Hz
        self._time_pub_timer = self.create_timer(0.1, self._publish_time)

        self.get_logger().info('Mission state node initialized')

    # =======================================================================
    # Subscriber callbacks
    # =======================================================================

    def _cb_start_signal(self, msg: Bool):
        if msg.data and self._state == MatchState.WAIT_START:
            self._match_start_time = self.get_clock().now().nanoseconds / 1e9
            self._transition_to(MatchState.DEPART_LANDING)
            self.get_logger().info('START SIGNAL RECEIVED – match clock running')

    def _cb_target_pad(self, msg: Int8):
        if msg.data >= 0 and self._target_pad < 0:
            self._target_pad = msg.data
            self.get_logger().info(f'Telemetry decoded: target pad = {self._target_pad}')

    def _cb_odom(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_theta = math.atan2(siny_cosp, cosy_cosp)

    def _cb_hall_event(self, msg: Bool):
        if msg.data:
            self._geodinium_count += 1
        else:
            self._nebulite_count += 1
        self._total_collected += 1

    # =======================================================================
    # Time management
    # =======================================================================

    def _get_time_remaining(self) -> float:
        if self._match_start_time is None:
            return self._match_duration_s
        elapsed = (self.get_clock().now().nanoseconds / 1e9) - self._match_start_time
        return max(0.0, self._match_duration_s - elapsed)

    def _get_phase_elapsed(self) -> float:
        if self._phase_start_time is None:
            return 0.0
        return (self.get_clock().now().nanoseconds / 1e9) - self._phase_start_time

    def _publish_time(self):
        self._time_remaining_s = self._get_time_remaining()
        msg = Float32()
        msg.data = self._time_remaining_s
        self.pub_time_remaining.publish(msg)

    # =======================================================================
    # State transition
    # =======================================================================

    def _transition_to(self, new_state: MatchState):
        old_name = MatchState(self._state).name
        new_name = MatchState(new_state).name
        self.get_logger().info(
            f'STATE: {old_name} → {new_name}  '
            f'(T-{self._get_time_remaining():.1f}s)')
        self._state = new_state
        self._phase_start_time = self.get_clock().now().nanoseconds / 1e9
        self._current_waypoint_idx = 0

    def _publish_state(self):
        msg = String()
        msg.data = MatchState(self._state).name
        self.pub_state.publish(msg)

    # =======================================================================
    # Motion helpers
    # =======================================================================

    def _navigate_to(self, target_x: float, target_y: float,
                     target_heading: Optional[float] = None) -> bool:
        """
        Simple proportional controller to drive toward a target pose.
        Returns True when the robot is within tolerance of the target.
        
        For the actual competition robot, replace this with the path_planner_node
        subscription.  This inline controller is for standalone testing.
        """
        dx = target_x - self._robot_x
        dy = target_y - self._robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        # Waypoint arrival tolerance
        POSITION_TOL = 0.03  # 3 cm
        HEADING_TOL = 0.08   # ~4.5 degrees

        if distance < POSITION_TOL:
            if target_heading is not None:
                heading_err = normalize_angle(target_heading - self._robot_theta)
                if abs(heading_err) < HEADING_TOL:
                    self._send_cmd_vel(0, 0, 0)
                    return True
                else:
                    # Rotate in place
                    omega = 1.5 * heading_err  # P-controller
                    omega = max(-1.0, min(1.0, omega))
                    self._send_cmd_vel(0, 0, omega)
                    return False
            else:
                self._send_cmd_vel(0, 0, 0)
                return True

        # Compute desired heading to target
        desired_heading = math.atan2(dy, dx)
        heading_err = normalize_angle(desired_heading - self._robot_theta)

        # Speed proportional to distance, capped
        speed = min(0.3, 0.8 * distance)

        # Decompose into vx, vy in robot frame
        vx = speed * math.cos(heading_err)
        vy = speed * math.sin(heading_err)

        # Rotate toward target heading
        omega = 0.0
        if target_heading is not None and distance < 0.15:
            # Start aligning heading when close
            final_err = normalize_angle(target_heading - self._robot_theta)
            omega = 1.0 * final_err

        self._send_cmd_vel(vx, vy, omega)
        return False

    def _send_cmd_vel(self, vx: float, vy: float, omega: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(omega)
        self.pub_cmd_vel.publish(msg)

    def _send_actuator(self, pub, value: float):
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    # =======================================================================
    # Boustrophedon sweep pattern generator
    # =======================================================================

    def _generate_sweep_waypoints(self) -> List[Tuple[float, float, float]]:
        """
        Generate a boustrophedon (lawnmower) waypoint list covering the
        open-field area.  Lane width = intake width.
        
        Open-field sweep area (approximate):
          X: from 0.40m (east of Rendezvous Pads) to field_length - 0.10m
          Y: from 0.10m (south margin) to field_width - 0.10m (north margin)
          
        CALIBRATE: Adjust margins based on field obstacles and robot dimensions.
        """
        x_start = 0.40   # clear of west-side Rendezvous Pads
        x_end = self._field_length - 0.10
        y_start = 0.10   # south margin
        y_end = self._field_width - 0.10

        lane_width = self._intake_width
        waypoints = []

        # Number of lanes
        num_lanes = max(1, int((y_end - y_start) / lane_width))
        heading_east = 0.0
        heading_west = math.pi

        for lane in range(num_lanes):
            y = y_start + lane * lane_width + lane_width / 2.0
            if y > y_end:
                break

            if lane % 2 == 0:
                # East-bound lane
                waypoints.append((x_start, y, heading_east))
                waypoints.append((x_end, y, heading_east))
            else:
                # West-bound lane
                waypoints.append((x_end, y, heading_west))
                waypoints.append((x_start, y, heading_west))

        self.get_logger().info(
            f'Generated {len(waypoints)} sweep waypoints '
            f'({num_lanes} lanes, width={lane_width:.2f}m)')
        return waypoints

    # =======================================================================
    # Main state machine tick (50 Hz)
    # =======================================================================

    def _tick(self):
        self._publish_state()
        self._time_remaining_s = self._get_time_remaining()

        # Global end-of-match check (overrides everything)
        if (self._match_start_time is not None and
                self._time_remaining_s <= 0.5 and
                self._state != MatchState.END_OF_MATCH):
            self._transition_to(MatchState.END_OF_MATCH)

        # Dispatch to state handler
        handler = {
            MatchState.INIT: self._state_init,
            MatchState.WAIT_START: self._state_wait_start,
            MatchState.DEPART_LANDING: self._state_depart_landing,
            MatchState.READ_TELEMETRY: self._state_read_telemetry,
            MatchState.DEPOSIT_BEACON: self._state_deposit_beacon,
            MatchState.COLLECT_MATERIALS: self._state_collect_materials,
            MatchState.LOAD_CSCS: self._state_load_cscs,
            MatchState.DELIVER_CSCS: self._state_deliver_cscs,
            MatchState.CAVE_ENTRY: self._state_cave_entry,
            MatchState.END_OF_MATCH: self._state_end_of_match,
            MatchState.FAULT: self._state_fault,
        }.get(self._state, self._state_fault)

        handler()

    # =======================================================================
    # State handlers
    # =======================================================================

    def _state_init(self):
        """System initialization.  Set servos to home, verify sensors."""
        # Set all actuators to safe positions
        self._send_actuator(self.pub_beacon_arm, self._beacon_arm_home)
        self._send_actuator(self.pub_container_arm, self._container_arm_release)
        self._send_actuator(self.pub_sort_gate, self._sort_gate_default)
        self._send_cmd_vel(0, 0, 0)

        # Transition immediately to WAIT_START
        self._transition_to(MatchState.WAIT_START)

    def _state_wait_start(self):
        """Robot stationary, waiting for start signal."""
        self._send_cmd_vel(0, 0, 0)
        # Start signal triggers transition via _cb_start_signal callback

    def _state_depart_landing(self):
        """
        Move out of the Landing Site ASAP for the departure bonus.
        Must exit within 3 seconds of start signal for +5 bonus points.
        """
        arrived = self._navigate_to(*self._depart_waypoint)

        if arrived:
            self.get_logger().info(
                f'Departed Landing Site in {self._get_phase_elapsed():.2f}s')
            self._transition_to(MatchState.READ_TELEMETRY)
        elif self._get_phase_elapsed() > self._budget_depart:
            # Force transition even if not perfectly at waypoint
            self.get_logger().warn('Depart timeout – forcing transition')
            self._transition_to(MatchState.READ_TELEMETRY)

    def _state_read_telemetry(self):
        """
        Navigate to position where Beacon Mast AprilTag is visible.
        Wait for telemetry_decoder_node to publish the target pad ID.
        """
        arrived = self._navigate_to(*self._telemetry_read_pos)

        if arrived and self._target_pad >= 0:
            self.get_logger().info(
                f'Telemetry decoded: pad {self._target_pad}')
            self._transition_to(MatchState.DEPOSIT_BEACON)
        elif self._get_phase_elapsed() > self._budget_read_telemetry:
            if self._target_pad < 0:
                # Fallback: use pad 2 (middle) as default
                self._target_pad = 2
                self.get_logger().warn(
                    f'Telemetry timeout – defaulting to pad {self._target_pad}')
            self._transition_to(MatchState.DEPOSIT_BEACON)

    def _state_deposit_beacon(self):
        """
        Execute the beacon deposit sub-state sequence:
        APPROACH_MAST → ALIGN_TO_MAST → EXTEND_ARM → DEPOSIT → RETRACT_ARM
        """
        if self._beacon_sub == BeaconSubState.APPROACH_MAST:
            arrived = self._navigate_to(*self._beacon_approach_pos)
            if arrived:
                self._beacon_sub = BeaconSubState.ALIGN_TO_MAST
                self.get_logger().info('Beacon: approach complete, aligning')

        elif self._beacon_sub == BeaconSubState.ALIGN_TO_MAST:
            # Fine alignment using current pose (AprilTag should be visible)
            # Drive slowly toward mast
            target_x = self._beacon_mast_x + 0.15  # CALIBRATE: final standoff
            arrived = self._navigate_to(
                target_x, self._beacon_mast_y, math.pi)
            if arrived:
                self._beacon_sub = BeaconSubState.EXTEND_ARM
                self._sub_state_timer = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info('Beacon: aligned, extending arm')

        elif self._beacon_sub == BeaconSubState.EXTEND_ARM:
            self._send_cmd_vel(0, 0, 0)
            self._send_actuator(self.pub_beacon_arm, self._beacon_arm_extended)
            # Wait for servo to reach position
            elapsed = (self.get_clock().now().nanoseconds / 1e9) - self._sub_state_timer
            if elapsed > 1.0:  # 1 second for arm extension
                self._beacon_sub = BeaconSubState.DEPOSIT
                self._sub_state_timer = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info('Beacon: arm extended, holding for deposit')

        elif self._beacon_sub == BeaconSubState.DEPOSIT:
            self._send_cmd_vel(0, 0, 0)
            # Hold arm position for 500ms to let beacon settle
            elapsed = (self.get_clock().now().nanoseconds / 1e9) - self._sub_state_timer
            if elapsed > 0.5:
                self._beacon_sub = BeaconSubState.RETRACT_ARM
                self._sub_state_timer = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info('Beacon: deposited, retracting arm')

        elif self._beacon_sub == BeaconSubState.RETRACT_ARM:
            self._send_actuator(self.pub_beacon_arm, self._beacon_arm_home)
            elapsed = (self.get_clock().now().nanoseconds / 1e9) - self._sub_state_timer
            if elapsed > 0.8:  # allow time for retraction
                self._beacon_sub = BeaconSubState.DONE
                self.get_logger().info('Beacon: DEPOSIT COMPLETE (40 pts)')

        elif self._beacon_sub == BeaconSubState.DONE:
            self._beacon_sub = BeaconSubState.APPROACH_MAST  # reset for next time
            self._transition_to(MatchState.COLLECT_MATERIALS)

        # Phase timeout
        if self._get_phase_elapsed() > self._budget_deposit_beacon:
            self.get_logger().warn('Beacon deposit timeout – skipping')
            self._send_actuator(self.pub_beacon_arm, self._beacon_arm_home)
            self._beacon_sub = BeaconSubState.APPROACH_MAST
            self._transition_to(MatchState.COLLECT_MATERIALS)

    def _state_collect_materials(self):
        """
        Execute boustrophedon sweep pattern.
        Hall sensor events trigger sort gate automatically on the roboRIO.
        """
        # Generate sweep waypoints on first entry
        if not self._sweep_waypoints:
            self._sweep_waypoints = self._generate_sweep_waypoints()
            self._current_waypoint_idx = 0

        # Follow waypoints
        if self._current_waypoint_idx < len(self._sweep_waypoints):
            wp = self._sweep_waypoints[self._current_waypoint_idx]
            arrived = self._navigate_to(*wp)
            if arrived:
                self._current_waypoint_idx += 1
        else:
            # Sweep complete
            self.get_logger().info(
                f'Sweep complete. Collected {self._total_collected} materials')
            self._transition_to(MatchState.LOAD_CSCS)
            return

        # Time budget check
        if self._get_phase_elapsed() > self._budget_collect:
            self.get_logger().warn(
                f'Collection timeout at waypoint {self._current_waypoint_idx}/'
                f'{len(self._sweep_waypoints)}')
            self._transition_to(MatchState.LOAD_CSCS)

    def _state_load_cscs(self):
        """
        Materials are already sorted into CSCs by the Hall sensor + sort gate.
        This state handles any final loading actions.
        For Strategy A, transition directly to delivery.
        """
        # In the current design, materials are sorted inline during collection.
        # No separate loading step is needed unless the design changes.
        self.get_logger().info('CSCs loaded (inline sorting during collection)')
        self._transition_to(MatchState.DELIVER_CSCS)

    def _state_deliver_cscs(self):
        """
        Drag CSCs to Rendezvous Pads.
        1. Navigate to Geodinium CSC position → latch → drag to target pad
        2. Navigate to Nebulite CSC position → latch → drag to any other pad
        """
        # Determine target pad positions
        telemetry_pad_pos = self._pad_positions[self._target_pad]

        # Pick a non-telemetry pad for the second CSC
        other_pad = 0 if self._target_pad != 0 else 1
        other_pad_pos = self._pad_positions[other_pad]

        # Simple two-phase delivery using waypoint list
        if not self._waypoints:
            self._waypoints = [
                # Phase 1: Go to Geodinium CSC
                (*self._geodinium_csc_pos, 0.0),
                # Phase 2: Drag to telemetry pad (30 pts)
                (*telemetry_pad_pos, math.pi),  # face west for pad alignment
                # Phase 3: Release, go to Nebulite CSC
                (*self._nebulite_csc_pos, 0.0),
                # Phase 4: Drag to other pad (15 pts)
                (*other_pad_pos, math.pi),
            ]
            self._current_waypoint_idx = 0

        if self._current_waypoint_idx < len(self._waypoints):
            wp = self._waypoints[self._current_waypoint_idx]
            arrived = self._navigate_to(*wp)

            if arrived:
                idx = self._current_waypoint_idx

                if idx == 0:
                    # At Geodinium CSC – latch it
                    self._send_actuator(self.pub_container_arm,
                                       self._container_arm_latch)
                    self.get_logger().info('Latched Geodinium CSC')
                elif idx == 1:
                    # At telemetry pad – release
                    self._send_actuator(self.pub_container_arm,
                                       self._container_arm_release)
                    self.get_logger().info(
                        f'Delivered Geodinium CSC to pad {self._target_pad} (30 pts)')
                elif idx == 2:
                    # At Nebulite CSC – latch it
                    self._send_actuator(self.pub_container_arm,
                                       self._container_arm_latch)
                    self.get_logger().info('Latched Nebulite CSC')
                elif idx == 3:
                    # At other pad – release
                    self._send_actuator(self.pub_container_arm,
                                       self._container_arm_release)
                    self.get_logger().info(
                        f'Delivered Nebulite CSC to pad {other_pad} (15 pts)')

                self._current_waypoint_idx += 1
        else:
            # Both CSCs delivered
            if self._time_remaining_s > self._min_time_for_cave:
                self._transition_to(MatchState.CAVE_ENTRY)
            else:
                self._transition_to(MatchState.END_OF_MATCH)
            return

        # Time budget
        if self._get_phase_elapsed() > self._budget_deliver:
            self.get_logger().warn('Delivery timeout')
            self._send_actuator(self.pub_container_arm,
                               self._container_arm_release)
            if self._time_remaining_s > self._min_time_for_cave:
                self._transition_to(MatchState.CAVE_ENTRY)
            else:
                self._transition_to(MatchState.END_OF_MATCH)

    def _state_cave_entry(self):
        """
        Navigate into the cave for 15 bonus points.
        Only attempted if sufficient time remains.
        """
        if self._time_remaining_s < 10.0:
            self.get_logger().info('Not enough time for cave – ending match')
            self._transition_to(MatchState.END_OF_MATCH)
            return

        arrived = self._navigate_to(*self._cave_entrance_pos)
        if arrived:
            self.get_logger().info('CAVE ENTRY COMPLETE (15 pts)')
            self._transition_to(MatchState.END_OF_MATCH)

        if self._get_phase_elapsed() > self._budget_cave:
            self.get_logger().warn('Cave entry timeout')
            self._transition_to(MatchState.END_OF_MATCH)

    def _state_end_of_match(self):
        """
        Zero all motors and hold.  All servos to safe positions.
        This is the terminal state.
        """
        self._send_cmd_vel(0, 0, 0)
        self._send_actuator(self.pub_beacon_arm, self._beacon_arm_home)
        self._send_actuator(self.pub_container_arm, self._container_arm_release)
        self._send_actuator(self.pub_sort_gate, self._sort_gate_default)

    def _state_fault(self):
        """
        Error recovery state.  Stop all motion and hold position.
        Log the fault for debugging.
        """
        self._send_cmd_vel(0, 0, 0)
        self._send_actuator(self.pub_beacon_arm, self._beacon_arm_home)
        self._send_actuator(self.pub_container_arm, self._container_arm_release)
        self.get_logger().error(
            f'FAULT state entered at T-{self._time_remaining_s:.1f}s')


# ===========================================================================
# Entry point
# ===========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MissionStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
