#!/usr/bin/env python3
# ===========================================================================
# match_watchdog_node.py
# Independent hardware safety timer. Publishes an emergency stop command
# at (match_duration - safety_margin) seconds regardless of software state.
#
# Subscribes:  /match/start_signal    (std_msgs/Bool)
# Publishes:   /match/emergency_stop  (std_msgs/Bool)
#              /cmd_vel               (geometry_msgs/Twist)  -- zeroed at match end
#
# This node is the last line of defense. It runs independently of
# mission_state_node and will halt the robot even if the state machine fails.
# Originally on Raspberry Pi 4, now consolidated onto Jetson.
#
# Design intent:
#   - The roboRIO has its own 180s match timer as primary enforcement.
#   - mission_state_node also transitions to END_OF_MATCH at T=0.
#   - This node adds a third independent timer as tertiary safety.
#   - It fires safety_margin_sec BEFORE official end to guarantee the robot
#     stops before referees could penalize continued motion.
# ===========================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class MatchWatchdogNode(Node):

    def __init__(self):
        super().__init__('match_watchdog_node')

        # Parameters (from apriltag_config.yaml)
        self.declare_parameter('match_duration_sec',   180.0)
        self.declare_parameter('safety_margin_sec',    1.0)
        self.declare_parameter('start_signal_topic',   '/match/start_signal')
        self.declare_parameter('emergency_stop_topic', '/match/emergency_stop')
        self.declare_parameter('cmd_vel_topic',        '/cmd_vel')

        self._match_duration = float(
            self.get_parameter('match_duration_sec').value)
        self._safety_margin  = float(
            self.get_parameter('safety_margin_sec').value)
        start_topic = str(self.get_parameter('start_signal_topic').value)
        estop_topic = str(self.get_parameter('emergency_stop_topic').value)
        vel_topic   = str(self.get_parameter('cmd_vel_topic').value)

        # State
        self._match_started     = False
        self._emergency_fired   = False
        self._match_start_stamp = None  # rclpy Time

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self._sub_start = self.create_subscription(
            Bool, start_topic, self._cb_start, reliable_qos)

        self._pub_estop = self.create_publisher(
            Bool, estop_topic, reliable_qos)

        self._pub_vel = self.create_publisher(
            Twist, vel_topic, sensor_qos)

        # Check timer: 10 Hz is plenty for a match-end check
        self._check_timer = self.create_timer(0.1, self._check_elapsed)

        # Keep publishing emergency stop after firing (in case consumer missed it)
        self._estop_repeat_timer = self.create_timer(0.5, self._repeat_estop)

        self.get_logger().info(
            f'match_watchdog ready | '
            f'will stop robot at T+{self._match_duration - self._safety_margin:.1f}s '
            f'after start signal')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _cb_start(self, msg: Bool):
        if msg.data and not self._match_started:
            self._match_started     = True
            self._match_start_stamp = self.get_clock().now()
            self.get_logger().info(
                f'Watchdog: match started. '
                f'Will emergency-stop in '
                f'{self._match_duration - self._safety_margin:.1f}s')

    def _check_elapsed(self):
        if not self._match_started or self._emergency_fired:
            return

        elapsed = (self.get_clock().now() -
                   self._match_start_stamp).nanoseconds / 1e9

        stop_at = self._match_duration - self._safety_margin
        time_left = stop_at - elapsed

        # Log countdown in the last 10 seconds
        if 0 < time_left <= 10.0:
            self.get_logger().info(
                f'Watchdog: {time_left:.1f}s until emergency stop',
                throttle_duration_sec=1.0)

        if elapsed >= stop_at:
            self._fire_emergency_stop(elapsed)

    def _fire_emergency_stop(self, elapsed: float):
        self._emergency_fired = True
        self.get_logger().warn(
            f'*** WATCHDOG EMERGENCY STOP *** '
            f'({elapsed:.2f}s elapsed, limit={self._match_duration - self._safety_margin:.1f}s)')

        # Publish zero velocity -- belt-and-suspenders alongside E-stop
        vel = Twist()
        self._pub_vel.publish(vel)

        # Publish E-stop Bool
        estop = Bool()
        estop.data = True
        self._pub_estop.publish(estop)

    def _repeat_estop(self):
        """Keep publishing E-stop after it fires so no subscriber misses it."""
        if not self._emergency_fired:
            return
        estop = Bool()
        estop.data = True
        self._pub_estop.publish(estop)
        # Also re-publish zero velocity in case something re-enabled motors
        self._pub_vel.publish(Twist())


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MatchWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
