#!/usr/bin/env python3
# ===========================================================================
# path_planner_node.py
# Simple proportional waypoint follower for Mining Mayhem.
#
# Subscribes to:
#   /goal_pose  (geometry_msgs/PoseStamped) — target waypoint
#   /odom/filtered (nav_msgs/Odometry)      — current robot pose
#
# Publishes to:
#   /cmd_vel    (geometry_msgs/Twist)        — velocity commands
#   /path_planner/status (std_msgs/String)   — IDLE / NAVIGATING / ARRIVED
# ===========================================================================

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        # Parameters
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('waypoint_tolerance', 0.03)
        self.declare_parameter('heading_tolerance', 0.05)

        self._max_linear  = self.get_parameter('max_linear_speed').value
        self._max_angular = self.get_parameter('max_angular_speed').value
        self._pos_tol     = self.get_parameter('waypoint_tolerance').value
        self._head_tol    = self.get_parameter('heading_tolerance').value

        # Robot pose
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._pose_received = False

        # Current goal
        self._goal_x:     float | None = None
        self._goal_y:     float | None = None
        self._goal_theta: float | None = None
        self._status = 'IDLE'

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

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry, '/odom/filtered', self._cb_odom, sensor_qos)
        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self._cb_goal, reliable_qos)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', sensor_qos)
        self.pub_status  = self.create_publisher(String, '/path_planner/status', reliable_qos)

        # Control loop at 50 Hz
        self._timer = self.create_timer(0.02, self._tick)

        self.get_logger().info(
            f'path_planner_node ready | '
            f'max_v={self._max_linear}m/s '
            f'max_w={self._max_angular}rad/s '
            f'pos_tol={self._pos_tol}m '
            f'head_tol={self._head_tol}rad'
        )

    # =======================================================================
    # Callbacks
    # =======================================================================

    def _cb_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._theta = math.atan2(siny_cosp, cosy_cosp)
        self._pose_received = True

    def _cb_goal(self, msg: PoseStamped):
        self._goal_x = msg.pose.position.x
        self._goal_y = msg.pose.position.y
        # Extract yaw from goal quaternion
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # Only use heading if quaternion is non-identity
        if abs(q.w - 1.0) > 0.01:
            self._goal_theta = yaw
        else:
            self._goal_theta = None
        self._status = 'NAVIGATING'
        self.get_logger().info(
            f'New goal: ({self._goal_x:.3f}, {self._goal_y:.3f}) '
            f'heading={math.degrees(yaw):.1f}°' if self._goal_theta else
            f'New goal: ({self._goal_x:.3f}, {self._goal_y:.3f}) heading=any'
        )

    # =======================================================================
    # Control loop
    # =======================================================================

    def _tick(self):
        # Publish status
        status_msg = String()
        status_msg.data = self._status
        self.pub_status.publish(status_msg)

        if not self._pose_received:
            return

        if self._goal_x is None or self._status == 'IDLE':
            self._send_cmd_vel(0.0, 0.0, 0.0)
            return

        dx = self._goal_x - self._x
        dy = self._goal_y - self._y
        distance = math.sqrt(dx * dx + dy * dy)

        # --- Position phase ---
        if distance > self._pos_tol:
            desired_heading = math.atan2(dy, dx)
            heading_err = normalize_angle(desired_heading - self._theta)

            # Speed proportional to distance, capped
            speed = min(self._max_linear, 0.8 * distance)

            # In robot frame: vx = speed * cos(err), vy = speed * sin(err)
            vx = speed * math.cos(heading_err)
            vy = speed * math.sin(heading_err)

            # Rotate toward final heading if close
            omega = 0.0
            if self._goal_theta is not None and distance < 0.15:
                final_err = normalize_angle(self._goal_theta - self._theta)
                omega = min(self._max_angular,
                            max(-self._max_angular, 1.5 * final_err))

            self._send_cmd_vel(vx, vy, omega)
            return

        # --- Heading phase (reached position) ---
        if self._goal_theta is not None:
            heading_err = normalize_angle(self._goal_theta - self._theta)
            if abs(heading_err) > self._head_tol:
                omega = min(self._max_angular,
                            max(-self._max_angular, 2.0 * heading_err))
                self._send_cmd_vel(0.0, 0.0, omega)
                return

        # --- Arrived ---
        self._send_cmd_vel(0.0, 0.0, 0.0)
        if self._status != 'ARRIVED':
            self._status = 'ARRIVED'
            self.get_logger().info(
                f'Arrived at ({self._goal_x:.3f}, {self._goal_y:.3f})')

    def _send_cmd_vel(self, vx: float, vy: float, omega: float):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.angular.z = float(omega)
        self.pub_cmd_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
