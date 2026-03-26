#!/usr/bin/env python3
# ===========================================================================
# apriltag_to_odom_node.py
# Converts wall AprilTag detections into EKF pose corrections.
#
# Subscribes:  /apriltag/detections          (AprilTagDetectionArray)
# Publishes:   /apriltag/pose_correction     (PoseWithCovarianceStamped)
#
# Wall tag IDs (5, 6, 7) are loaded from ROS 2 parameters (apriltag_config.yaml).
# Telemetry tags (0-4) are explicitly ignored -- handled by telemetry_decoder_node.
# Add bench test IDs via bench_extra_ids parameter without editing source.
#
# Math (all transforms as 4x4 homogeneous matrices):
#   T_camera_tag  = detection pose      (tag expressed in camera_link frame)
#   T_map_tag     = TF static publisher (known field position of each wall tag)
#   T_base_camera = TF static publisher (camera mount on robot)
#
#   T_map_camera = T_map_tag  x inv(T_camera_tag)
#   T_map_base   = T_map_camera x inv(T_base_camera)
#
# Output published in 'odom' frame. On this small field (<2.5m) map approx odom
# drift between AprilTag fixes is negligible.
#
# MUST RUN inside Isaac ROS dev container (needs isaac_ros_apriltag_interfaces).
# ===========================================================================

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray


# ---------------------------------------------------------------------------
# Pure-numpy SE(3) helpers
# ---------------------------------------------------------------------------

def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-10:
        return np.eye(3)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def matrix_to_quat(R):
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def pose_to_mat(pose):
    mat = np.eye(4)
    mat[:3, :3] = quat_to_matrix(
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w)
    mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return mat


def tf_stamped_to_mat(tf_stamped):
    t = tf_stamped.transform
    mat = np.eye(4)
    mat[:3, :3] = quat_to_matrix(
        t.rotation.x, t.rotation.y,
        t.rotation.z, t.rotation.w)
    mat[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return mat


def mat_to_pose(mat):
    pose = Pose()
    pose.position.x = float(mat[0, 3])
    pose.position.y = float(mat[1, 3])
    pose.position.z = float(mat[2, 3])
    x, y, z, w = matrix_to_quat(mat[:3, :3])
    pose.orientation.x = float(x)
    pose.orientation.y = float(y)
    pose.orientation.z = float(z)
    pose.orientation.w = float(w)
    return pose


def yaw_from_mat(mat):
    return math.atan2(float(mat[1, 0]), float(mat[0, 0]))


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class AprilTagToOdomNode(Node):

    def __init__(self):
        super().__init__('apriltag_to_odom_node')

        # Parameters (loaded from apriltag_config.yaml via launch file)
        self.declare_parameter('wall_tag_ids',    [5, 6, 7])
        self.declare_parameter('wall_tag_frames', ['tag_5', 'tag_6', 'tag_7'])
        self.declare_parameter('bench_extra_ids', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('camera_frame',    'camera_link')
        self.declare_parameter('base_frame',      'base_link')
        self.declare_parameter('map_frame',       'map')
        self.declare_parameter('output_frame',    'odom')
        self.declare_parameter('min_tag_half_px', 10.0)
        self.declare_parameter('pos_cov_base',    0.005)
        self.declare_parameter('pos_cov_slope',   0.010)
        self.declare_parameter('yaw_cov_base',    0.003)
        self.declare_parameter('yaw_cov_slope',   0.005)

        # Build tag-ID -> TF-frame mapping
        wall_ids    = list(self.get_parameter('wall_tag_ids').value)
        wall_frames = list(self.get_parameter('wall_tag_frames').value)
        bench_ids = [int(x) for x in self.get_parameter('bench_extra_ids').value if int(x) >= 0]

        if len(wall_ids) != len(wall_frames):
            raise ValueError(
                f'wall_tag_ids len ({len(wall_ids)}) != '
                f'wall_tag_frames len ({len(wall_frames)})')

        self._tag_frames = dict(zip(wall_ids, wall_frames))

        # Bench extra IDs auto-generate frame name as tag_{id}
        for bid in bench_ids:
            if bid not in self._tag_frames:
                self._tag_frames[bid] = f'tag_{bid}'

        self._wall_tag_ids = set(self._tag_frames.keys())

        self._camera_frame = self.get_parameter('camera_frame').value
        self._base_frame   = self.get_parameter('base_frame').value
        self._map_frame    = self.get_parameter('map_frame').value
        self._output_frame = self.get_parameter('output_frame').value
        self._min_half_px  = self.get_parameter('min_tag_half_px').value

        # TF buffer
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self._sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self._cb_detections,
            sensor_qos)

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/apriltag/pose_correction',
            reliable_qos)

        self.get_logger().info(
            f'apriltag_to_odom_node ready | '
            f'wall_tag_ids={sorted(self._wall_tag_ids)} | '
            f'output_frame={self._output_frame}')

    def _cb_detections(self, msg: AprilTagDetectionArray):
        for det in msg.detections:
            # Ignore telemetry tags (0-4) and anything not in our map
            if det.id not in self._wall_tag_ids:
                continue

            if not self._tag_size_ok(det):
                self.get_logger().debug(f'Tag {det.id}: too small, skipping')
                continue

            tag_frame = self._tag_frames[det.id]

            try:
                tf_map_tag = self._tf_buffer.lookup_transform(
                    self._map_frame, tag_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05))
            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException) as e:
                self.get_logger().warn(
                    f'TF {self._map_frame}->{tag_frame} unavailable: {e}',
                    throttle_duration_sec=5.0)
                continue

            try:
                tf_base_camera = self._tf_buffer.lookup_transform(
                    self._base_frame, self._camera_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05))
            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException) as e:
                self.get_logger().warn(
                    f'TF {self._base_frame}->{self._camera_frame} unavailable: {e}',
                    throttle_duration_sec=5.0)
                continue

            T_camera_tag  = pose_to_mat(det.pose.pose.pose)
            T_map_tag     = tf_stamped_to_mat(tf_map_tag)
            T_base_camera = tf_stamped_to_mat(tf_base_camera)

            T_map_camera = T_map_tag @ np.linalg.inv(T_camera_tag)
            T_map_base   = T_map_camera @ np.linalg.inv(T_base_camera)

            robot_pose = mat_to_pose(T_map_base)
            dist = float(T_camera_tag[2, 3])

            out = PoseWithCovarianceStamped()
            out.header.stamp    = msg.header.stamp
            out.header.frame_id = self._output_frame
            out.pose.pose       = robot_pose

            pos_cov = (self.get_parameter('pos_cov_base').value +
                       self.get_parameter('pos_cov_slope').value * dist)
            yaw_cov = (self.get_parameter('yaw_cov_base').value +
                       self.get_parameter('yaw_cov_slope').value * dist)

            cov = [0.0] * 36
            cov[0]  = pos_cov
            cov[7]  = pos_cov
            cov[14] = 1.0   # z  -- ignored by 2D EKF
            cov[21] = 1.0   # roll -- ignored
            cov[28] = 1.0   # pitch -- ignored
            cov[35] = yaw_cov
            out.pose.covariance = cov

            self._pub.publish(out)
            self.get_logger().info(
                f'Tag {det.id} | robot x={robot_pose.position.x:.3f} '
                f'y={robot_pose.position.y:.3f} '
                f'yaw={math.degrees(yaw_from_mat(T_map_base)):.1f}deg | '
                f'dist={dist:.2f}m cov_xy={pos_cov:.4f}')

    def _tag_size_ok(self, det) -> bool:
        if not det.corners:
            return False
        xs = [c.x for c in det.corners]
        ys = [c.y for c in det.corners]
        half_span = max((max(xs) - min(xs)) / 2.0, (max(ys) - min(ys)) / 2.0)
        return half_span >= self._min_half_px


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
