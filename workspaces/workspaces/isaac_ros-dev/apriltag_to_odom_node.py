#!/usr/bin/env python3
# ===========================================================================
# apriltag_to_odom_node.py
# Converts wall AprilTag detections into robot pose corrections for the EKF.
#
# Subscribes:  /apriltag/detections  (AprilTagDetectionArray)
# Publishes:   /apriltag/pose_correction  (PoseWithCovarianceStamped)
#
# Only processes wall tags: ID 5 (north), ID 6 (south), ID 7 (east).
# Telemetry tags (IDs 0–4) are ignored — handled by telemetry_decoder_node.
#
# Math (all transforms as 4x4 homogeneous matrices):
#   T_camera_tag  = detection pose  (tag in camera_link frame)
#   T_map_tag     = TF static       (known field position of each wall tag)
#   T_base_camera = TF static       (camera mount on robot)
#
#   T_map_camera = T_map_tag  · inv(T_camera_tag)
#   T_map_base   = T_map_camera · inv(T_base_camera)
#
# Result published in 'odom' frame — pragmatic for this small field (<2.5m)
# where map/odom drift between AprilTag fixes is negligible.
#
# Run inside the Isaac ROS dev container (needs isaac_ros_apriltag_interfaces).
#
# Usage:
#   python3 apriltag_to_odom_node.py
# ===========================================================================

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray


# ---------------------------------------------------------------------------
# Pure-numpy transform helpers (no scipy dependency)
# ---------------------------------------------------------------------------

def quat_to_matrix(x, y, z, w):
    """Quaternion → 3×3 rotation matrix."""
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
    """3×3 rotation matrix → quaternion (x, y, z, w)."""
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


def pose_to_mat(pose: Pose) -> np.ndarray:
    """geometry_msgs/Pose → 4×4 homogeneous transform matrix."""
    mat = np.eye(4)
    mat[:3, :3] = quat_to_matrix(
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w)
    mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return mat


def tf_to_mat(tf_stamped) -> np.ndarray:
    """geometry_msgs/TransformStamped → 4×4 homogeneous transform matrix."""
    t = tf_stamped.transform
    mat = np.eye(4)
    mat[:3, :3] = quat_to_matrix(
        t.rotation.x, t.rotation.y,
        t.rotation.z, t.rotation.w)
    mat[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return mat


def mat_to_pose(mat: np.ndarray) -> Pose:
    """4×4 homogeneous transform matrix → geometry_msgs/Pose."""
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


def yaw_from_mat(mat: np.ndarray) -> float:
    """Extract yaw angle (radians) from rotation matrix."""
    return math.atan2(mat[1, 0], mat[0, 0])


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class AprilTagToOdomNode(Node):

    # Wall tag IDs — telemetry tags (0–4) are intentionally excluded
    WALL_TAG_IDS = {5, 6, 7}

    # TF frame names for each wall tag (must match static_transform_publisher)
    TAG_FRAMES = {5: 'tag_5', 6: 'tag_6', 7: 'tag_7'}

    def __init__(self):
        super().__init__('apriltag_to_odom_node')

        # ---------------------------------------------------------------
        # Parameters — match ekf.yaml and jetson_bringup.launch.py
        # ---------------------------------------------------------------
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame',   'base_link')
        self.declare_parameter('map_frame',    'map')
        self.declare_parameter('output_frame', 'odom')

        # Minimum tag pixel size to accept (rejects tiny far detections)
        # Tag corners span roughly (tag_half_px * 2) pixels across.
        # At 640×480 and 2.4m field, a 6" tag at worst case is ~21px wide.
        # Accept anything > 10px half-span to exclude garbage detections.
        self.declare_parameter('min_tag_half_px', 10.0)

        # Base position covariance (m²) — scales linearly with distance
        self.declare_parameter('pos_cov_base',  0.005)
        self.declare_parameter('pos_cov_slope', 0.010)

        # Yaw covariance (rad²) — scales linearly with distance
        self.declare_parameter('yaw_cov_base',  0.003)
        self.declare_parameter('yaw_cov_slope', 0.005)

        self._camera_frame = self.get_parameter('camera_frame').value
        self._base_frame   = self.get_parameter('base_frame').value
        self._map_frame    = self.get_parameter('map_frame').value
        self._output_frame = self.get_parameter('output_frame').value
        self._min_half_px  = self.get_parameter('min_tag_half_px').value

        # ---------------------------------------------------------------
        # TF buffer — used to look up known tag and camera positions
        # ---------------------------------------------------------------
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---------------------------------------------------------------
        # QoS
        # ---------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---------------------------------------------------------------
        # Pub / Sub
        # ---------------------------------------------------------------
        self._sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self._cb_detections,
            sensor_qos,
        )

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/apriltag/pose_correction',
            reliable_qos,
        )

        self.get_logger().info(
            f'apriltag_to_odom_node started | '
            f'camera={self._camera_frame} base={self._base_frame} '
            f'map={self._map_frame} → publishing to /apriltag/pose_correction'
        )

    # -----------------------------------------------------------------------
    # Detection callback
    # -----------------------------------------------------------------------

    def _cb_detections(self, msg: AprilTagDetectionArray):
        for det in msg.detections:
            # Only wall tags
            if det.id not in self.WALL_TAG_IDS:
                continue

            # Basic quality gate: tag must be large enough in image
            if not self._tag_size_ok(det):
                self.get_logger().debug(
                    f'Tag {det.id}: too small in image, skipping')
                continue

            tag_frame = self.TAG_FRAMES[det.id]

            # -- Look up T_map_tag (known field position) ------------------
            try:
                tf_map_tag = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    tag_frame,
                    rclpy.time.Time(),          # latest available
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException) as e:
                self.get_logger().warn(
                    f'TF {self._map_frame}→{tag_frame} unavailable: {e}',
                    throttle_duration_sec=5.0)
                continue

            # -- Look up T_base_camera (static camera mount) ---------------
            try:
                tf_base_camera = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    self._camera_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException) as e:
                self.get_logger().warn(
                    f'TF {self._base_frame}→{self._camera_frame} unavailable: {e}',
                    throttle_duration_sec=5.0)
                continue

            # -- Compute T_map_base ----------------------------------------
            # T_camera_tag: tag expressed in camera frame (from detection)
            # isaac_ros_apriltag pose.pose.pose has frame_id='' but is in camera frame
            T_camera_tag  = pose_to_mat(det.pose.pose.pose)
            T_map_tag     = tf_to_mat(tf_map_tag)
            T_base_camera = tf_to_mat(tf_base_camera)

            # T_map_camera = T_map_tag · inv(T_camera_tag)
            T_map_camera = T_map_tag @ np.linalg.inv(T_camera_tag)

            # T_map_base = T_map_camera · inv(T_base_camera)
            T_map_base = T_map_camera @ np.linalg.inv(T_base_camera)

            robot_pose = mat_to_pose(T_map_base)

            # Distance from camera to tag (z in camera frame)
            dist = float(T_camera_tag[2, 3])

            # -- Build output message ---------------------------------------
            out = PoseWithCovarianceStamped()
            out.header.stamp    = msg.header.stamp
            out.header.frame_id = self._output_frame
            out.pose.pose       = robot_pose

            # Covariance [x, y, z, roll, pitch, yaw] — 6×6 row-major
            # Position and yaw uncertainties grow linearly with distance.
            # z / roll / pitch are set high — EKF 2D mode ignores them.
            pos_cov = (self.get_parameter('pos_cov_base').value +
                       self.get_parameter('pos_cov_slope').value * dist)
            yaw_cov = (self.get_parameter('yaw_cov_base').value +
                       self.get_parameter('yaw_cov_slope').value * dist)

            cov = [0.0] * 36
            cov[0]  = pos_cov   # x variance
            cov[7]  = pos_cov   # y variance
            cov[14] = 1.0       # z  (not fused in 2D mode)
            cov[21] = 1.0       # roll  (not fused)
            cov[28] = 1.0       # pitch (not fused)
            cov[35] = yaw_cov   # yaw variance
            out.pose.covariance = cov

            self._pub.publish(out)

            self.get_logger().info(
                f'Tag {det.id} | robot x={robot_pose.position.x:.3f} '
                f'y={robot_pose.position.y:.3f} '
                f'yaw={math.degrees(yaw_from_mat(T_map_base)):.1f}° '
                f'| dist={dist:.2f}m cov_xy={pos_cov:.4f}'
            )

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _tag_size_ok(self, det) -> bool:
        """Return True if the tag occupies enough pixels to be reliable."""
        if not det.corners:
            return False
        xs = [c.x for c in det.corners]
        ys = [c.y for c in det.corners]
        half_span = max(
            (max(xs) - min(xs)) / 2.0,
            (max(ys) - min(ys)) / 2.0,
        )
        return half_span >= self._min_half_px


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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
