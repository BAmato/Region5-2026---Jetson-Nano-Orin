#!/usr/bin/env python3
# ===========================================================================
# telemetry_decoder_node.py
# Reads the Beacon Mast AprilTag and publishes the target Rendezvous Pad ID.
#
# Subscribes:  /apriltag/detections   (AprilTagDetectionArray)
# Publishes:   /match/target_pad      (std_msgs/Int8)
#
# Game logic (GM2 Section 3.3):
#   The Beacon Mast on the west wall holds a single AprilTag with a
#   RANDOMIZED ID between 0 and 4.  The ID equals the target Rendezvous Pad
#   number (0 = southmost, 4 = northmost).  This node detects that tag,
#   confirms it over N consecutive frames to prevent false positives from
#   glancing passes, then latches and re-publishes continuously.
#
# Tag position: west wall, center (pad 2 y-position).
# Tag face direction: east (facing into the field).
# We only need the ID -- NOT the pose -- for telemetry decoding.
# The pose path (wall-tag localization) is handled by apriltag_to_odom_node.
#
# MUST RUN inside Isaac ROS dev container (needs isaac_ros_apriltag_interfaces).
# ===========================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Int8
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray


class TelemetryDecoderNode(Node):

    # Sentinel: no pad confirmed yet
    NO_PAD = -1

    def __init__(self):
        super().__init__('telemetry_decoder_node')

        # Parameters (loaded from apriltag_config.yaml via launch file)
        self.declare_parameter('telemetry_tag_ids',    [0, 1, 2, 3, 4])
        self.declare_parameter('confirmation_frames',  3)
        self.declare_parameter('latch_result',         True)
        self.declare_parameter('latch_publish_rate_hz', 2.0)
        self.declare_parameter('min_tag_half_px',      8.0)

        self._telemetry_ids  = set(
            self.get_parameter('telemetry_tag_ids').value)
        self._confirm_needed = int(
            self.get_parameter('confirmation_frames').value)
        self._latch          = bool(
            self.get_parameter('latch_result').value)
        self._min_half_px    = float(
            self.get_parameter('min_tag_half_px').value)

        # State
        self._candidate_id    = self.NO_PAD   # ID currently accumulating frames
        self._confirm_count   = 0             # consecutive frames seen
        self._confirmed_pad   = self.NO_PAD   # latched result once confirmed

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self._sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self._cb_detections,
            sensor_qos)

        self._pub = self.create_publisher(
            Int8, '/match/target_pad', reliable_qos)

        # Latch re-publish timer
        rate = float(self.get_parameter('latch_publish_rate_hz').value)
        self._latch_timer = self.create_timer(
            1.0 / rate, self._publish_latched)

        self.get_logger().info(
            f'telemetry_decoder_node ready | '
            f'telemetry_ids={sorted(self._telemetry_ids)} | '
            f'confirmation={self._confirm_needed} frames')

    # -----------------------------------------------------------------------
    # Detection callback
    # -----------------------------------------------------------------------

    def _cb_detections(self, msg: AprilTagDetectionArray):
        # If already latched, nothing more to do
        if self._latch and self._confirmed_pad != self.NO_PAD:
            return

        # Find any telemetry tag in this detection batch
        detected_id = self.NO_PAD
        for det in msg.detections:
            if det.id not in self._telemetry_ids:
                continue
            if not self._tag_size_ok(det):
                self.get_logger().debug(
                    f'Telemetry tag {det.id}: too small, skipping')
                continue
            detected_id = det.id
            break   # only one telemetry tag can be in the mast at a time

        if detected_id == self.NO_PAD:
            # No telemetry tag this frame -- reset confirmation streak
            if self._candidate_id != self.NO_PAD:
                self.get_logger().debug(
                    f'Telemetry: lost tag {self._candidate_id}, '
                    f'resetting confirmation ({self._confirm_count} frames)')
            self._candidate_id  = self.NO_PAD
            self._confirm_count = 0
            return

        # Telemetry tag detected this frame
        if detected_id != self._candidate_id:
            # Different ID than previous streak -- restart
            self.get_logger().info(
                f'Telemetry: new candidate pad {detected_id} '
                f'(was {self._candidate_id}), restarting confirmation')
            self._candidate_id  = detected_id
            self._confirm_count = 1
        else:
            # Same ID as ongoing streak
            self._confirm_count += 1
            self.get_logger().debug(
                f'Telemetry: pad {detected_id} confirmation '
                f'{self._confirm_count}/{self._confirm_needed}')

        # Check if confirmation threshold reached
        if self._confirm_count >= self._confirm_needed:
            if self._confirmed_pad != detected_id:
                self._confirmed_pad = detected_id
                self.get_logger().info(
                    f'*** TELEMETRY CONFIRMED: target pad = {self._confirmed_pad} ***')
                # Immediately publish on confirmation
                self._publish_latched()

    # -----------------------------------------------------------------------
    # Latched re-publish timer callback
    # -----------------------------------------------------------------------

    def _publish_latched(self):
        if self._confirmed_pad == self.NO_PAD:
            return
        msg = Int8()
        msg.data = int(self._confirmed_pad)
        self._pub.publish(msg)

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _tag_size_ok(self, det) -> bool:
        """Pixel-size quality gate -- looser than wall tags since mast may be angled."""
        if not det.corners:
            return False
        xs = [c.x for c in det.corners]
        ys = [c.y for c in det.corners]
        half_span = max((max(xs) - min(xs)) / 2.0, (max(ys) - min(ys)) / 2.0)
        return half_span >= self._min_half_px


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
