#!/usr/bin/env python3
# ===========================================================================
# start_led_detector_node.py
# Detects the NeoPixel Start LED and publishes /match/start_signal.
#
# Subscribes:  /led_cam/image_raw     (sensor_msgs/Image)
# Publishes:   /match/start_signal    (std_msgs/Bool)
#
# Camera: rear-facing USB webcam (/dev/video1) aimed at south wall Start LED.
# The Arducam CSI camera is front-facing for AprilTags.
#
# Algorithm: ROI brightness thresholding
#   1. Convert incoming frame to grayscale
#   2. Crop to ROI (pixel region where LED appears in frame)
#   3. Compute mean pixel intensity in ROI
#   4. If mean > brightness_threshold for min_consecutive_frames: trigger
#   5. Latch — once triggered, keep publishing True forever
#
# CALIBRATION PROCEDURE (run once before competition):
#   Step 1: Place robot in Landing Site facing north (intake toward field)
#   Step 2: ros2 run rqt_image_view rqt_image_view → subscribe /led_cam/image_raw
#   Step 3: Note pixel (x, y) where Start LED appears in the 640x480 frame
#   Step 4: Set roi_x = LED_x - 40, roi_y = LED_y - 40, roi_w = 80, roi_h = 80
#   Step 5: Run with LED OFF → read "ROI brightness" from node log → that is baseline (~30-60)
#   Step 6: Run with LED ON  → read "ROI brightness" from node log → that is trigger (~180-220)
#   Step 7: Set brightness_threshold = (baseline + trigger) / 2  (e.g. 130)
#   Step 8: Update apriltag_config.yaml with measured values and redeploy
#
# All parameters live in apriltag_config.yaml under start_led_detector_node:
#   roi_x, roi_y, roi_w, roi_h, brightness_threshold, min_consecutive_frames
# ===========================================================================

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from sensor_msgs.msg import Image

# Try cv_bridge — use it if available, fall back to manual decoding otherwise.
try:
    from cv_bridge import CvBridge
    import cv2
    _CV_BRIDGE_AVAILABLE = True
except ImportError:
    _CV_BRIDGE_AVAILABLE = False


class StartLedDetectorNode(Node):

    def __init__(self):
        super().__init__('start_led_detector_node')

        # ------------------------------------------------------------------
        # Parameters (set in apriltag_config.yaml)
        # ------------------------------------------------------------------
        self.declare_parameter('image_topic',            '/led_cam/image_raw')
        self.declare_parameter('roi_x',                  280)
        self.declare_parameter('roi_y',                  200)
        self.declare_parameter('roi_w',                  80)
        self.declare_parameter('roi_h',                  80)
        self.declare_parameter('brightness_threshold',   150.0)
        self.declare_parameter('min_consecutive_frames', 3)
        self.declare_parameter('latch_signal',           True)
        self.declare_parameter('latch_publish_rate_hz',  10.0)

        self._roi_x      = int(self.get_parameter('roi_x').value)
        self._roi_y      = int(self.get_parameter('roi_y').value)
        self._roi_w      = int(self.get_parameter('roi_w').value)
        self._roi_h      = int(self.get_parameter('roi_h').value)
        self._threshold  = float(self.get_parameter('brightness_threshold').value)
        self._min_frames = int(self.get_parameter('min_consecutive_frames').value)
        self._latch      = bool(self.get_parameter('latch_signal').value)
        image_topic      = str(self.get_parameter('image_topic').value)

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._consecutive_count = 0
        self._triggered         = False

        if _CV_BRIDGE_AVAILABLE:
            self._bridge = CvBridge()
            self.get_logger().info('cv_bridge available — using optimised path')
        else:
            self.get_logger().info(
                'cv_bridge not found — using manual decode (rgb8/bgr8/yuyv/mono8)')

        # ------------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self._sub = self.create_subscription(
            Image, image_topic, self._cb_image, sensor_qos)

        self._pub = self.create_publisher(
            Bool, '/match/start_signal', reliable_qos)

        rate = float(self.get_parameter('latch_publish_rate_hz').value)
        self._latch_timer = self.create_timer(1.0 / rate, self._publish_latched)

        self.get_logger().info(
            f'start_led_detector ready | '
            f'topic={image_topic} | '
            f'ROI=({self._roi_x},{self._roi_y},{self._roi_w},{self._roi_h}) | '
            f'threshold={self._threshold} | '
            f'min_frames={self._min_frames}')

    # -----------------------------------------------------------------------
    # Image callback
    # -----------------------------------------------------------------------

    def _cb_image(self, msg: Image):
        if self._latch and self._triggered:
            return

        try:
            mean_brightness = self._extract_roi_brightness(msg)
        except Exception as e:
            self.get_logger().warn(
                f'Image processing error: {e}', throttle_duration_sec=5.0)
            return

        # Log ROI brightness every frame at DEBUG level.
        # During calibration: temporarily change to INFO to see it in terminal.
        self.get_logger().debug(f'ROI mean brightness: {mean_brightness:.1f}')

        if mean_brightness >= self._threshold:
            self._consecutive_count += 1
            self.get_logger().debug(
                f'LED bright ({mean_brightness:.1f} >= {self._threshold}) | '
                f'count {self._consecutive_count}/{self._min_frames}')
        else:
            if self._consecutive_count > 0:
                self.get_logger().debug(
                    f'LED dim ({mean_brightness:.1f}), resetting streak')
            self._consecutive_count = 0

        if self._consecutive_count >= self._min_frames and not self._triggered:
            self._triggered = True
            self.get_logger().info(
                f'*** START LED DETECTED *** '
                f'brightness={mean_brightness:.1f} — '
                f'publishing /match/start_signal = True')
            self._publish_latched()

    # -----------------------------------------------------------------------
    # ROI brightness extraction
    # Uses cv_bridge if available; falls back to manual numpy decode.
    # Supports encodings: rgb8, bgr8, yuyv / yuv422, mono8.
    # -----------------------------------------------------------------------

    def _extract_roi_brightness(self, msg: Image) -> float:
        h, w = msg.height, msg.width
        x0 = max(0, self._roi_x)
        y0 = max(0, self._roi_y)
        x1 = min(w, self._roi_x + self._roi_w)
        y1 = min(h, self._roi_y + self._roi_h)

        if x1 <= x0 or y1 <= y0:
            raise ValueError(
                f'ROI ({x0},{y0})-({x1},{y1}) is empty for {w}x{h} image')

        if _CV_BRIDGE_AVAILABLE:
            gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            roi = gray[y0:y1, x0:x1]
            return float(np.mean(roi))

        # Manual decode path
        enc = msg.encoding.lower()
        raw = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ('rgb8', 'bgr8'):
            img = raw.reshape((h, w, 3))
            if enc == 'rgb8':
                gray = (0.299 * img[:, :, 0].astype(np.float32) +
                        0.587 * img[:, :, 1].astype(np.float32) +
                        0.114 * img[:, :, 2].astype(np.float32))
            else:
                gray = (0.299 * img[:, :, 2].astype(np.float32) +
                        0.587 * img[:, :, 1].astype(np.float32) +
                        0.114 * img[:, :, 0].astype(np.float32))

        elif enc in ('yuyv', 'yuv422', 'yuyv422', 'yuv422_yuy2'):
            # YUYV: 2 bytes per pixel. Y byte at every even index.
            img_yuyv = raw.reshape((h, w * 2))
            gray = img_yuyv[:, 0::2].astype(np.float32)

        elif enc == 'mono8':
            gray = raw.reshape((h, w)).astype(np.float32)

        else:
            raise ValueError(
                f'Unsupported encoding: {enc}. '
                f'Install cv_bridge (apt install ros-humble-cv-bridge) '
                f'for automatic encoding support.')

        roi = gray[y0:y1, x0:x1]
        return float(np.mean(roi))

    # -----------------------------------------------------------------------
    # Latched publish
    # -----------------------------------------------------------------------

    def _publish_latched(self):
        if not self._triggered:
            return
        msg = Bool()
        msg.data = True
        self._pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = StartLedDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
