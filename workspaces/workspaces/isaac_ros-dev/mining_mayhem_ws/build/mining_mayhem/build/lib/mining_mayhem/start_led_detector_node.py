#!/usr/bin/env python3
# ===========================================================================
# start_led_detector_node.py
# Detects the NeoPixel Start LED and publishes /match/start_signal.
#
# Subscribes:  /led_cam/image_raw     (sensor_msgs/Image)
# Publishes:   /match/start_signal    (std_msgs/Bool)
#
# Originally on Raspberry Pi 4, now consolidated onto Jetson (Pi elimination).
# Camera: rear-facing USB UVC webcam aimed at Landing Site south wall LED.
#
# Algorithm: ROI brightness thresholding
#   1. Convert incoming frame to grayscale
#   2. Crop to ROI (pixel region where LED appears when robot is in Landing Site)
#   3. Compute mean pixel intensity in ROI
#   4. If mean > brightness_threshold for min_consecutive_frames: trigger
#   5. Latch the signal -- once triggered, keep publishing True
#
# CALIBRATION PROCEDURE:
#   Step 1: Place robot in Landing Site
#   Step 2: Run: ros2 run rqt_image_view rqt_image_view, subscribe /led_cam/image_raw
#   Step 3: Note pixel coordinates (x, y) where Start LED appears in the frame
#   Step 4: Set roi_x, roi_y, roi_w, roi_h to bracket the LED with ~20px margin
#   Step 5: Run with LED OFF and print mean intensity -> that is your baseline (~30-60)
#   Step 6: Run with LED ON and print mean intensity -> that is your trigger (~180-220)
#   Step 7: Set brightness_threshold = (baseline + trigger) / 2
#
# IMPORTANT: All parameters are in apriltag_config.yaml under start_led_detector_node
# ===========================================================================

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from sensor_msgs.msg import Image

# TODO (chat 5.2): Install cv_bridge in the ROS workspace if not present.
# In container: apt install ros-humble-cv-bridge
# Then uncomment the cv_bridge import below.
# from cv_bridge import CvBridge


class StartLedDetectorNode(Node):

    def __init__(self):
        super().__init__('start_led_detector_node')

        # Parameters (from apriltag_config.yaml)
        self.declare_parameter('image_topic',           '/led_cam/image_raw')
        self.declare_parameter('roi_x',                 280)
        self.declare_parameter('roi_y',                 200)
        self.declare_parameter('roi_w',                 80)
        self.declare_parameter('roi_h',                 80)
        self.declare_parameter('brightness_threshold',  150)
        self.declare_parameter('min_consecutive_frames', 3)
        self.declare_parameter('latch_signal',          True)
        self.declare_parameter('latch_publish_rate_hz', 10.0)

        self._roi_x         = int(self.get_parameter('roi_x').value)
        self._roi_y         = int(self.get_parameter('roi_y').value)
        self._roi_w         = int(self.get_parameter('roi_w').value)
        self._roi_h         = int(self.get_parameter('roi_h').value)
        self._threshold     = float(self.get_parameter('brightness_threshold').value)
        self._min_frames    = int(self.get_parameter('min_consecutive_frames').value)
        self._latch         = bool(self.get_parameter('latch_signal').value)
        image_topic         = str(self.get_parameter('image_topic').value)

        # State
        self._consecutive_count = 0
        self._triggered         = False

        # TODO (chat 5.2): Uncomment when cv_bridge is available
        # self._bridge = CvBridge()

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
            f'ROI=({self._roi_x},{self._roi_y},{self._roi_w},{self._roi_h}) | '
            f'threshold={self._threshold} | '
            f'min_frames={self._min_frames}')

    # -----------------------------------------------------------------------
    # Image callback
    # -----------------------------------------------------------------------

    def _cb_image(self, msg: Image):
        # If already latched, no processing needed
        if self._latch and self._triggered:
            return

        # TODO (chat 5.2): Replace this entire block with cv_bridge once available.
        # Current implementation converts raw bytes manually for YUYV/RGB8/BGR8.
        # This is less efficient than cv_bridge but avoids the dependency.

        try:
            mean_brightness = self._extract_roi_brightness(msg)
        except Exception as e:
            self.get_logger().warn(
                f'Image processing error: {e}', throttle_duration_sec=5.0)
            return

        # Debug: log ROI brightness on every frame so calibration is easy
        # Remove or throttle this before competition
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

        if self._consecutive_count >= self._min_frames:
            if not self._triggered:
                self._triggered = True
                self.get_logger().info(
                    f'*** START LED DETECTED *** (brightness={mean_brightness:.1f}) '
                    f'-- publishing /match/start_signal = True')
                self._publish_latched()

    # -----------------------------------------------------------------------
    # ROI extraction (manual -- replace with cv_bridge when available)
    # -----------------------------------------------------------------------

    def _extract_roi_brightness(self, msg: Image) -> float:
        """
        Extract the mean grayscale brightness of the configured ROI.

        Supports encoding: rgb8, bgr8, yuyv (yuv422).
        For other encodings, this will raise ValueError -- add cases as needed.

        TODO (chat 5.2): Replace this with:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'mono8')
            roi = cv_image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
            return float(np.mean(roi))
        """
        h = msg.height
        w = msg.width
        enc = msg.encoding.lower()

        # Clip ROI to image bounds
        x0 = max(0, self._roi_x)
        y0 = max(0, self._roi_y)
        x1 = min(w, self._roi_x + self._roi_w)
        y1 = min(h, self._roi_y + self._roi_h)

        if x1 <= x0 or y1 <= y0:
            raise ValueError(f'ROI ({x0},{y0})-({x1},{y1}) is empty for image {w}x{h}')

        raw = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ('rgb8', 'bgr8'):
            # 3 bytes per pixel
            img = raw.reshape((h, w, 3))
            # Luminance approximation
            if enc == 'rgb8':
                gray = (0.299 * img[:, :, 0] +
                        0.587 * img[:, :, 1] +
                        0.114 * img[:, :, 2])
            else:
                gray = (0.299 * img[:, :, 2] +
                        0.587 * img[:, :, 1] +
                        0.114 * img[:, :, 0])
            roi = gray[y0:y1, x0:x1]

        elif enc in ('yuyv', 'yuv422', 'yuyv422'):
            # YUYV: 2 bytes per pixel, layout [Y0 U0 Y1 V0 Y2 U2 Y3 V2 ...]
            # Y byte is at even indices in each 2-byte pair
            # Full row in bytes: w * 2 bytes
            img_yuyv = raw.reshape((h, w * 2))
            # Y channel is at columns 0, 2, 4, ... (every other byte starting at 0)
            gray = img_yuyv[:, 0::2].astype(np.float32)
            roi = gray[y0:y1, x0:x1]

        elif enc == 'mono8':
            gray = raw.reshape((h, w)).astype(np.float32)
            roi = gray[y0:y1, x0:x1]

        else:
            raise ValueError(
                f'Unsupported encoding: {enc}. '
                f'Add case or switch to cv_bridge (see TODO in source).')

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
