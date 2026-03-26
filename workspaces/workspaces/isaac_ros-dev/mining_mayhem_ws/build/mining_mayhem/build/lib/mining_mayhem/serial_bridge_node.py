#!/usr/bin/env python3
# ===========================================================================
# serial_bridge_node.py
# Jetson-side USB-CDC serial bridge to roboRIO.
#
# Reads binary structs from roboRIO, publishes to ROS 2 topics.
# Subscribes to ROS 2 topics, packs into binary structs for roboRIO.
#
# Runs at 50 Hz (matches roboRIO 20 ms periodic loop).
#
# Serial device: /dev/ttyACM0  (CALIBRATE: confirm with `ls /dev/ttyACM*`)
# Baud: 115200, 8N1
# ===========================================================================

import struct
import time
import math
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial

from geometry_msgs.msg import Twist, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String

# ---------------------------------------------------------------------------
# Packet definitions (must match SerialBridgeSubsystem.h exactly)
# ---------------------------------------------------------------------------

# roboRIO → Jetson: 60 bytes
# 2B magic + 1B seq + 6×f32 odom + 3×f32 imu + 3×i32 enc
# + 1B hall + 1I ts + 1B state + 1H time + 1B crc
RIO_TO_JETSON_FMT = '<2sB6f3f3iBI BH B'
RIO_TO_JETSON_SIZE = struct.calcsize(RIO_TO_JETSON_FMT)
RIO_TO_JETSON_MAGIC = b'\xA5\x5A'

# Jetson → roboRIO: 30 bytes
# 2B magic + 1B seq + 6×f32 + 1B flags + 1B reserved + 1B crc
JETSON_TO_RIO_FMT = '<2sB6fBBB'
JETSON_TO_RIO_SIZE = struct.calcsize(JETSON_TO_RIO_FMT)
JETSON_TO_RIO_MAGIC = b'\x5A\xA5'

# Verify sizes match C++ structs
assert RIO_TO_JETSON_SIZE == 60, f"RIO→Jetson size mismatch: {RIO_TO_JETSON_SIZE}"
assert JETSON_TO_RIO_SIZE == 30, f"Jetson→RIO size mismatch: {JETSON_TO_RIO_SIZE}"


def crc8(data: bytes) -> int:
    """CRC-8 polynomial 0x07, init 0x00 – matches roboRIO implementation."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle (radians) to quaternion (rotation about Z)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SerialBridgeNode(Node):
    """ROS 2 node bridging roboRIO serial data to/from ROS 2 topics."""

    def __init__(self):
        super().__init__('serial_bridge_node')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('loop_rate_hz', 50.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        rate = self.get_parameter('loop_rate_hz').value

        # Serial connection
        self.get_logger().info(f'Opening serial port {port} at {baud} baud')
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.005,   # 5ms read timeout (non-blocking-ish)
                write_timeout=0.01,
            )
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # QoS for sensor data: best-effort, keep latest
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -------------------------------------------------------------------
        # Publishers (data FROM roboRIO → ROS 2)
        # -------------------------------------------------------------------
        self.pub_odom = self.create_publisher(
            Odometry, '/odom/wheel', sensor_qos)
        self.pub_imu = self.create_publisher(
            Imu, '/imu/data', sensor_qos)
        self.pub_hall = self.create_publisher(
            Bool, '/robot/hall_event', sensor_qos)
        self.pub_match_state = self.create_publisher(
            String, '/match/state_rio', sensor_qos)
        self.pub_match_time = self.create_publisher(
            Float32, '/match/time_remaining', sensor_qos)

        # -------------------------------------------------------------------
        # Subscribers (data FROM ROS 2 → roboRIO)
        # -------------------------------------------------------------------
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self._cb_cmd_vel, sensor_qos)
        self.sub_beacon_arm = self.create_subscription(
            Float32, '/actuator/beacon_arm', self._cb_beacon_arm, sensor_qos)
        self.sub_container_arm = self.create_subscription(
            Float32, '/actuator/container_arm', self._cb_container_arm,
            sensor_qos)
        self.sub_sort_gate = self.create_subscription(
            Float32, '/actuator/sort_gate', self._cb_sort_gate, sensor_qos)
        self.sub_start_signal = self.create_subscription(
            Bool, '/match/start_signal', self._cb_start_signal, sensor_qos)

        # -------------------------------------------------------------------
        # TX state (protected by lock for thread safety)
        # -------------------------------------------------------------------
        self._tx_lock = Lock()
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_omega = 0.0
        self._beacon_arm_pos = 0.0
        self._container_arm_pos = 0.0
        self._sort_gate_pos = 0.0
        self._start_signal_fwd = False
        self._tx_seq = 0

        # RX buffer for stream parsing
        self._rx_buf = bytearray()
        self._last_rx_seq = 0
        self._rx_count = 0
        self._drop_count = 0

        # Match state name table (must match roboRIO enum)
        self._state_names = [
            'INIT', 'WAIT_START', 'DEPART_LANDING', 'READ_TELEMETRY',
            'DEPOSIT_BEACON', 'COLLECT_MATERIALS', 'LOAD_CSCS',
            'DELIVER_CSCS', 'CAVE_ENTRY', 'END_OF_MATCH', 'FAULT',
        ]

        # -------------------------------------------------------------------
        # Main loop timer
        # -------------------------------------------------------------------
        period = 1.0 / rate
        self.timer = self.create_timer(period, self._loop)
        self.get_logger().info('Serial bridge node started')

    # =======================================================================
    # Subscriber callbacks
    # =======================================================================

    def _cb_cmd_vel(self, msg: Twist):
        with self._tx_lock:
            self._cmd_vx = msg.linear.x
            self._cmd_vy = msg.linear.y
            self._cmd_omega = msg.angular.z

    def _cb_beacon_arm(self, msg: Float32):
        with self._tx_lock:
            self._beacon_arm_pos = msg.data

    def _cb_container_arm(self, msg: Float32):
        with self._tx_lock:
            self._container_arm_pos = msg.data

    def _cb_sort_gate(self, msg: Float32):
        with self._tx_lock:
            self._sort_gate_pos = msg.data

    def _cb_start_signal(self, msg: Bool):
        with self._tx_lock:
            self._start_signal_fwd = msg.data

    # =======================================================================
    # Main periodic loop
    # =======================================================================

    def _loop(self):
        self._send_to_roborio()
        self._receive_from_roborio()

    # =======================================================================
    # TX: Jetson → roboRIO
    # =======================================================================

    def _send_to_roborio(self):
        with self._tx_lock:
            flags = 0x01 if self._start_signal_fwd else 0x00

            # Pack all fields EXCEPT CRC (last byte)
            payload = struct.pack(
                '<2sB6fBB',
                JETSON_TO_RIO_MAGIC,
                self._tx_seq & 0xFF,
                float(self._cmd_vx),
                float(self._cmd_vy),
                float(self._cmd_omega),
                float(self._beacon_arm_pos),
                float(self._container_arm_pos),
                float(self._sort_gate_pos),
                flags,
                0,  # reserved
            )

            crc = crc8(payload)
            packet = payload + bytes([crc])
            self._tx_seq = (self._tx_seq + 1) & 0xFF

        try:
            self.ser.write(packet)
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')

    # =======================================================================
    # RX: roboRIO → Jetson
    # =======================================================================

    def _receive_from_roborio(self):
        try:
            # Read available bytes
            avail = self.ser.in_waiting
            if avail > 0:
                data = self.ser.read(min(avail, 512))
                self._rx_buf.extend(data)
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return

        # Scan for valid packets (keep the latest one if multiple)
        latest_packet = None

        while len(self._rx_buf) >= RIO_TO_JETSON_SIZE:
            # Look for magic bytes
            if (self._rx_buf[0] == 0xA5 and self._rx_buf[1] == 0x5A):
                pkt_bytes = bytes(self._rx_buf[:RIO_TO_JETSON_SIZE])

                # Validate CRC
                expected_crc = crc8(pkt_bytes[:-1])
                if pkt_bytes[-1] == expected_crc:
                    latest_packet = pkt_bytes
                    self._rx_buf = self._rx_buf[RIO_TO_JETSON_SIZE:]
                    continue
                else:
                    # CRC mismatch – skip one byte
                    self._rx_buf = self._rx_buf[1:]
            else:
                # Not magic – skip one byte
                self._rx_buf = self._rx_buf[1:]

        # Prevent buffer from growing unbounded
        if len(self._rx_buf) > 1024:
            self._rx_buf = self._rx_buf[-RIO_TO_JETSON_SIZE:]

        if latest_packet is None:
            return

        # Unpack the packet
        fields = struct.unpack(RIO_TO_JETSON_FMT, latest_packet)
        idx = 0
        _magic = fields[idx]; idx += 1
        seq = fields[idx]; idx += 1
        odom_x = fields[idx]; idx += 1
        odom_y = fields[idx]; idx += 1
        odom_theta = fields[idx]; idx += 1
        odom_vx = fields[idx]; idx += 1
        odom_vy = fields[idx]; idx += 1
        odom_vtheta = fields[idx]; idx += 1
        gyro_yaw_rate = fields[idx]; idx += 1
        accel_x = fields[idx]; idx += 1
        accel_y = fields[idx]; idx += 1
        enc_left = fields[idx]; idx += 1
        enc_right = fields[idx]; idx += 1
        enc_horiz = fields[idx]; idx += 1
        hall_event = fields[idx]; idx += 1
        timestamp_ms = fields[idx]; idx += 1
        match_state = fields[idx]; idx += 1
        match_time_ms = fields[idx]; idx += 1
        _crc = fields[idx]; idx += 1

        # Sequence tracking
        if self._rx_count > 0:
            expected = (self._last_rx_seq + 1) & 0xFF
            if seq != expected:
                gap = (seq - expected) & 0xFF
                self._drop_count += gap
        self._last_rx_seq = seq
        self._rx_count += 1

        now = self.get_clock().now().to_msg()

        # -------------------------------------------------------------------
        # Publish /odom/wheel
        # -------------------------------------------------------------------
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = float(odom_x)
        odom_msg.pose.pose.position.y = float(odom_y)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = yaw_to_quaternion(float(odom_theta))

        # Pose covariance: [x, y, z, roll, pitch, yaw] diagonal
        # Moderate uncertainty for wheel odometry
        odom_msg.pose.covariance[0] = 0.01    # x variance (m²)
        odom_msg.pose.covariance[7] = 0.01    # y variance
        odom_msg.pose.covariance[35] = 0.005  # yaw variance (rad²)

        odom_msg.twist.twist.linear.x = float(odom_vx)
        odom_msg.twist.twist.linear.y = float(odom_vy)
        odom_msg.twist.twist.angular.z = float(odom_vtheta)

        odom_msg.twist.covariance[0] = 0.005   # vx variance
        odom_msg.twist.covariance[7] = 0.005   # vy variance
        odom_msg.twist.covariance[35] = 0.003  # omega variance

        self.pub_odom.publish(odom_msg)

        # -------------------------------------------------------------------
        # Publish /imu/data
        # -------------------------------------------------------------------
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'base_link'

        # We only have yaw rate from MPU-6050 (no orientation from IMU alone)
        imu_msg.orientation_covariance[0] = -1.0  # orientation unknown

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = float(gyro_yaw_rate)
        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[4] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.001  # yaw rate most reliable

        imu_msg.linear_acceleration.x = float(accel_x)
        imu_msg.linear_acceleration.y = float(accel_y)
        imu_msg.linear_acceleration.z = 9.81  # assume flat surface
        imu_msg.linear_acceleration_covariance[0] = 0.05
        imu_msg.linear_acceleration_covariance[4] = 0.05
        imu_msg.linear_acceleration_covariance[8] = 0.1

        self.pub_imu.publish(imu_msg)

        # -------------------------------------------------------------------
        # Publish /robot/hall_event (only on positive edge)
        # -------------------------------------------------------------------
        if hall_event:
            hall_msg = Bool()
            hall_msg.data = True
            self.pub_hall.publish(hall_msg)

        # -------------------------------------------------------------------
        # Publish /match/state_rio (string name of current state)
        # -------------------------------------------------------------------
        state_msg = String()
        if match_state < len(self._state_names):
            state_msg.data = self._state_names[match_state]
        else:
            state_msg.data = f'UNKNOWN_{match_state}'
        self.pub_match_state.publish(state_msg)

        # -------------------------------------------------------------------
        # Publish /match/time_remaining (seconds)
        # -------------------------------------------------------------------
        time_msg = Float32()
        time_msg.data = float(match_time_ms) / 1000.0
        self.pub_match_time.publish(time_msg)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
