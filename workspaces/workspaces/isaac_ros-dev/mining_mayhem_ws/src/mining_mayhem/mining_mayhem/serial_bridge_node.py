#!/usr/bin/env python3
# ===========================================================================
# serial_bridge_node.py
# Jetson-side UDP bridge to roboRIO.
# Transport: UDP over Ethernet
#   Jetson IP:   10.0.67.5
#   roboRIO IP:  10.0.67.2
#   RX port:     5800  (Jetson receives sensor data FROM roboRIO)
#   TX port:     5801  (Jetson sends commands TO roboRIO)
# ===========================================================================

import socket
import struct
import math
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String

# ---------------------------------------------------------------------------
# Network config
# ---------------------------------------------------------------------------
ROBORIO_IP = '10.0.67.2'
RX_PORT    = 5800
TX_PORT    = 5801

# ---------------------------------------------------------------------------
# Packet definitions (must match SerialBridgeSubsystem.h exactly)
# ---------------------------------------------------------------------------
RIO_TO_JETSON_FMT  = '<2sB6f3f3iBI BH B'
RIO_TO_JETSON_SIZE = struct.calcsize(RIO_TO_JETSON_FMT)
RIO_TO_JETSON_MAGIC = b'\xA5\x5A'

JETSON_TO_RIO_FMT  = '<2sB6fBBB'
JETSON_TO_RIO_SIZE = struct.calcsize(JETSON_TO_RIO_FMT)
JETSON_TO_RIO_MAGIC = b'\x5A\xA5'

assert RIO_TO_JETSON_SIZE == 60, f"RIO→Jetson size mismatch: {RIO_TO_JETSON_SIZE}"
assert JETSON_TO_RIO_SIZE == 30, f"Jetson→RIO size mismatch: {JETSON_TO_RIO_SIZE}"


def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('loop_rate_hz', 50.0)
        rate = self.get_parameter('loop_rate_hz').value

        # UDP sockets
        try:
            self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._rx_sock.bind(('0.0.0.0', RX_PORT))
            self._rx_sock.setblocking(False)

            self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._roborio_addr = (ROBORIO_IP, TX_PORT)

            self.get_logger().info(
                f'UDP sockets ready — RX on :{RX_PORT}, TX to {ROBORIO_IP}:{TX_PORT}')
        except Exception as e:
            self.get_logger().error(f'Failed to create UDP sockets: {e}')
            raise

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

        # Publishers
        self.pub_odom        = self.create_publisher(Odometry, '/odom/wheel',           sensor_qos)
        self.pub_imu         = self.create_publisher(Imu,      '/imu/data',             sensor_qos)
        self.pub_hall        = self.create_publisher(Bool,     '/robot/hall_event',      sensor_qos)
        self.pub_match_state = self.create_publisher(String,   '/match/state_rio',       sensor_qos)
        self.pub_match_time  = self.create_publisher(Float32,  '/match/time_remaining',  sensor_qos)

        # Subscribers
        self.sub_cmd_vel = self.create_subscription(
            Twist,   '/cmd_vel',               self._cb_cmd_vel,       sensor_qos)
        self.sub_beacon_arm = self.create_subscription(
            Float32, '/actuator/beacon_arm',   self._cb_beacon_arm,    sensor_qos)
        self.sub_container_arm = self.create_subscription(
            Float32, '/actuator/container_arm', self._cb_container_arm, sensor_qos)
        self.sub_sort_gate = self.create_subscription(
            Float32, '/actuator/sort_gate',    self._cb_sort_gate,     sensor_qos)
        self.sub_start_signal = self.create_subscription(
            Bool,    '/match/start_signal',    self._cb_start_signal,  reliable_qos)
        # Software enable — allows motors to run without DS (testing only)
        self.sub_sw_enable = self.create_subscription(
            Bool,    '/bridge/software_enable', self._cb_sw_enable,    reliable_qos)

        # TX state
        self._tx_lock            = Lock()
        self._cmd_vx             = 0.0
        self._cmd_vy             = 0.0
        self._cmd_omega          = 0.0
        self._beacon_arm_pos     = 0.0
        self._container_arm_pos  = 0.0
        self._sort_gate_pos      = 0.0
        self._start_signal_fwd   = False
        self._software_enable    = False
        self._tx_seq             = 0

        # RX state
        self._rx_buf      = bytearray()
        self._last_rx_seq = 0
        self._rx_count    = 0
        self._drop_count  = 0

        self._state_names = [
            'INIT', 'WAIT_START', 'DEPART_LANDING', 'READ_TELEMETRY',
            'DEPOSIT_BEACON', 'COLLECT_MATERIALS', 'LOAD_CSCS',
            'DELIVER_CSCS', 'CAVE_ENTRY', 'END_OF_MATCH', 'FAULT',
        ]

        self._timer = self.create_timer(1.0 / rate, self._loop)
        self.get_logger().info('Serial bridge node started (UDP transport)')

    # =======================================================================
    # Callbacks
    # =======================================================================

    def _cb_cmd_vel(self, msg: Twist):
        with self._tx_lock:
            self._cmd_vx    = msg.linear.x
            self._cmd_vy    = msg.linear.y
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

    def _cb_sw_enable(self, msg: Bool):
        with self._tx_lock:
            self._software_enable = msg.data
            self.get_logger().info(
                f'Software enable: {"ON" if msg.data else "OFF"}')

    # =======================================================================
    # Main loop
    # =======================================================================

    def _loop(self):
        self._send_to_roborio()
        self._receive_from_roborio()

    # =======================================================================
    # TX
    # =======================================================================

    def _send_to_roborio(self):
        with self._tx_lock:
            flags = 0x00
            if self._start_signal_fwd:
                flags |= 0x01
            if self._software_enable:
                flags |= 0x02

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
                0,
            )
            crc = crc8(payload)
            packet = payload + bytes([crc])
            self._tx_seq = (self._tx_seq + 1) & 0xFF

        try:
            self._tx_sock.sendto(packet, self._roborio_addr)
        except OSError:
            pass

    # =======================================================================
    # RX
    # =======================================================================

    def _receive_from_roborio(self):
        latest = None
        try:
            while True:
                data, _ = self._rx_sock.recvfrom(4096)
                latest = data
        except BlockingIOError:
            pass

        if latest is None:
            return

        self._rx_buf = bytearray(latest)

        while len(self._rx_buf) >= RIO_TO_JETSON_SIZE:
            if (self._rx_buf[0] == 0xA5 and self._rx_buf[1] == 0x5A):
                pkt_bytes = bytes(self._rx_buf[:RIO_TO_JETSON_SIZE])
                if pkt_bytes[-1] == crc8(pkt_bytes[:-1]):
                    self._process_packet(pkt_bytes)
                    self._rx_buf = self._rx_buf[RIO_TO_JETSON_SIZE:]
                    continue
            self._rx_buf = self._rx_buf[1:]

    def _process_packet(self, pkt_bytes: bytes):
        fields = struct.unpack(RIO_TO_JETSON_FMT, pkt_bytes)
        idx = 0
        _magic        = fields[idx]; idx += 1
        seq           = fields[idx]; idx += 1
        odom_x        = fields[idx]; idx += 1
        odom_y        = fields[idx]; idx += 1
        odom_theta    = fields[idx]; idx += 1
        odom_vx       = fields[idx]; idx += 1
        odom_vy       = fields[idx]; idx += 1
        odom_vtheta   = fields[idx]; idx += 1
        gyro_yaw_rate = fields[idx]; idx += 1
        accel_x       = fields[idx]; idx += 1
        accel_y       = fields[idx]; idx += 1
        enc_left      = fields[idx]; idx += 1
        enc_right     = fields[idx]; idx += 1
        enc_horiz     = fields[idx]; idx += 1
        hall_event    = fields[idx]; idx += 1
        timestamp_ms  = fields[idx]; idx += 1
        match_state   = fields[idx]; idx += 1
        match_time_ms = fields[idx]; idx += 1
        _crc          = fields[idx]; idx += 1

        if self._rx_count > 0:
            expected = (self._last_rx_seq + 1) & 0xFF
            if seq != expected:
                self._drop_count += (seq - expected) & 0xFF
        self._last_rx_seq = seq
        self._rx_count += 1

        now = self.get_clock().now().to_msg()

        # /odom/wheel
        odom_msg = Odometry()
        odom_msg.header.stamp    = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id  = 'base_link'
        odom_msg.pose.pose.position.x  = float(odom_x)
        odom_msg.pose.pose.position.y  = float(odom_y)
        odom_msg.pose.pose.position.z  = 0.0
        odom_msg.pose.pose.orientation = yaw_to_quaternion(float(odom_theta))
        odom_msg.pose.covariance[0]    = 0.01
        odom_msg.pose.covariance[7]    = 0.01
        odom_msg.pose.covariance[35]   = 0.005
        odom_msg.twist.twist.linear.x  = float(odom_vx)
        odom_msg.twist.twist.linear.y  = float(odom_vy)
        odom_msg.twist.twist.angular.z = float(odom_vtheta)
        odom_msg.twist.covariance[0]   = 0.005
        odom_msg.twist.covariance[7]   = 0.005
        odom_msg.twist.covariance[35]  = 0.003
        self.pub_odom.publish(odom_msg)

        # /imu/data
        imu_msg = Imu()
        imu_msg.header.stamp              = now
        imu_msg.header.frame_id           = 'base_link'
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity.z               = float(gyro_yaw_rate)
        imu_msg.angular_velocity_covariance[0]   = 0.01
        imu_msg.angular_velocity_covariance[4]   = 0.01
        imu_msg.angular_velocity_covariance[8]   = 0.001
        imu_msg.linear_acceleration.x            = float(accel_x)
        imu_msg.linear_acceleration.y            = float(accel_y)
        imu_msg.linear_acceleration.z            = 9.81
        imu_msg.linear_acceleration_covariance[0] = 0.05
        imu_msg.linear_acceleration_covariance[4] = 0.05
        imu_msg.linear_acceleration_covariance[8] = 0.1
        self.pub_imu.publish(imu_msg)

        # /robot/hall_event
        if hall_event:
            hall_msg = Bool()
            hall_msg.data = True
            self.pub_hall.publish(hall_msg)

        # /match/state_rio
        state_msg = String()
        state_msg.data = (self._state_names[match_state]
                          if match_state < len(self._state_names)
                          else f'UNKNOWN_{match_state}')
        self.pub_match_state.publish(state_msg)

        # /match/time_remaining
        time_msg = Float32()
        time_msg.data = float(match_time_ms) / 1000.0
        self.pub_match_time.publish(time_msg)

    def destroy_node(self):
        try:
            self._rx_sock.close()
            self._tx_sock.close()
        except Exception:
            pass
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
