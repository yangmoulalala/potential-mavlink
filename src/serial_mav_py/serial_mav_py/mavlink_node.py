import math
import threading
import time

import rclpy
import serial
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32

from . import mavlink


class MavLinkNode(Node):
    EMPTY_READ_WARN_THRESHOLD = 20

    def __init__(self):
        super().__init__('mavlink_node')

        self.mav = mavlink.MAVLink(None, srcSystem=1, srcComponent=200)
        self.mav.robust_parsing = True
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.serial_is_init = False
        self.bad_frame_count = 0
        self.empty_read_count = 0
        self.consecutive_empty_reads = 0

        self.last_state = {
            'yaw': 0.0,
            'pitch': 0.0,
            'wr': 0.0,
            'robot_id': 0,
            'distance': 0.0,
        }
        self.target_point = Point()
        self.rx_count = 0
        self.tx_count = 0
        self.last_stats_time = self.get_clock().now()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.gimbal_sub = self.create_subscription(
            Float32MultiArray,
            '/vision/auto_aim',
            self.gimbal_callback,
            qos_profile_sensor_data,
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/sentry/cmd_vel',
            self.cmd_vel_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/sentry/odometry',
            self.odom_callback,
            10,
        )
        self.insta360_sub = self.create_subscription(
            Float32MultiArray,
            '/insta360/top3_angles',
            self.insta360_callback,
            10,
        )

        self.imu_pub = self.create_publisher(
            Imu,
            '/mavlink/serial_driver/imu_raw',
            qos_profile_sensor_data,
        )
        self.referee_pub = self.create_publisher(
            Float32MultiArray,
            '/mavlink/referee',
            10,
        )
        self.target_position_pub = self.create_publisher(
            Point,
            '/mavlink/target_position',
            10,
        )
        self.insta360_color_slot_pub = self.create_publisher(
            Int32,
            '/mavlink/insta360/color_slot',
            10,
        )

        self.target_timer = self.create_timer(0.1, self.publish_target_position)
        self.stats_timer = self.create_timer(1.0, self.stats_callback)

        self.serial_init()
        self.serial_thread = threading.Thread(target=self.serial_thread_loop, daemon=True)
        self.serial_thread.start()

    def reset_parser_state(self):
        self.mav.buf = bytearray()
        self.mav.buf_index = 0
        self.mav.expected_length = mavlink.HEADER_LEN_V1 + 2
        self.mav.have_prefix_error = False

    def serial_init(self):
        with self.serial_lock:
            self._close_serial_locked(log_reason=False)
            self.reset_parser_state()
            self.empty_read_count = 0
            self.consecutive_empty_reads = 0
            try:
                self.serial_port = serial.Serial(
                    port='/dev/c_board',
                    baudrate=115200,
                    timeout=0.02,
                    exclusive=True,
                    write_timeout=0.02,
                    rtscts=False,
                    dsrdtr=False,
                    xonxoff=False,
                )
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                self.mav.file = self.serial_port
                self.serial_is_init = self.serial_port.is_open
                if self.serial_is_init:
                    self.get_logger().info('Serial connected to /dev/c_board')
            except Exception as exc:
                self.serial_is_init = False
                self.serial_port = None
                self.get_logger().error(f'Serial init failed: {exc}')

    def _close_serial_locked(self, log_reason=True, reason='serial disconnected'):
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except Exception as exc:
                self.get_logger().warning(f'Serial close error: {exc}')
        self.serial_port = None
        was_init = self.serial_is_init
        self.serial_is_init = False
        self.consecutive_empty_reads = 0
        self.reset_parser_state()
        if log_reason and was_init:
            self.get_logger().warning(reason)

    def handle_serial_disconnect(self, reason):
        with self.serial_lock:
            self._close_serial_locked(reason=reason)

    def send_mavlink(self, msg_obj):
        if not self.serial_is_init:
            return

        packet = msg_obj.pack(self.mav)
        with self.serial_lock:
            if self.serial_port is None or not self.serial_port.is_open:
                self._close_serial_locked(reason='port not open before write')
                return
            try:
                written = self.serial_port.write(packet)
                self.tx_count += 1
                if written != len(packet):
                    self._close_serial_locked(reason='short write')
            except Exception as exc:
                self._close_serial_locked(reason=str(exc))
                self.get_logger().error(f'Serial write error: {exc}')

    def gimbal_callback(self, msg):
        data = msg.data
        if len(data) < 7:
            self.get_logger().warning(
                f'Auto aim data size insufficient: {len(data)}, expected 7'
            )
            return

        is_detected = int(data[0])
        is_fire = int(data[3])

        if not is_detected:
            yaw = self.last_state['yaw']
            pitch = self.last_state['pitch']
            wr = self.last_state['wr']
            robot_id = self.last_state['robot_id']
            distance = self.last_state['distance']
        else:
            yaw = float(data[1])
            pitch = float(data[2])
            robot_id = int(data[4])
            wr = float(data[5])
            distance = float(data[6])
            self.last_state.update(
                yaw=yaw,
                pitch=pitch,
                wr=wr,
                robot_id=robot_id,
                distance=distance,
            )

        mav_msg = self.mav.auto_aim_encode(
            is_detect=is_detected,
            yaw=yaw,
            pitch=pitch,
            is_fire=is_fire,
            robot_id=robot_id,
            wr=wr,
            distance=distance,
        )
        self.send_mavlink(mav_msg)

    def cmd_vel_callback(self, msg):
        mav_msg = self.mav.chassis_speed_encode(
            vx=float(msg.linear.x),
            vy=float(msg.linear.y),
        )
        self.send_mavlink(mav_msg)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.degrees(self.quaternion_to_yaw(q.x, q.y, q.z, q.w))

        mav_msg = self.mav.current_position_encode(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=float(yaw),
        )
        self.send_mavlink(mav_msg)

    def insta360_callback(self, msg):
        if len(msg.data) < 6:
            self.get_logger().warning(
                f'Insta360 data size insufficient: {len(msg.data)}, expected 6'
            )
            return

        mav_msg = self.mav.insta360_encode(*[float(v) for v in msg.data[:6]])
        self.send_mavlink(mav_msg)

    def handle_empty_read_error(self, exc):
        self.empty_read_count += 1
        self.consecutive_empty_reads += 1

        if self.consecutive_empty_reads == self.EMPTY_READ_WARN_THRESHOLD:
            self.get_logger().warning(
                'Serial empty-read persisted for %d consecutive polls: %s'
                % (self.consecutive_empty_reads, exc)
            )

        try:
            if self.serial_port is not None and self.serial_port.is_open:
                self.serial_port.reset_input_buffer()
        except Exception:
            pass

    def receive_and_parse(self):
        if not self.serial_is_init:
            return

        with self.serial_lock:
            if self.serial_port is None or not self.serial_port.is_open:
                self._close_serial_locked(reason='port not open before read')
                return
            try:
                waiting = self.serial_port.in_waiting
                data = self.serial_port.read(waiting) if waiting > 0 else b''
            except serial.SerialException as exc:
                if 'device reports readiness to read but returned no data' in str(exc):
                    self.handle_empty_read_error(exc)
                    return
                self._close_serial_locked(reason=str(exc))
                self.get_logger().error(f'Serial read error: {exc}')
                return
            except Exception as exc:
                self._close_serial_locked(reason=str(exc))
                self.get_logger().error(f'Serial read error: {exc}')
                return

        if not data:
            return

        self.consecutive_empty_reads = 0
        messages = self.mav.parse_buffer(data)
        if not messages:
            return

        for mav_msg in messages:
            if mav_msg.get_type() == 'BAD_DATA':
                self.bad_frame_count += 1
                continue
            self.rx_count += 1
            self.handle_parsed_msg(mav_msg)

    def handle_parsed_msg(self, mav_msg):
        msg_type = mav_msg.get_type()

        if msg_type == 'IMU':
            roll = math.radians(float(mav_msg.roll))
            pitch = math.radians(-float(mav_msg.pitch))
            yaw = math.radians(float(mav_msg.yaw))
            qx, qy, qz, qw = self.rpy_to_quaternion(roll, pitch, yaw)

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            self.imu_pub.publish(imu_msg)

            transform = TransformStamped()
            transform.header.stamp = imu_msg.header.stamp
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'gimbal_link'
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)
            return

        if msg_type == 'REFEREE':
            referee_msg = Float32MultiArray()
            referee_msg.data = [
                float(mav_msg.game_progress),
                float(mav_msg.stage_remain_time),
                float(mav_msg.is_red),
                float(mav_msg.bullet_speed),
            ]
            self.referee_pub.publish(referee_msg)

            color_msg = Int32()
            color_msg.data = int(not bool(mav_msg.is_red))
            self.insta360_color_slot_pub.publish(color_msg)
            return

        if msg_type == 'TARGET_POSITION':
            self.target_point.x = float(mav_msg.x)
            self.target_point.y = float(mav_msg.y)
            self.target_point.z = 0.0

    def publish_target_position(self):
        self.target_position_pub.publish(self.target_point)

    def stats_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_stats_time).nanoseconds / 1e9
        if elapsed > 0.0:
            rx_frequency = self.rx_count / elapsed
            tx_frequency = self.tx_count / elapsed
            self.get_logger().info(
                f'Serial RX frequency: {rx_frequency:.2f} Hz, TX frequency: {tx_frequency:.2f} Hz, '
                f'bad frames: {self.bad_frame_count}, empty reads: {self.empty_read_count}, '
                f'consecutive empty reads: {self.consecutive_empty_reads}'
            )
        self.rx_count = 0
        self.tx_count = 0
        self.bad_frame_count = 0
        self.empty_read_count = 0
        self.last_stats_time = now

    def serial_thread_loop(self):
        while rclpy.ok():
            if not self.serial_is_init:
                time.sleep(0.2)
                self.serial_init()
                continue
            self.receive_and_parse()
            time.sleep(0.0005)

    def destroy_node(self):
        self.handle_serial_disconnect('node shutdown')
        return super().destroy_node()

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def rpy_to_quaternion(roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = MavLinkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
