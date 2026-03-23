import math
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mavlink import MAVLink

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int32
from rm_interfaces.msg import Cboard, Referee

from tf2_ros import TransformBroadcaster
import tf_transformations


class MavLinkNode(Node):
    PORT = "/dev/c_board"
    BAUDRATE = 115200
    TIMEOUT = 0.02
    RECONNECT_INTERVAL = 1.0
    READ_SLEEP = 0.001

    MAVLINK_MSG_ID_IMU = 0
    MAVLINK_MSG_ID_REFEREE = 1
    MAVLINK_MSG_ID_TARGET_POSITION = 4

    def __init__(self):
        super().__init__("mavlink_node")

        self.serial_port = None
        self.mav = None
        self.connected = False
        self.lock = threading.Lock()
        self.running = True

        self.tx_count = 0
        self.rx_count = 0
        self.last_stats_time = self.get_clock().now()

        self.last_aim_state = {
            "yaw": 0.0,
            "pitch": 0.0,
            "wr": 0.0,
            "robot_id": 0,
            "distance": 0.0,
        }

        self.tf_broadcaster = TransformBroadcaster(self)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(Cboard, "/vision/auto_aim", self.gimbal_callback, sensor_qos)
        self.create_subscription(Twist, "/sentry/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, "/sentry/odometry", self.odom_callback, 10)
        self.create_subscription(Float32MultiArray, "/insta360/top3_angles", self.insta360_callback, 10)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, "/mavlink/serial_driver/imu_raw", sensor_qos)
        self.referee_pub = self.create_publisher(Referee, "/mavlink/referee", 10)
        self.target_position_pub = self.create_publisher(Point, "/mavlink/target_position", 10)
        self.insta360_color_slot_pub = self.create_publisher(Int32, "/mavlink/insta360/color_slot", 10)

        # Timer
        self.create_timer(1.0, self.stats_callback)

        # 串口线程
        self.io_thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.io_thread.start()

    # =========================
    # 串口管理
    # =========================
    def connect_serial(self):
        with self.lock:
            try:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()

                self.serial_port = serial.Serial(
                    port=self.PORT,
                    baudrate=self.BAUDRATE,
                    timeout=self.TIMEOUT,
                )
                self.mav = MAVLink(self.serial_port, srcSystem=1, srcComponent=200)
                self.connected = True
                self.get_logger().info(f"Serial connected: {self.PORT}")
                return True
            except Exception as e:
                self.connected = False
                self.serial_port = None
                self.mav = None
                self.get_logger().warn(f"Serial connect failed: {e}")
                return False

    def disconnect_serial(self, reason="unknown"):
        with self.lock:
            if self.connected:
                self.get_logger().warn(f"Serial disconnected: {reason}")
            self.connected = False
            try:
                if self.serial_port:
                    self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
            self.mav = None

    def serial_loop(self):
        while rclpy.ok() and self.running:
            if not self.connected:
                self.connect_serial()
                if not self.connected:
                    time.sleep(self.RECONNECT_INTERVAL)
                    continue

            try:
                self.receive_and_parse()
            except Exception as e:
                self.disconnect_serial(f"read loop error: {e}")
                time.sleep(self.RECONNECT_INTERVAL)

            time.sleep(self.READ_SLEEP)

    # =========================
    # MAVLink 发送
    # =========================
    def send_mavlink(self, msg_obj):
        with self.lock:
            if not self.connected or self.serial_port is None:
                return
            try:
                self.serial_port.write(msg_obj.pack(self.mav))
                self.tx_count += 1
            except Exception as e:
                self.disconnect_serial(f"write error: {e}")

    # =========================
    # ROS 回调
    # =========================
    def gimbal_callback(self, msg: Cboard):
        if msg.is_detected:
            self.last_aim_state.update(
                yaw=msg.yaw,
                pitch=msg.pitch,
                wr=msg.wr,
                robot_id=msg.robot_id,
                distance=msg.distance,
            )
            yaw = msg.yaw
            pitch = msg.pitch
            wr = msg.wr
            robot_id = msg.robot_id
            distance = msg.distance
        else:
            yaw = self.last_aim_state["yaw"]
            pitch = self.last_aim_state["pitch"]
            wr = self.last_aim_state["wr"]
            robot_id = self.last_aim_state["robot_id"]
            distance = self.last_aim_state["distance"]

        if self.mav is None:
            return

        mav_msg = self.mav.auto_aim_encode(
            msg.is_detected,
            yaw,
            pitch,
            msg.is_fire,
            robot_id,
            wr,
            distance,
        )
        self.send_mavlink(mav_msg)

    def cmd_vel_callback(self, msg: Twist):
        if self.mav is None:
            return
        mav_msg = self.mav.chassis_speed_encode(msg.linear.x, msg.linear.y)
        self.send_mavlink(mav_msg)

    def odom_callback(self, msg: Odometry):
        if self.mav is None:
            return

        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw_deg = math.degrees(yaw)

        mav_msg = self.mav.current_position_encode(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_deg,
        )
        self.send_mavlink(mav_msg)

    def insta360_callback(self, msg: Float32MultiArray):
        if self.mav is None or len(msg.data) < 6:
            return
        mav_msg = self.mav.insta360_encode(*msg.data[:6])
        self.send_mavlink(mav_msg)

    # =========================
    # MAVLink 接收与解析
    # =========================
    def receive_and_parse(self):
        if not self.connected or self.serial_port is None or self.mav is None:
            return

        waiting = self.serial_port.in_waiting
        if waiting <= 0:
            return

        data = self.serial_port.read(waiting)
        for b in data:
            msg = self.mav.parse_char(bytes([b]))
            if msg is not None:
                self.rx_count += 1
                self.handle_parsed_msg(msg)

    def handle_parsed_msg(self, msg):
        msg_id = msg.get_msgId()

        if msg_id == self.MAVLINK_MSG_ID_IMU:
            self.publish_imu(msg)

        elif msg_id == self.MAVLINK_MSG_ID_REFEREE:
            self.publish_referee(msg)

        elif msg_id == self.MAVLINK_MSG_ID_TARGET_POSITION:
            p = Point()
            p.x = msg.x
            p.y = msg.y
            self.target_position_pub.publish(p)

    def publish_imu(self, msg):
        roll = math.radians(msg.roll)
        pitch = math.radians(-msg.pitch)
        yaw = math.radians(msg.yaw)

        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        self.imu_pub.publish(imu_msg)

        t = TransformStamped()
        t.header.stamp = imu_msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "gimbal_link"
        t.transform.rotation = imu_msg.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_referee(self, msg):
        ref_msg = Referee()
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.game_progress = msg.game_progress
        ref_msg.stage_remain_time = float(msg.stage_remain_time)
        ref_msg.is_red = msg.is_red
        ref_msg.bullet_speed = msg.bullet_speed
        self.referee_pub.publish(ref_msg)

        color_msg = Int32()
        color_msg.data = 1 if msg.is_red == 0 else 0
        self.insta360_color_slot_pub.publish(color_msg)

    # =========================
    # 统计
    # =========================
    def stats_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_stats_time).nanoseconds / 1e9
        if dt <= 0:
            return

        rx_hz = self.rx_count / dt
        tx_hz = self.tx_count / dt
        self.get_logger().info(
            f"Serial[{ 'OK' if self.connected else 'DOWN' }] RX: {rx_hz:.2f} Hz, TX: {tx_hz:.2f} Hz"
        )

        self.rx_count = 0
        self.tx_count = 0
        self.last_stats_time = now

    # =========================
    # 退出清理
    # =========================
    def destroy_node(self):
        self.running = False
        self.disconnect_serial("shutdown")
        super().destroy_node()


def main():
    rclpy.init()
    node = MavLinkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
