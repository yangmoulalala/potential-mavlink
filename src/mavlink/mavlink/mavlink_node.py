import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import serial

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int32
from rm_interfaces.msg import Cboard, Referee

import tf_transformations

import mavlink


class MavLinkNode(Node):
    def __init__(self):
        super().__init__("mavlink_node")

        # QoS
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, "/mavlink/serial_driver/imu_raw", sensor_qos)
        self.referee_pub = self.create_publisher(Referee, "/mavlink/referee", 10)
        self.target_position_pub = self.create_publisher(Point, "/mavlink/target_position", 10)
        self.insta360_color_slot_pub = self.create_publisher(Int32, "/mavlink/insta360/color_slot", 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/nav/goal_pose", 10)

        # Subscribers
        self.create_subscription(Cboard, "/vision/auto_aim", self.gimbal_callback, sensor_qos)
        self.create_subscription(Twist, "/nav/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, "/nav/odometry", self.odom_callback, 10)
        self.create_subscription(Float32MultiArray, "/insta360/top3_angles", self.insta360_callback, 10)


        # MAVLink
        # 发送和接收分离，避免一个对象同时 pack / parse 的线程安全问题
        self.tx_mav = mavlink.MAVLink(None, srcSystem=0, srcComponent=0)
        self.rx_mav = mavlink.MAVLink(None, srcSystem=0, srcComponent=0)
        self.rx_mav.robust_parsing = True

        # Serial
        self.port = "/dev/c_board"
        self.baudrate = 115200
        self.retry_interval = 1.0
        self.read_interval = 0.001

        self.ser = None
        self.ser_lock = threading.Lock()

        # Stats
        self.stats_lock = threading.Lock()
        self.rx_count = 0
        self.tx_count = 0
        self.bad_count = 0
        self.last_stat_time = time.time()

        self.create_timer(1.0, self.print_stats)

        # Background thread
        self.serial_thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.serial_thread.start()

    # ---------------- ROS callbacks ----------------

    def gimbal_callback(self, msg: Cboard):

        mav_msg = self.tx_mav.auto_aim_encode(
            msg.is_detected,
            msg.yaw,
            msg.pitch,
            msg.is_fire,
            msg.robot_id,
            msg.wr,
            msg.distance,
        )
        self.send_mavlink(mav_msg)

    def cmd_vel_callback(self, msg: Twist):
        mav_msg = self.tx_mav.chassis_speed_encode(
            float(msg.linear.x),
            float(msg.linear.y),
        )
        self.send_mavlink(mav_msg)

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw_deg = math.degrees(yaw)

        mav_msg = self.tx_mav.current_position_encode(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(yaw_deg),
        )
        self.send_mavlink(mav_msg)

    def insta360_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 6:
            self.get_logger().warning(
                f"/insta360/top3_angles length={len(msg.data)}, expected>=6"
            )
            return

        mav_msg = self.tx_mav.insta360_encode(
            float(msg.data[0]),
            float(msg.data[1]),
            float(msg.data[2]),
            float(msg.data[3]),
            float(msg.data[4]),
            float(msg.data[5]),
        )
        self.send_mavlink(mav_msg)

    # ---------------- Serial helpers ----------------

    def is_serial_open(self):
        with self.ser_lock:
            return self.ser is not None and self.ser.is_open

    def connect_serial(self):
        self.close_serial()

        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.02,
                write_timeout=0.02,
                exclusive=True,
            )
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            with self.ser_lock:
                self.ser = ser

            self.get_logger().info(f"Serial connected: {self.port}")
            return True

        except Exception as e:
            self.get_logger().error(f"Serial connect failed: {e}")
            return False

    def close_serial(self):
        with self.ser_lock:
            if self.ser is None:
                return

            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception as e:
                self.get_logger().warning(f"Serial close error: {e}")
            finally:
                self.ser = None

    def send_mavlink(self, msg_obj):
        try:
            packet = msg_obj.pack(self.tx_mav)
        except Exception as e:
            self.get_logger().error(f"MAVLink pack error: {e}")
            return

        with self.ser_lock:
            if self.ser is None or not self.ser.is_open:
                return

            try:
                self.ser.write(packet)
            except Exception as e:
                self.get_logger().error(f"Serial write error: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                return

        with self.stats_lock:
            self.tx_count += 1

    def read_and_parse(self):
        with self.ser_lock:
            if self.ser is None or not self.ser.is_open:
                return

            try:
                n = self.ser.in_waiting
                data = self.ser.read(n) if n > 0 else b""
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                return

        if not data:
            return

        try:
            msgs = self.rx_mav.parse_buffer(data)
        except Exception as e:
            self.get_logger().error(f"MAVLink parse error: {e}")
            return

        if not msgs:
            return

        for mav_msg in msgs:
            if mav_msg is None:
                continue

            if mav_msg.get_type() == "BAD_DATA":
                with self.stats_lock:
                    self.bad_count += 1
                continue

            with self.stats_lock:
                self.rx_count += 1

            self.handle_mavlink_msg(mav_msg)

    # ---------------- MAVLink -> ROS ----------------

    def handle_mavlink_msg(self, mav_msg):
        msg_type = mav_msg.get_type()

        if msg_type == "IMU":
            self.publish_imu(mav_msg)

        elif msg_type == "REFEREE":
            self.publish_referee(mav_msg)

        elif msg_type == "TARGET_POSITION":
            p = Point()
            p.x = float(mav_msg.x)
            p.y = float(mav_msg.y)
            p.z = 0.0
            self.target_position_pub.publish(p)
            
            # # Publish goal_pose as PoseStamped
            # goal_pose = PoseStamped()
            # goal_pose.header.stamp = self.get_clock().now().to_msg()
            # goal_pose.header.frame_id = "map"  # Assuming map frame for navigation goals
            # goal_pose.pose.position.x = float(mav_msg.x)
            # goal_pose.pose.position.y = float(mav_msg.y)
            # goal_pose.pose.position.z = 0.0
            # # Orientation is set to identity (no rotation)
            # goal_pose.pose.orientation.x = 0.0
            # goal_pose.pose.orientation.y = 0.0
            # goal_pose.pose.orientation.z = 0.0
            # goal_pose.pose.orientation.w = 1.0
            # self.goal_pose_pub.publish(goal_pose)

    def publish_imu(self, mav_msg):
        roll = math.radians(float(mav_msg.roll))
        pitch = math.radians(-float(mav_msg.pitch))
        yaw = math.radians(float(mav_msg.yaw))

        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        self.imu_pub.publish(imu_msg)


    def publish_referee(self, mav_msg):
        ref_msg = Referee()
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.game_progress = int(mav_msg.game_progress)
        ref_msg.stage_remain_time = float(mav_msg.stage_remain_time)
        ref_msg.is_red = int(mav_msg.is_red)
        ref_msg.bullet_speed = float(mav_msg.bullet_speed)
        self.referee_pub.publish(ref_msg)

        color_msg = Int32()
        color_msg.data = 1 if int(mav_msg.is_red) == 0 else 0
        self.insta360_color_slot_pub.publish(color_msg)

    # ---------------- Stats ----------------

    def print_stats(self):
        now = time.time()

        with self.stats_lock:
            dt = now - self.last_stat_time
            if dt <= 0:
                return

            rx_hz = self.rx_count / dt
            tx_hz = self.tx_count / dt
            bad_count = self.bad_count

            self.rx_count = 0
            self.tx_count = 0
            self.bad_count = 0
            self.last_stat_time = now

        connected = self.is_serial_open()
        self.get_logger().info(
            f"serial={'connected' if connected else 'disconnected'}, "
            f"rx={rx_hz:.2f}Hz, tx={tx_hz:.2f}Hz, bad={bad_count}"
        )

    # ---------------- Thread loop ----------------

    def serial_loop(self):
        while rclpy.ok():
            try:
                if not self.is_serial_open():
                    if not self.connect_serial():
                        time.sleep(self.retry_interval)
                        continue

                self.read_and_parse()
                time.sleep(self.read_interval)

            except Exception as e:
                self.get_logger().error(f"Serial loop error: {e}")
                self.close_serial()
                time.sleep(self.retry_interval)

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MavLinkNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
