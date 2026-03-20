#include "mavlink.hpp"

using std::placeholders::_1;
// =============================================================================
// 构造函数
// =============================================================================
MavLink::MavLink() : Node("MavLink")
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // QoS 设置
    auto sensor_qos = rclcpp::SensorDataQoS();

    // ── 串口初始化 ────────────────────────────────────────────────────────────
    serial_init();

    // ── 订阅 ─────────────────────────────────────────────────────────────────
    gimbal_sub_ = this->create_subscription<rm_interfaces::msg::Cboard>(
        "/vision/auto_aim",rclcpp::SensorDataQoS(),std::bind(&MavLink::gimbal_callback, this, _1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/sentry/cmd_vel", 10, std::bind(&MavLink::cmd_vel_callback, this, _1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/sentry/odometry", 10,std::bind(&MavLink::odom_callback, this, std::placeholders::_1));
    insta360_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/insta360", 10, std::bind(&MavLink::insta360_callback, this, std::placeholders::_1));
    
    // ── 发布 ─────────────────────────────────────────────────────────────────
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/mavlink/serial_driver/imu_raw", rclcpp::SensorDataQoS());

    referee_pub_ = this->create_publisher<rm_interfaces::msg::Referee>(
        "/mavlink/referee", 10);
    
    target_position_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
        "/mavlink/target_position", 10);

    insta360_color_slot_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "/mavlink/insta360/color_slot", 10);

    timer_ = create_wall_timer(
            std::chrono::milliseconds(constants::TIMER_PERIOD_MS),
            std::bind(&MavLink::timer_callback, this));
}
MavLink::~MavLink() {
    std::lock_guard<std::mutex> lock(serial_mtx_);
    handle_serial_disconnect_locked("node shutdown");
}

// =============================================================================
// 串口管理
// =============================================================================
void MavLink::handle_serial_disconnect_locked(const std::string& reason) {
    if (ros_ser_.isOpen()) {
        try {
            ros_ser_.close();
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Serial close error while handling [%s]: %s", reason.c_str(), e.what());
        }
    }

    if (serial_is_init_.exchange(false)) {
        RCLCPP_WARN(get_logger(), "Serial disconnected: %s", reason.c_str());
    }
}

void MavLink::serial_init() {
    std::lock_guard<std::mutex> lock(serial_mtx_);
    serial_is_init_ = false;

    try {
        if (ros_ser_.isOpen()) {
            ros_ser_.close();
        }

        ros_ser_.setPort("/dev/c_board");
        ros_ser_.setBaudrate(115200);
        auto timeout = serial::Timeout::simpleTimeout(20);
        ros_ser_.setTimeout(timeout);
        ros_ser_.open();

        serial_is_init_ = ros_ser_.isOpen();
        if (serial_is_init_) {
            RCLCPP_INFO(get_logger(), "Serial connected to /dev/c_board");
        }
    } catch (const std::exception& e) {
        handle_serial_disconnect_locked(e.what());
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Serial init failed: %s", e.what());
    }
}


// =============================================================================
// 订阅回调
// =============================================================================
void MavLink::gimbal_callback(const rm_interfaces::msg::Cboard::SharedPtr msg)
{
    auto cmd = *msg;
    
    // 逻辑处理：目标丢失时保持
    if (!cmd.is_detected) {
        cmd.yaw = last_state_.yaw;
        cmd.pitch = last_state_.pitch;
        cmd.wr = last_state_.wr;
        cmd.robot_id = last_state_.robot_id;
        cmd.distance = last_state_.distance;
    } else {
        last_state_.yaw = cmd.yaw;
        last_state_.pitch = cmd.pitch;
        last_state_.wr = cmd.wr;
        last_state_.robot_id = cmd.robot_id;
        last_state_.distance = cmd.distance;
    }

    mavlink_message_t mav_msg;
    mavlink_msg_auto_aim_pack(1, 200, &mav_msg, 
        cmd.is_detected, cmd.yaw, cmd.pitch, cmd.is_fire, 
        cmd.robot_id, cmd.wr, cmd.distance);
    
    send_mavlink(mav_msg);
}

void MavLink::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    auto cmd = *msg;

    mavlink_message_t mav_msg;
    mavlink_msg_chassis_speed_pack(1, 200, &mav_msg,
                                   cmd.linear.x, 
                                   cmd.linear.y);

    send_mavlink(mav_msg);
}

void MavLink::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    auto cmd = *msg;

    //四元数获取yaw
    tf2::Quaternion q(
    cmd.pose.pose.orientation.x,
    cmd.pose.pose.orientation.y,
    cmd.pose.pose.orientation.z,
    cmd.pose.pose.orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    mavlink_message_t mav_msg;
    mavlink_msg_current_position_pack(1, 200, &mav_msg,
                                      cmd.pose.pose.position.x,
                                      cmd.pose.pose.position.y,
                                      yaw * 180.0 / M_PI);

    send_mavlink(mav_msg);
}

void MavLink::insta360_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // 验证数据长度
    if (msg->data.size() < 6) {
        RCLCPP_WARN(get_logger(), "Insta360 message data size insufficient: %zu, expected 6", msg->data.size());
        return;
    }
    
    // 提取数据
    float a0 = msg->data[0];
    float c0 = msg->data[1];
    float a1 = msg->data[2];
    float c1 = msg->data[3];
    float a2 = msg->data[4];
    float c2 = msg->data[5];
    
    // 打包MAVLink消息并发送
    mavlink_message_t mav_msg;
    mavlink_msg_insta360_pack(1, 200, &mav_msg, a0, c0, a1, c1, a2, c2);
    
    send_mavlink(mav_msg);
}

// =============================================================================
// 定时器主回调（10 Hz）
// =============================================================================

void MavLink::timer_callback()
{
    publish_target_position();
}


// =============================================================================
// 业务逻辑
// =============================================================================

/**
 * @brief 打包 MAVLink 帧并通过串口发送云台控制指令。
 */

void MavLink::send_mavlink(const mavlink_message_t& msg) {
    if (!serial_is_init_) return;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    std::lock_guard<std::mutex> lock(serial_mtx_);
    if (!ros_ser_.isOpen()) {
        handle_serial_disconnect_locked("port not open before write");
        return;
    }

    try {
        const size_t written = ros_ser_.write(buf, len);
        RCLCPP_INFO(get_logger(), "Serial write %zu/%u bytes", written, len);
        if (written != len) {
            handle_serial_disconnect_locked("short write");
        }
    } catch (const std::exception& e) {
        handle_serial_disconnect_locked(e.what());
        RCLCPP_ERROR(get_logger(), "Serial write error: %s", e.what());
    }
}

// =============================================================================
// 发布接口
// =============================================================================

void MavLink::publish_imu()
{
    imu_data_.header.stamp    = this->now();
    imu_data_.header.frame_id = "imu_link";
    imu_pub_->publish(imu_data_);
}

void MavLink::publish_tf()
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id  = "gimbal_link";

    t.transform.rotation.x = imu_data_.orientation.x;
    t.transform.rotation.y = imu_data_.orientation.y;
    t.transform.rotation.z = imu_data_.orientation.z;
    t.transform.rotation.w = imu_data_.orientation.w;

    tf_broadcaster_->sendTransform(t);
}

void MavLink::publish_referee()
{
    referee_.header.stamp = this->now();
    referee_pub_->publish(referee_);
}

void MavLink::publish_target_position()
{
    target_position_pub_->publish(target_point_);
}


void MavLink::parse_mavlink_msg(const mavlink_message_t& msg)
{ 
    switch (msg.msgid) {

        case MAVLINK_MSG_ID_imu: {
                mavlink_imu_t imu;
                mavlink_msg_imu_decode(&msg, &imu);

                // 角度转弧度，并修正坐标系（pitch 取反）
                const double roll  =  imu.roll  / 180.0 * M_PI;
                const double pitch = -imu.pitch / 180.0 * M_PI;
                const double yaw   =  imu.yaw   / 180.0 * M_PI;

                // std::cout << "[IMU] roll=" << roll
                //             << " pitch=" << pitch
                //             << " yaw=" << yaw << '\n';

                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);

                imu_data_.orientation.x = q.getX();
                imu_data_.orientation.y = q.getY();
                imu_data_.orientation.z = q.getZ();
                imu_data_.orientation.w = q.getW();

                publish_imu();
                publish_tf();

                // RCLCPP_INFO(this->get_logger(), "roll=%.3f pitch=%.3f yaw=%.3f", roll, pitch, yaw);
            break;
        }

        case MAVLINK_MSG_ID_referee: {
            mavlink_referee_t ref;
            mavlink_msg_referee_decode(&msg, &ref);

            this->referee_.game_progress =ref.game_progress;
            this->referee_.stage_remain_time = ref.stage_remain_time;
            this->referee_.is_red = ref.is_red;
            this->referee_.bullet_speed = ref.bullet_speed;
    
            publish_referee();

            //向insta360发送消息，topic: 0=红色, 1=蓝色
            std_msgs::msg::Int32 color_msg;
            color_msg.data = static_cast<int32_t>(!ref.is_red);
            insta360_color_slot_pub_->publish(color_msg);

            // RCLCPP_INFO(this->get_logger(), "game_progress=%d stage_remain_time=%d is_red=%d bullet_speed=%.3f", ref.game_progress, ref.stage_remain_time, ref.is_red, ref.bullet_speed);
            break;
        }

        case MAVLINK_MSG_ID_target_position: {
            mavlink_target_position_t position;
            mavlink_msg_target_position_decode(&msg, &position);
            this->target_point_.x = position.x;
            this->target_point_.y = position.y;

            // RCLCPP_INFO(this->get_logger(), "target_position x=%.3f y=%.3f", position.x, position.y);
            break;
        }

        default:

            break;
    }
}
// 串口接收逻辑（由 main 线程调用）
void MavLink::receive_and_parse() {
    if (!serial_is_init_) return;

    std::vector<uint8_t> buf;
    {
        std::lock_guard<std::mutex> lock(serial_mtx_);
        if (!ros_ser_.isOpen()) {
            handle_serial_disconnect_locked("port not open before read");
            return;
        }

        try {
            const size_t n = ros_ser_.available();
            if (n > 0) {
                ros_ser_.read(buf, n);
            }
        } catch (const std::exception& e) {
            handle_serial_disconnect_locked(e.what());
            RCLCPP_ERROR(get_logger(), "Serial read error: %s", e.what());
            return;
        }
    }

    if (!buf.empty()) {
        mavlink_message_t msg;
        mavlink_status_t status{};
        for (uint8_t c : buf) {
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
                parse_mavlink_msg(msg);
            }
        }
    }
}