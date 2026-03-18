#include "mavlink.hpp"

using std::placeholders::_1;

// =============================================================================
// 构造函数
// =============================================================================
MavLink::MavLink()
    : Node("MavLink"),
      last_gimbal_time_(this->now()),
      last_set_color_time_(this->now()),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
{
    // ── 串口初始化 ────────────────────────────────────────────────────────────
    serial_init();

    // ── 订阅 ─────────────────────────────────────────────────────────────────
    gimbal_sub_ = this->create_subscription<rm_interfaces::msg::Cboard>(
        "/vision/auto_aim",
        rclcpp::SensorDataQoS(),
        std::bind(&MavLink::gimbal_callback, this, _1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/sentry/cmd_vel", 10, std::bind(&MavLink::cmd_vel_callback, this, _1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/sentry/odometry", 10,std::bind(&MavLink::odom_callback, this, std::placeholders::_1));
    
    // ── 发布 ─────────────────────────────────────────────────────────────────
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/serial_driver/imu_raw", rclcpp::SensorDataQoS());

    referee_pub_ = this->create_publisher<rm_interfaces::msg::Referee>(
        "/referee", 10);
    
    target_position_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
        "/sentry/target_position", 10);


    // ── 定时器（100 Hz） ──────────────────────────────────────────────────────
    timer_100hz_ = this->create_wall_timer(
        std::chrono::milliseconds(constants::TIMER_PERIOD_10MS),
        std::bind(&MavLink::timer_100hz_callback, this));
    timer_10hz_ = this->create_wall_timer(
        std::chrono::milliseconds(constants::TIMER_PERIOD_100MS),
        std::bind(&MavLink::timer_10hz_callback, this));
}

// =============================================================================
// 串口管理
// =============================================================================
void MavLink::serial_init()
{
    if (ros_ser.isOpen()) {
        try { ros_ser.close(); }
        catch (const serial::IOException& e) {
            RCLCPP_ERROR(get_logger(), "Close serial error: %s", e.what());
        }
    }

    ros_ser.setPort("/dev/c_board");
    ros_ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(20);
    ros_ser.setTimeout(to);

    try {
        ros_ser.open();
        serial_is_init = ros_ser.isOpen();
        if (serial_is_init) {
            RCLCPP_INFO(get_logger(), "Serial port opened successfully.");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Open serial error: %s", e.what());
        serial_is_init = false;
    }
}


// =============================================================================
// 订阅回调
// =============================================================================
void MavLink::gimbal_callback(const rm_interfaces::msg::Cboard::SharedPtr msg)
{
    last_gimbal_time_ = this->now();
    gimbal_cmd_       = *msg;
}

void MavLink::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel_ = *msg;
}

void MavLink::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    odometry_ = *msg;
}

// =============================================================================
// 定时器主回调（100 Hz）
// =============================================================================
void MavLink::timer_100hz_callback()
{
    if (serial_is_init) {
        try {
            send_gimbal_cmd();
            send_cmd_vel();
            send_odometry();
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "Error in uart process, closing serial.");
            ros_ser.close();
            serial_is_init = false;
        }
    }
    publish_imu();
    publish_tf();
    publish_referee();
}

void MavLink::timer_10hz_callback()
{
    publish_target_position();
}


// =============================================================================
// 业务逻辑
// =============================================================================

/**
 * @brief 打包 MAVLink 帧并通过串口发送云台控制指令。
 */
void MavLink::send_gimbal_cmd()
{

    // if (best_armor_.has_value()) {
    //     is_detected    = 1;
    //     last_cmd_yaw_  = gimbal_cmd_.yaw;
    //     last_cmd_pitch_= gimbal_cmd_.pitch;
    //     RCLCPP_INFO(get_logger(), "Target v_yaw=%.3f", target_.v_yaw);
    // } else {
    //     // 保持上一帧指令，防止丢帧时云台抖动
    //     gimbal_cmd_.yaw   = last_cmd_yaw_;
    //     gimbal_cmd_.pitch = last_cmd_pitch_;
    // }

    if (!gimbal_cmd_.is_detected){
        gimbal_cmd_.yaw   = last_cmd_yaw_;
        gimbal_cmd_.pitch = last_cmd_pitch_;
        gimbal_cmd_.wr = last_wr_;
        gimbal_cmd_.robot_id = last_robot_id_;
        gimbal_cmd_.distance = last_distance_;
    }
    
    if (gimbal_cmd_.is_detected) {
        last_cmd_yaw_  = gimbal_cmd_.yaw;
        last_cmd_pitch_= gimbal_cmd_.pitch;
        last_is_fire_ = gimbal_cmd_.is_fire;
        last_wr_ = gimbal_cmd_.wr;
        last_robot_id_ = gimbal_cmd_.robot_id;
    }
    // RCLCPP_INFO(get_logger(), "gimbal_cmd_yaw=%.3f", gimbal_cmd_.yaw);
    mavlink_message_t mav_msg;
    uint8_t buf[128];

    mavlink_msg_auto_aim_pack(1, 200, &mav_msg,
                         gimbal_cmd_.is_detected,
                         gimbal_cmd_.yaw,
                         gimbal_cmd_.pitch,
                         gimbal_cmd_.is_fire,
                         gimbal_cmd_.robot_id,
                         gimbal_cmd_.wr,
                         gimbal_cmd_.distance
                        );

    send(mav_msg);
}
void MavLink::send_cmd_vel() {
    if (!serial_is_init || !ros_ser.isOpen()) return;

    mavlink_message_t mav_msg;

    mavlink_msg_chassis_speed_pack(1, 200, &mav_msg,
                                               cmd_vel_.linear.x, 
                                               cmd_vel_.linear.y);

    send(mav_msg);
}
void MavLink::send_odometry(){
    if (!serial_is_init || !ros_ser.isOpen()) return;

    mavlink_message_t mav_msg;


    //四元数获取yaw
    tf2::Quaternion q(
    odometry_.pose.pose.orientation.x,
    odometry_.pose.pose.orientation.y,
    odometry_.pose.pose.orientation.z,
    odometry_.pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    u_int16_t len = mavlink_msg_current_position_pack(1,200, &mav_msg,
                                            odometry_.pose.pose.position.x,
                                            odometry_.pose.pose.position.y,
                                            yaw * 180.0 / M_PI);

    send(mav_msg);
}

void MavLink::send(const mavlink_message_t &mav_msg){

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    try {
        uint16_t len = mavlink_msg_to_send_buffer(buf, &mav_msg); 
        size_t bytes_written = ros_ser.write(buf, len);
        if (bytes_written != len) {
            RCLCPP_WARN(this->get_logger(), "Wait! Send mismatch: req %d, sent %ld", len, bytes_written);
        } 
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Write error: %s", e.what());
        serial_is_init = false;
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
            break;
        }

        case MAVLINK_MSG_ID_referee: {
                mavlink_referee_t ref;
                mavlink_msg_referee_decode(&msg, &ref);

                this->referee_.game_progress =ref.game_progress;
                this->referee_.stage_remain_time = ref.stage_remain_time;
                this->referee_.is_red = ref.is_red;
                this->referee_.bullet_speed = ref.bullet_speed;
                
            break;
        }

        case MAVLINK_MSG_ID_target_position: {
            mavlink_target_position_t position;
            mavlink_msg_target_position_decode(&msg, &position);
            this->target_point_.x = position.x;
            this->target_point_.y = position.y;
            break;
        }

        default:

            break;
    }
}
