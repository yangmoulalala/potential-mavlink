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
    gimbal_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "/vision/gimbal_cmd",
        rclcpp::SensorDataQoS(),
        std::bind(&MavLink::gimbal_callback, this, _1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/red_standard_robot1/cmd_vel", 10, std::bind(&MavLink::cmd_vel_callback, this, _1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/red_standard_robot1/odometry", 10,std::bind(&MavLink::odom_callback, this, std::placeholders::_1));
    
    // ── 发布 ─────────────────────────────────────────────────────────────────
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/serial_dirver/imu_raw", rclcpp::SensorDataQoS());
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/red_standard_robot1/navigate_to_pose");


    // ── 定时器（100 Hz） ──────────────────────────────────────────────────────
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(constants::TIMER_PERIOD_MS),
        std::bind(&MavLink::timer_callback, this));
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

    ros_ser.setPort("/dev/ttyACM0");
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
void MavLink::gimbal_callback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg)
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
void MavLink::timer_callback()
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

    // set_color();
    // set_bullet_speed();

    publish_imu();
    publish_tf();

    publish_nav_goal();
}

// =============================================================================
// 业务逻辑
// =============================================================================

/**
 * @brief 打包 MAVLink 帧并通过串口发送云台控制指令。
 */
void MavLink::send_gimbal_cmd()
{
    uint8_t is_detected = 0;
    uint8_t locked      = (gimbal_cmd_.pitch != 0.0f && gimbal_cmd_.yaw != 0.0f) ? 1 : 0;

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

    if (!gimbal_cmd_.control){
        gimbal_cmd_.yaw   = last_cmd_yaw_;
        gimbal_cmd_.pitch = last_cmd_pitch_;
    }
    // RCLCPP_INFO(get_logger(), "gimbal_cmd_yaw=%.3f", gimbal_cmd_.yaw);
    mavlink_message_t msg;
    uint8_t buf[128];

    mavlink_msg_aim_pack(1, 200, &msg,
                         gimbal_cmd_.control,
                         gimbal_cmd_.fire_advice,
                         gimbal_cmd_.yaw,
                         gimbal_cmd_.pitch);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ros_ser.write(buf, len);

}

void MavLink::send_cmd_vel() {
    if (!serial_is_init || !ros_ser.isOpen()) return;

    mavlink_message_t mav_msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_nav_cmd_vel_pack(1, 200, &mav_msg,
                                               cmd_vel_.linear.x, 
                                               cmd_vel_.linear.y);

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
void MavLink::send_odometry(){
    if (!serial_is_init || !ros_ser.isOpen()) return;

    mavlink_message_t mav_msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    //四元数获取yaw
    static tf2::Quaternion q(
    odometry_.pose.pose.orientation.x,
    odometry_.pose.pose.orientation.y,
    odometry_.pose.pose.orientation.z,
    odometry_.pose.pose.orientation.w
    );
    static double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    u_int16_t len = mavlink_msg_odometry_pack(1,200, &mav_msg,
                                            odometry_.pose.pose.position.x,
                                            odometry_.pose.pose.position.y,
                                            yaw);
    
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
    imu_data.header.stamp    = this->now();
    imu_data.header.frame_id = "imu_link";
    imu_pub_->publish(imu_data);
}

void MavLink::publish_tf()
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id  = "gimbal_link";

    t.transform.rotation.x = imu_data.orientation.x;
    t.transform.rotation.y = imu_data.orientation.y;
    t.transform.rotation.z = imu_data.orientation.z;
    t.transform.rotation.w = imu_data.orientation.w;

    tf_broadcaster_->sendTransform(t);
}

void MavLink::publish_nav_goal()
{
    // 1. 频率限制：例如每 500ms 最多发送一次请求，避免高频冲击 Action Server
    auto now = this->now();
    if ((now - last_nav_send_time_).seconds() < 0.5) {
        return;
    }

    // 2. 距离阈值检查：如果目标点位移非常小，则不重复发送
    double dx = target_point.x - last_sent_target_.x;
    double dy = target_point.y - last_sent_target_.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // 如果距离变化小于 0.1 米，则认为目标没变，直接返回
    if (dist < 0.1) {
        return;
    }

    // 3. 检查服务器是否在线
    if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Nav2 Action server not available");
        return;
    }

    // 4. 构建目标消息
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = now;
    goal_msg.pose.pose.position = target_point; // 直接赋值
    goal_msg.pose.pose.orientation.w = 1.0;     // 默认方向

    // 5. 设置回调
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&MavLink::nav_goal_response_callback, this, _1);
    send_goal_options.feedback_callback = 
        std::bind(&MavLink::nav_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    // 可选：添加完成回调来清理 handle
    send_goal_options.result_callback = [this](const auto &) {
        current_goal_handle_ = nullptr;
    };

    // 6. 发送异步目标
    RCLCPP_INFO(get_logger(), "Sending new Nav2 goal: x=%.2f, y=%.2f", target_point.x, target_point.y);
    nav_client_->async_send_goal(goal_msg, send_goal_options);

    // 7. 更新缓存状态
    last_sent_target_ = target_point;
    last_nav_send_time_ = now;
}

// 导航action服务器是否接收到导航目标点
void MavLink::nav_goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& goal_handle) 
{
    if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    } else {
        current_goal_handle_ = goal_handle;
        RCLCPP_INFO(get_logger(), "Goal accepted by server");
    }
}

void MavLink::nav_feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    //导航状态日志
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "剩余距离: %.2f m, 已耗时: %d s", 
        feedback->distance_remaining, 
        feedback->navigation_time.sec); 
}