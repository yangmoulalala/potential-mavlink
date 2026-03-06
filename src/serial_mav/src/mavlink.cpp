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
    target_sub_ = this->create_subscription<rm_interfaces::msg::Target>(
        "/armor_solver/target",
        rclcpp::SensorDataQoS(),
        std::bind(&MavLink::target_callback, this, _1));

    armors_sub_ = this->create_subscription<rm_interfaces::msg::Armors>(
        "armor_detector/armors",
        rclcpp::SensorDataQoS(),
        std::bind(&MavLink::armors_callback, this, _1));

    gimbal_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "/armor_solver/cmd_gimbal",
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

    // ── 服务客户端 ────────────────────────────────────────────────────────────
    set_color_client_ = this->create_client<rm_interfaces::srv::SetMode>(
        "/armor_detector/set_mode");


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
void MavLink::target_callback(const rm_interfaces::msg::Target::SharedPtr msg)
{
    target_ = *msg;
}

void MavLink::armors_callback(const rm_interfaces::msg::Armors::SharedPtr msg)
{
    armors_ = *msg;
}

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

    set_color();
    set_bullet_speed();

    publish_imu();
    publish_tf();
    publish_nav_goal();
}

// =============================================================================
// 业务逻辑
// =============================================================================

/**
 * @brief 从当前帧的装甲板列表中选出最接近跟踪目标的装甲板。
 */
void MavLink::select_best_armor()
{
    best_armor_.reset();

    if (armors_.armors.empty() || !target_.tracking) {
        return;
    }

    double min_dist = std::numeric_limits<double>::max();
    rm_interfaces::msg::Armor candidate;

    for (const auto& armor : armors_.armors) {
        const double dx = armor.pose.position.x - target_.position.x;
        const double dy = armor.pose.position.y - target_.position.y;
        const double dz = armor.pose.position.z - target_.position.z;
        const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < min_dist) {
            min_dist  = dist;
            candidate = armor;
        }
    }

    best_armor_ = candidate;

    if (best_armor_.has_value()) {
        RCLCPP_INFO(get_logger(), "Best armor z=%.3f", best_armor_->pose.position.z);
    }
}

/**
 * @brief 打包 MAVLink 帧并通过串口发送云台控制指令。
 */
void MavLink::send_gimbal_cmd()
{
    if (!serial_is_init || !ros_ser.isOpen()) return;

    select_best_armor();

    uint8_t is_detected = 0;
    uint8_t locked      = (gimbal_cmd_.pitch != 0.0f && gimbal_cmd_.yaw != 0.0f) ? 1 : 0;

    if (best_armor_.has_value()) {
        is_detected    = 1;
        last_cmd_yaw_  = gimbal_cmd_.yaw;
        last_cmd_pitch_= gimbal_cmd_.pitch;
        RCLCPP_INFO(get_logger(), "Target v_yaw=%.3f", target_.v_yaw);
    } else {
        // 保持上一帧指令，防止丢帧时云台抖动
        gimbal_cmd_.yaw   = last_cmd_yaw_;
        gimbal_cmd_.pitch = last_cmd_pitch_;
    }

    mavlink_message_t msg;
    uint8_t buf[128];

    mavlink_msg_aim_pack(1, 200, &msg,
                         is_detected,
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

/**
 * @brief 若队伍颜色请求与当前值不同，则异步向识别节点发送切换指令。
 *        使用 1 s 限流，避免服务端过载。
 */
void MavLink::set_color()
{
    if (team_color_ == team_color_request) return;
    if (this->now() - last_set_color_time_ < std::chrono::seconds(1)) return;

    auto req = std::make_shared<rm_interfaces::srv::SetMode_Request>();

    if (team_color_request == constants::TEAM_RED) {
        req->mode = constants::TEAM_RED;
    } else if (team_color_request == constants::TEAM_BLUE) {
        req->mode = constants::TEAM_BLUE;
    } else {
        RCLCPP_ERROR(get_logger(), "Invalid team color: %d", team_color_request);
        return;
    }

    RCLCPP_INFO(get_logger(), "Requesting color change to %d", team_color_request);
    set_color_client_->async_send_request(
        req, std::bind(&MavLink::set_color_callback, this, _1));
    last_set_color_time_ = this->now();
}

void MavLink::set_color_callback(
    rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture response)
{
    if (response.get()->message == "0") {
        team_color_ = team_color_request;
        RCLCPP_WARN(get_logger(), "Team color switched to %d", team_color_);
    }
}

/**
 * @brief 若弹速请求值发生变化，则通过参数服务器同步更新弹速。
 */
void MavLink::set_bullet_speed()
{
    if (bullet_speed_ == bullet_speed_request) return;

    const auto new_params = std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("/armor_solver.solver.bullet_speed", bullet_speed_request)
    };

    auto result = this->set_parameters_atomically(new_params);
    if (result.successful) {
        RCLCPP_INFO(get_logger(), "Bullet speed updated: %.1f -> %.1f",
                    bullet_speed_, bullet_speed_request);
        bullet_speed_ = bullet_speed_request;
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

// =============================================================================
// 工具函数
// =============================================================================
std::string MavLink::to_hex_string(const std::string& data)
{
    std::ostringstream ss;
    ss << std::hex << std::uppercase;
    for (unsigned char c : data) {
        ss << std::setw(2) << std::setfill('0') << static_cast<int>(c) << ' ';
    }
    std::string result = ss.str();
    if (!result.empty()) result.pop_back();
    return result;
}