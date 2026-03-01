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

    // ── 发布 ─────────────────────────────────────────────────────────────────
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/serial_dirver/imu_raw", rclcpp::SensorDataQoS());
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/red_standard_robot1/navigate_to_pose");

    // ── 服务客户端 ────────────────────────────────────────────────────────────
    set_color_client_ = this->create_client<rm_interfaces::srv::SetMode>(
        "/armor_detector/set_mode");

    // ── 等待识别服务就绪 ──────────────────────────────────────────────────────
    // wait_for_detector_service();

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

// void MavLink::wait_for_detector_service()
// {
//     while (!set_color_client_->wait_for_service(std::chrono::seconds(1))) {
//         if (!rclcpp::ok()) {
//             RCLCPP_ERROR(get_logger(), "Interrupted while waiting for detector service.");
//             return;
//         }
//         RCLCPP_INFO(get_logger(), "Waiting for rm_vision to start...");
//         if (serial_is_init) {
//             ros_ser.read(ros_ser.available());  // 防止串口缓冲区溢出
//         }
//     }
// }

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
    if (!serial_is_init) return;

    mavlink_message_t mav_msg;
    uint8_t buf[128];

    // 将 Twist 消息映射到 MAVLink 字段
    // 假设：linear.x (前向), linear.y (横移)
    mavlink_msg_nav_cmd_vel_pack(1, 200, &mav_msg,
                                 msg->linear.x, 
                                 msg->linear.y);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &mav_msg);
    try {
        ros_ser.write(buf, len);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial write cmd_vel error: %s", e.what());
    }
}

// =============================================================================
// 定时器主回调（100 Hz）
// =============================================================================
void MavLink::timer_callback()
{
    if (serial_is_init) {
        try {
            send_gimbal_cmd();
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "Error in uart process, closing serial.");
            ros_ser.close();
            serial_is_init = false;
        }
    }

    set_color();
    set_bullet_speed();
    // publish_imu();
    // publish_tf();
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

    (void)locked;  // 预留：locked 后续可通过 MAVLink 扩展字段传出
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

void MavLink::send_nav_goal(double x, double y, double theta)
{
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 Action server not available");
        return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;

    // 四元数转换 (简单处理 Z 轴旋转)
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&MavLink::nav_goal_response_callback, this, _1);
    // send_goal_options.feedback_callback = std::bind(&MavLink::nav_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);
    // send_goal_options.result_callback = std::bind(&MavLink::nav_result_callback, this, _1);

    nav_client_->async_send_goal(goal_msg, send_goal_options);
}

// 对应的 Action 回调函数实现 (略，可根据需要打印日志)
void MavLink::nav_goal_response_callback(const GoalHandleNav::SharedPtr& goal_handle) {
    if (!goal_handle) { RCLCPP_ERROR(get_logger(), "Goal rejected"); }
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