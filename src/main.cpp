#include "mavlink.hpp"

// =============================================================================
// MAVLink 串口解析（独立于 ROS2 spin，运行在 main 线程）
// =============================================================================

/**
 * @brief 解析一帧 IMU MAVLink 消息并更新节点的 imu_data。
 */
static void handle_imu_message(MavLink& node, const mavlink_message_t& msg,
                                int& print_count)
{
    mavlink_imu_t imu;
    mavlink_msg_imu_decode(&msg, &imu);

    // 角度转弧度，并修正坐标系（pitch 取反）
    const double roll  =  imu.roll  / 180.0 * M_PI;
    const double pitch = -imu.pitch / 180.0 * M_PI;
    const double yaw   =  imu.yaw   / 180.0 * M_PI;

    if (++print_count > constants::PRINT_INTERVAL) {
        std::cout << "[IMU] roll=" << roll
                  << " pitch=" << pitch
                  << " yaw=" << yaw << '\n';
        print_count = 0;
    }

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    node.imu_data.orientation.x = q.getX();
    node.imu_data.orientation.y = q.getY();
    node.imu_data.orientation.z = q.getZ();
    node.imu_data.orientation.w = q.getW();
}

/**
 * @brief 解析一帧裁判系统 MAVLink 消息并更新节点的颜色/弹速请求。
 */
static void handle_referee_message(MavLink& node, const mavlink_message_t& msg)
{
    mavlink_referee_t ref;
    mavlink_msg_referee_decode(&msg, &ref);

    // is_red == 0 表示我方为蓝色，目标颜色为红色
    node.team_color_request = (ref.is_red == 0)
                              ? constants::TEAM_RED
                              : constants::TEAM_BLUE;

    if (ref.bullet_speed >= constants::MIN_BULLET_SPD &&
        ref.bullet_speed <  constants::MAX_BULLET_SPD)
    {
        node.bullet_speed_request = ref.bullet_speed;
    }
}

/**
 * @brief 消费串口缓冲区，逐字节送入 MAVLink 解析器。
 */
static void process_serial(MavLink& node, int& print_count)
{
    const size_t available = node.ros_ser.available();
    if (available == 0) return;

    const std::string data = node.ros_ser.read(available);

    mavlink_message_t mav_msg;
    mavlink_status_t  mav_status;

    for (char c : data) {
        if (!mavlink_parse_char(MAVLINK_COMM_0,
                                static_cast<uint8_t>(c),
                                &mav_msg, &mav_status)) {
            continue;
        }

        switch (mav_msg.msgid) {
            case MAVLINK_MSG_ID_imu:
                handle_imu_message(node, mav_msg, print_count);
                break;
            case MAVLINK_MSG_ID_referee:
                handle_referee_message(node, mav_msg);
                break;
            
            default:
                RCLCPP_DEBUG(node.get_logger(),
                             "Unknown MAVLink msg id: %d", mav_msg.msgid);
                break;
        }
    }
}

// =============================================================================
// main
// =============================================================================
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MavLink>();

    int  print_count     = 0;
    auto last_retry_time = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // ── 串口未就绪：每秒重试一次 ─────────────────────────────────────────
        if (!node->serial_is_init) {
            const auto now = std::chrono::steady_clock::now();
            if (now - last_retry_time > std::chrono::seconds(1)) {
                node->serial_init();
                last_retry_time = now;
            }
            continue;
        }

        // ── 串口读取与 MAVLink 解析 ───────────────────────────────────────────
        try {
            process_serial(*node, print_count);
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(node->get_logger(), "Serial read failed: %s", e.what());
            node->ros_ser.close();
            node->serial_is_init = false;
        } catch (...) {
            RCLCPP_ERROR(node->get_logger(), "Unexpected serial error.");
            node->ros_ser.close();
            node->serial_is_init = false;
        }
    }

    rclcpp::shutdown();
    return 0;
}