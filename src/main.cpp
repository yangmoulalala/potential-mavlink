#include "mavlink.hpp"

// =============================================================================
// MAVLink 串口解析（独立于 ROS2 spin，运行在 main 线程）
// =============================================================================

/**
 * @brief 解析一帧 IMU MAVLink 消息并更新节点的 imu_data。
 */
static void handle_imu_message(MavLink& node, const mavlink_message_t& msg)
{
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

static void handle_target_pose_message(MavLink& node, const mavlink_message_t& msg)
{
    mavlink_target_pose_t position;
    mavlink_msg_target_pose_decode(&msg, &position);

    node.target_point.x = position.pos_x;
    node.target_point.y = position.pos_y;
    // std::cout << "pos.x " << position.pos_x
    //           << "pos.y " << position.pos_y << '\n';
}


/**
 * @brief 消费串口缓冲区，逐字节送入 MAVLink 解析器。
 */
// static void process_serial(MavLink& node)
// {
//     while (node.ros_ser.available() > 0) {
//         std::string data = node.ros_ser.read(node.ros_ser.available());

//         static mavlink_message_t mav_msg;
//         static mavlink_status_t mav_status;

//         for (char c : data) {
//             if (!mavlink_parse_char(MAVLINK_COMM_0,
//                                     static_cast<uint8_t>(c),
//                                     &mav_msg, &mav_status)) {
//                 continue;
//             }

//             switch (mav_msg.msgid) {
//                 case MAVLINK_MSG_ID_imu:
//                     handle_imu_message(node, mav_msg);
//                     break;
//                 case MAVLINK_MSG_ID_referee:
//                     handle_referee_message(node, mav_msg);
//                     break;
//                 case MAVLINK_MSG_ID_target_pose:
//                     handle_target_pose_message(node, mav_msg);
//                     break;
//                 default:
//                     RCLCPP_DEBUG(node.get_logger(),
//                                 "Unknown MAVLink msg id: %d", mav_msg.msgid);
//                     break;
//             }
//         }
//     }
// }
static void process_serial(MavLink& node)
{
    // 将缓冲区改大，例如 512 或 1024 字节，确保一次能吞下多个包
    uint8_t buf[512]; 
    static mavlink_message_t mav_msg;
    static mavlink_status_t mav_status;

    // 使用 while 循环尽可能排空串口内核缓冲区
    while (node.ros_ser.available() > 0) {
        size_t n = node.ros_ser.read(buf, sizeof(buf));

        //         // 使用cout输出收到的数据
        // std::cout << "收到 " << n << " 字节: ";
        // for (size_t i = 0; i < n; i++) {
        //     // 使用hex和setw来格式化十六进制输出
        //     std::cout << std::hex << std::setw(2) << std::setfill('0') 
        //              << static_cast<int>(buf[i]) << " ";
        // }
        // std::cout << std::dec << std::endl; // 恢复十进制输出

        for (size_t i = 0; i < n; i++) {
            // mavlink_parse_char 是状态机，它能自动处理连续拼在一起的包
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mav_msg, &mav_status)) {
                switch (mav_msg.msgid) {
                    case MAVLINK_MSG_ID_imu:
                        handle_imu_message(node, mav_msg);
                        break;
                    case MAVLINK_MSG_ID_referee:
                        handle_referee_message(node, mav_msg);
                        break;
                    case MAVLINK_MSG_ID_target_pose:
                        handle_target_pose_message(node, mav_msg);
                        break;
                }
            }
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
            process_serial(*node);
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(node->get_logger(), "Serial read failed: %s", e.what());
            node->ros_ser.close();
            node->serial_is_init = false;
        } catch (...) {
            RCLCPP_ERROR(node->get_logger(), "Unexpected serial error.");
            node->ros_ser.close();
            node->serial_is_init = false;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    rclcpp::shutdown();
    return 0;
}
