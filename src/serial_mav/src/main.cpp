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
}


/**
 * @brief 消费串口缓冲区，逐字节送入 MAVLink 解析器。
 */
static void process_serial(MavLink& node)
{

    static mavlink_message_t mav_msg;
    static mavlink_status_t mav_status;

    int buf_len = node.ros_ser.available();
    if (buf_len <= 0)
        return;

    std::string serial_read = node.ros_ser.read(buf_len);

    for (char c : serial_read) {
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &mav_msg, &mav_status)) {

            switch (mav_msg.msgid) {

                case MAVLINK_MSG_ID_imu: {
                    handle_imu_message(node, mav_msg);
                    break;
                }

                case MAVLINK_MSG_ID_referee: {
                    handle_referee_message(node, mav_msg);
                    break;
                }

                case MAVLINK_MSG_ID_target_pose: {
                    handle_target_pose_message(node, mav_msg);
                    break;
                }

                default:
                    // 可加调试信息
                    break;
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

        // ── 串口未就绪：1 秒重试 ─────────────────────────────────────────────
        if (!node->serial_is_init) {
            auto now = std::chrono::steady_clock::now();
            if (now - last_retry_time > std::chrono::seconds(1)) {
                node->serial_init();
                last_retry_time = now;
            }
            continue;
        }

        // ── 串口读取与 MAVLink 解析 ───────────────────────────────────────────
        try {
            if (node->serial_is_init) {
                process_serial(*node);
            }

        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(node->get_logger(), "Serial read failed: %s", e.what());
            node->ros_ser.close();
            node->serial_is_init = false;
        } catch (...) {
            RCLCPP_ERROR(node->get_logger(), "Unexpected serial error.");
            node->ros_ser.close();
            node->serial_is_init = false;
        }

        // 略微 sleep，让 CPU 不要太满
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    rclcpp::shutdown();
    return 0;
}