#include "mavlink.hpp"


/**
 * @brief 消费串口缓冲区，逐字节送入 MAVLink 解析器。
 */
static void serial_callback(MavLink& node)
{

    static mavlink_message_t mav_msg;
    static mavlink_status_t mav_status;

    int buf_len = node.ros_ser.available();
    if (buf_len <= 0)
        return;

    std::string serial_read = node.ros_ser.read(buf_len);
    for (char c : serial_read) {
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &mav_msg, &mav_status)) {
            node.prase_mavlink_msg(mav_msg);
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
                serial_callback(*node);
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