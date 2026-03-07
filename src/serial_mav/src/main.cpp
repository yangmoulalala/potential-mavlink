#include "mavlink.hpp"
#include <thread>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MavLink>();

    // ROS executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // 串口线程
    std::thread serial_thread([node]() {

        mavlink_message_t msg;
        mavlink_status_t status;

        while (rclcpp::ok()) {

            if (!node->serial_is_init) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                node->serial_init();
                continue;
            }

            try {

                size_t n = node->ros_ser.available();

                if (n > 0) {

                    std::vector<uint8_t> buf(n);
                    node->ros_ser.read(buf, n);

                    for (uint8_t c : buf) {

                        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
                            node->parse_mavlink_msg(msg);
                        }
                    }
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

            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    });

    // ROS线程
    executor.spin();

    serial_thread.join();

    rclcpp::shutdown();
}