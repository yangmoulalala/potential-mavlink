#include "mavlink.hpp"
#include <thread>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavLink>();

    // 使用多线程 Executor 提高响应速度
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread serial_thread([node]() {
        while (rclcpp::ok()) {
            if (!node->is_serial_ok()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                node->serial_init();
                continue;
            }

            try {
                node->receive_and_parse();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node->get_logger(), "Serial thread error: %s", e.what());
            }
            
            // 降低 CPU 占用，串口 115200 波特率下 500us 足够快了
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    });

    executor.spin();

    rclcpp::shutdown();
    if (serial_thread.joinable()) serial_thread.join();
    return 0;
}