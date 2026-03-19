#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "rm_interfaces/msg/cboard.hpp"
#include "rm_interfaces/msg/referee.hpp"

#include "serial/serial.h"
#include "mavlink/msg/mavlink.h"

#include <mutex>
#include <memory>

namespace constants {
    constexpr int TIMER_PERIOD_MS = 100; 
}

class MavLink : public rclcpp::Node {
public:
    explicit MavLink();
    ~MavLink() override;

    void serial_init();
    void parse_mavlink_msg(const mavlink_message_t& msg);
    
    // 供主循环调用的读取接口
    void receive_and_parse();
    bool is_serial_ok() const { return serial_is_init_ && ros_ser_.isOpen(); }

private:
    // 串口发送封装（线程安全）
    void send_mavlink(const mavlink_message_t& msg);

    // 回调函数
    void gimbal_callback(const rm_interfaces::msg::Cboard::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timer_callback();

    // 内部处理
    void process_gimbal_logic(rm_interfaces::msg::Cboard& cmd);

    // 句柄
    serial::Serial ros_ser_;
    std::mutex serial_mtx_;
    std::atomic<bool> serial_is_init_{false};

    // ROS 通信
    rclcpp::Subscription<rm_interfaces::msg::Cboard>::SharedPtr gimbal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<rm_interfaces::msg::Referee>::SharedPtr referee_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pos_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 数据缓存
    sensor_msgs::msg::Imu imu_msg_;
    rm_interfaces::msg::Referee referee_msg_;
    geometry_msgs::msg::Point target_point_;
    
    // 记录上一次的状态用于丢失目标时的保持
    struct {
        float yaw = 0, pitch = 0, wr = 0, distance = 0;
        int robot_id = 0;
    } last_state_;
};