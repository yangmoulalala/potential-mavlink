#pragma once

// ─── ROS2 核心 ────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
// ─── TF2 ──────────────────────────────────────────────────────────────────────
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// ─── 自定义消息 ───────────────────────────────────────────────────────────────
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/cboard.hpp"
#include "rm_interfaces/msg/referee.hpp"

// ─── 串口 & MAVLink ───────────────────────────────────────────────────────────
#include "serial/serial.h"
#include "mavlink/msg/mavlink.h"

// ─── 标准库 ───────────────────────────────────────────────────────────────────
#include <memory>
#include <string>
#include <iomanip>
#include <cmath>
#include <limits>

namespace constants{
    constexpr int TIMER_PERIOD_100MS = 100; //10Hz
}

// ─── 节点类 ───────────────────────────────────────────────────────────────────
class MavLink : public rclcpp::Node
{
public:
    explicit MavLink();

    // ── 串口管理（main 线程访问） ──────────────────────────────────────────────
    void serial_init();

    // ── 对外暴露的状态（main 线程读写） ─────────────────────────────────────
    serial::Serial ros_ser;
    bool   serial_is_init   = false;

    void parse_mavlink_msg(const mavlink_message_t& msg);

private:

    // =========================================================================
    // ROS2 通信句柄
    // =========================================================================
    rclcpp::Subscription<rm_interfaces::msg::Cboard>::SharedPtr    gimbal_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr            imu_pub_;
    rclcpp::TimerBase::SharedPtr                                   timer_10hz_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                 tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odometry_sub_;
    rclcpp::Publisher<rm_interfaces::msg::Referee>::SharedPtr       referee_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr         target_position_pub_;

    // =========================================================================
    // 内部状态
    // =========================================================================
    sensor_msgs::msg::Imu imu_data_;
    geometry_msgs::msg::Point target_point_ ;
    rm_interfaces::msg::Referee referee_;

    rm_interfaces::msg::Cboard               gimbal_cmd_;
    geometry_msgs::msg::Twist                cmd_vel_;
    nav_msgs::msg::Odometry                  odometry_;

    float last_cmd_yaw_     = 0.0f;
    float last_cmd_pitch_   = 0.0f;
    bool last_is_fire_ = false;
    int last_robot_id_ = 0;
    float last_wr_ = 0;
    float last_distance_ = 0;


    rclcpp::Time last_gimbal_time_;
    rclcpp::Time last_set_color_time_;



    // 上一次发送的时间戳
    rclcpp::Time last_nav_send_time_{0, 0, RCL_ROS_TIME};

    // =========================================================================
    // 回调函数
    // =========================================================================
    void timer_10hz_callback();

    void gimbal_callback(const rm_interfaces::msg::Cboard::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // =========================================================================
    // 业务逻辑
    // =========================================================================

    void send_gimbal_cmd();
    void send_cmd_vel();
    void send_odometry();

    void send(const mavlink_message_t& msg);

    // =========================================================================
    // 发布接口
    // =========================================================================
    void publish_imu();
    void publish_tf();
    void publish_target_position();
    void publish_referee(); 
};