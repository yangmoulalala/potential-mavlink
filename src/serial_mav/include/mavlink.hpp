#pragma once

// ─── ROS2 核心 ────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
// ─── TF2 ──────────────────────────────────────────────────────────────────────
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// ─── 自定义消息 ───────────────────────────────────────────────────────────────
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/srv/set_mode.hpp"

// ─── 串口 & MAVLink ───────────────────────────────────────────────────────────
#include "serial/serial.h"
#include "mavlink/msg/mavlink.h"

// ─── 标准库 ───────────────────────────────────────────────────────────────────
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>

// ─── 常量定义 ─────────────────────────────────────────────────────────────────
namespace constants
{
    constexpr double IMU_DISTANT        = 0.2;    // IMU 到云台中心距离 (m)
    constexpr float  DEFAULT_BULLET_SPD = 20.0f;  // 默认弹速 (m/s)
    constexpr float  MIN_BULLET_SPD     = 10.0f;
    constexpr float  MAX_BULLET_SPD     = 30.0f;
    constexpr int    TEAM_RED           = 0;
    constexpr int    TEAM_BLUE          = 1;
    constexpr int    TIMER_PERIOD_MS    = 10;     // 定时器周期 (Ms)
}  // namespace constants


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
    sensor_msgs::msg::Imu imu_data;
    geometry_msgs::msg::Point target_point;
    

    int   team_color_request        = constants::TEAM_RED;
    float bullet_speed_request      = constants::DEFAULT_BULLET_SPD;

private:


    // =========================================================================
    // ROS2 通信句柄
    // =========================================================================
    rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr    target_sub_;
    rclcpp::Subscription<rm_interfaces::msg::Armors>::SharedPtr    armors_sub_;
    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr            imu_pub_;
    rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr         set_color_client_;
    rclcpp::TimerBase::SharedPtr                                   timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                 tf_broadcaster_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr               nav_client_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odometry_sub_;
 
    // =========================================================================
    // 内部状态
    // =========================================================================
    rm_interfaces::msg::Target               target_;
    rm_interfaces::msg::Armors               armors_;
    rm_interfaces::msg::GimbalCmd            gimbal_cmd_;
    std::optional<rm_interfaces::msg::Armor> best_armor_;
    geometry_msgs::msg::Twist                cmd_vel_;
    nav_msgs::msg::Odometry                  odometry_;

    int   team_color_       = constants::TEAM_RED;
    float bullet_speed_     = constants::DEFAULT_BULLET_SPD;
    float last_cmd_yaw_     = 0.0f;
    float last_cmd_pitch_   = 0.0f;

    rclcpp::Time last_gimbal_time_;
    rclcpp::Time last_set_color_time_;

    // =========================================================================
    // 回调函数
    // =========================================================================
    void target_callback(const rm_interfaces::msg::Target::SharedPtr msg);
    void armors_callback(const rm_interfaces::msg::Armors::SharedPtr msg);
    void gimbal_callback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg);
    void timer_callback();
    void set_color_callback(rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture response);
    
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void nav_goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& goal_handle);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // =========================================================================
    // 业务逻辑
    // =========================================================================
    void select_best_armor();
    void send_gimbal_cmd();
    void send_cmd_vel();
    void set_color();
    void set_bullet_speed();

    // =========================================================================
    // 发布接口
    // =========================================================================
    void publish_imu();
    void publish_tf();
    void publish_nav_goal();


    // =========================================================================
    // 工具函数
    // =========================================================================
    // void wait_for_detector_service();
    std::string to_hex_string(const std::string& data);
};