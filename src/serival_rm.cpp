#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//串口库
#include "serial/serial.h"
//rm msgs 自定义消息类型
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "serial_mav/msg/mavlink.h"
using std::placeholders::_1;
//using namespace std::chrono_literals;

//imu到云台中心的距离
#define IMU_DISTANT 0.2
#define TEAM_COLOR_RED 0
#define TEAM_COLOR_BULE 1


class MavTest1 : public rclcpp::Node
{
public:
    explicit MavTest1() : Node("MavTest_1")
    {
        
        serial_init();
        //目标订阅
        target_sub_ = this->create_subscription<rm_interfaces::msg::Target>(
            "/armor_solver/target", rclcpp::SensorDataQoS(), std::bind(&MavTest1::target_callback, this, _1));
        armors_sub_ = this->create_subscription<rm_interfaces::msg::Armors>(
            "armor_detector/armors", rclcpp::SensorDataQoS(), std::bind(&MavTest1::armors_callback, this, _1));
        //云台cmd订阅
        gimbal_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
            "/armor_solver/cmd_gimbal", rclcpp::SensorDataQoS(), std::bind(&MavTest1::gimbal_callback, this, _1));
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/serial_dirver/imu_raw",rclcpp::SensorDataQoS());
        // armors_sub_.subscribe(this, "armor_detector/armors", rmw_qos_profile_sensor_data);
        // 创建定时器，每秒触发一次回调函数
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(10), std::bind(&MavTest1::timer_callback, this));
        
        set_color_client_ = this->create_client<rm_interfaces::srv::SetMode>("/armor_detector/set_mode");
        while (!set_color_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rm_vision is not started!");
        if(serial_is_init)
            ros_ser.read(ros_ser.available());
        }
    }
    std::string serial_read(int len)
    {
        return ros_ser.read(len);
    }
    sensor_msgs::msg::Imu imu_data;
    serial::Serial ros_ser;
    bool serial_is_init = false;
    int team_color = TEAM_COLOR_RED;
    float bullet_speed = 20;//弹丸初速度
    float bullet_speed_request = bullet_speed;//请求设置的初速度
    float last_gimbal_cmd_yaw_ = 0;
    float last_gimbal_cmd_pitch_ = 0;
    int team_color_request = team_color;//由串口来更新
    rclcpp::Time last_gimbal_ballback = this->now();//上一次云台指令
    rclcpp::Time last_set_color=this->now();

    void serial_init()
    {   
        if (ros_ser.isOpen()) {
            try {
                ros_ser.close();
            } catch (const serial::IOException& e) {
                RCLCPP_ERROR(this->get_logger(), "Error closing serial port: %s", e.what());
            }
        }
        
        ros_ser.setPort("/dev/ttyACM0");
        ros_ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(20);
        ros_ser.setTimeout(to);
        try {
            ros_ser.open();
            if (ros_ser.isOpen()) {
                serial_is_init = true;
                RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
            } else {
                serial_is_init = false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception opening serial port: %s", e.what());
            serial_is_init = false;
        }
    }


private:
    rm_interfaces::msg::Target target;
    rm_interfaces::msg::Armors armors;
    // rm_interfaces::msg::Armor best_armor;
    std::optional<rm_interfaces::msg::Armor> best_armor;
    rm_interfaces::msg::GimbalCmd gimbal_cmd;
    
    
    std::string string_to_hex(const std::string& input) {
        std::stringstream ss;
        ss << std::hex << std::uppercase;
        
        for (unsigned char c : input) {
            ss << std::setw(2) << std::setfill('0') 
            << static_cast<int>(c) << " ";
        }
        
        // 移除最后一个空格
        std::string result = ss.str();
        if (!result.empty()) result.pop_back();
        return result;
    }
    void log_info(const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        
        // 计算需要的缓冲区大小
        va_list args_copy;
        va_copy(args_copy, args);
        int size = vsnprintf(nullptr, 0, format, args_copy) + 1; // +1 for null terminator
        va_end(args_copy);
        
        if (size > 0)
        {
            std::vector<char> buffer(size);
            vsnprintf(buffer.data(), size, format, args);
            RCLCPP_INFO(this->get_logger(), "%s", buffer.data());
        }
        
        va_end(args);
    }
    
    void target_callback(const rm_interfaces::msg::Target::SharedPtr target_msg)
    {
        target = *target_msg;
        //log_info("target id : %s \n",target_msg->id.c_str());
        //log_info("position: %.4lf %.4lf %.4lf\n",target_msg->position.x,target_msg->position.y,target_msg->position.z);
    }
    void armors_callback(const rm_interfaces::msg::Armors::SharedPtr armors_msg)
    {

        armors = *armors_msg;
    }
    // }
    void gimbal_callback(const rm_interfaces::msg::GimbalCmd::SharedPtr gimbal_msg)
    {
        last_gimbal_ballback = this->now();
        gimbal_cmd = *gimbal_msg;
    }
    //100hz callback 串口消息解析以及更新变量值 发布动态tf坐标变换
    void timer_callback()
    {
        if (serial_is_init) {
            try {
                uart_process();
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Error in uart process");
                ros_ser.close();
                serial_is_init = false;
            }
        }
        set_color();
        
        imu_publish();//publish imu
        tf_publish();//publish tf
    }
    // void setect_armor(){
    //     for (const auto &armor : armors->armors){

    //     }
    // }
    void select_armor()
    {
        best_armor.reset();
        if (armors.armors.empty() || !target.tracking) {
            return;
        }

        double min_dist = std::numeric_limits<double>::max();
        rm_interfaces::msg::Armor best_armor_limit;
        for (const auto &armor : armors.armors) {

            double dx = armor.pose.position.x - target.position.x;
            double dy = armor.pose.position.y - target.position.y;
            double dz = armor.pose.position.z - target.position.z;
            
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            // RCLCPP_INFO(this->get_logger(),"dist: %f", dist);
            if (dist < min_dist) {
                min_dist = dist;
                best_armor_limit = armor;  
            }
        }
        best_armor = best_armor_limit;
        if (best_armor.has_value()) {
        // RCLCPP_INFO(this->get_logger(),
        // "Armor: number %s, distance_to_image_center %.2f",
        // best_armor->number.c_str(),
        // best_armor->distance_to_image_center);
        RCLCPP_INFO(this->get_logger(), "best_armor.pose.position.z %f", best_armor->pose.position.z);
        }

    }

    void set_bullet_speed()
    {
        if(bullet_speed!=bullet_speed_request)
        {
            std::vector<rclcpp::Parameter> new_params = {
                rclcpp::Parameter("/armor_solver.solver.bullet_speed",bullet_speed_request)
            };
            auto result = this->set_parameters_atomically(new_params);
            if(result.successful)
            {
                bullet_speed = bullet_speed_request;
            }
        }
    }
    //识别颜色，裁判系统数据更新
    void set_color()
    {
        if(team_color!=team_color_request)//如果自身颜色发生改变
        {
            auto set_color_request_ = std::make_shared<rm_interfaces::srv::SetMode_Request>();
            switch (team_color_request)
            {
                case TEAM_COLOR_RED:
                {
                    set_color_request_->mode = TEAM_COLOR_RED;
                    break;
                }   
                case TEAM_COLOR_BULE:
                {
                    set_color_request_->mode = TEAM_COLOR_BULE;
                    break;
                }
                default:
                {
                    RCLCPP_ERROR(this->get_logger(),"Wrong team color!!!");
                    return;
                }
                
            }
            if(this->now()-last_set_color>std::chrono::seconds(1))
            {
                log_info("start change color");
                set_color_client_->async_send_request(set_color_request_,std::bind(&MavTest1::set_color_callback,this,_1));
                last_set_color = this->now();
            }
                
        }
    }
    void uart_process()
    {   
        select_armor();
        uint8_t locked = 0;
        uint8_t is_detected = 0;
        if(gimbal_cmd.pitch!=0 && gimbal_cmd.yaw!=0)
        {
            locked = 1;
        }
        if (best_armor.has_value()) {
            is_detected = 1;
            last_gimbal_cmd_yaw_ = gimbal_cmd.yaw;
            last_gimbal_cmd_pitch_ = gimbal_cmd.pitch;
            RCLCPP_INFO(this->get_logger(),"target yaw_v %f", target.v_yaw);
            // RCLCPP_INFO(this->get_logger(), "corners: %f, %f, %f, %f", best_armor->corners[0].x, best_armor->corners[1].x, best_armor->corners[2].x, best_armor->corners[3].x);
        }
        else{
            gimbal_cmd.yaw = last_gimbal_cmd_yaw_;
            gimbal_cmd.pitch = last_gimbal_cmd_pitch_;
        }
        mavlink_message_t gimbal_msg;
        // RCLCPP_INFO(this->get_logger(), "is_detected %d, fire_advice %d, yaw %f, pitch %f", is_detected, gimbal_cmd.fire_advice, gimbal_cmd.yaw, gimbal_cmd.pitch);
        uint8_t buf[128];
        mavlink_msg_aim_pack(1,200,&gimbal_msg, is_detected,gimbal_cmd.fire_advice,gimbal_cmd.yaw,gimbal_cmd.pitch);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &gimbal_msg);
        // std::string send_string(reinterpret_cast<char*>(buf), len);
        ros_ser.write(buf, len);
    }
    // void imu_publish() argv[]
    // {
        
    //     imu_pub_->publish(imu_data);
    // }
    void imu_publish()
    {
    imu_data.header.stamp = this->now();
    imu_data.header.frame_id = "imu_link";
    // RCLCPP_INFO(this->get_logger(), "imu_publish called");
    imu_pub_->publish(imu_data);
    }

    //发布imu角度
    void tf_publish()
    {
        geometry_msgs::msg::TransformStamped t;
        // 设置时间戳
        t.header.stamp = this->get_clock()->now();
        // 设置父坐标系
        t.header.frame_id = "odom";
        // 设置子坐标系
        t.child_frame_id = "gimbal_link";

        //四元数转欧拉角
        tf2::Quaternion quaternion;
        quaternion.setX(imu_data.orientation.x);
        quaternion.setY(imu_data.orientation.y);
        quaternion.setZ(imu_data.orientation.z);
        quaternion.setW(imu_data.orientation.w);
        // 设置变换的旋转部分
        t.transform.rotation.x = quaternion.x();
        t.transform.rotation.y = quaternion.y();
        t.transform.rotation.z = quaternion.z();
        t.transform.rotation.w = quaternion.w();
        // 发布变换
        tf_broadcaster_->sendTransform(t);
    }
    void set_color_callback(rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture response)
    {
        auto result = response.get();
        if(result->message=="0")
        {
            team_color=team_color_request;//已经设置成功
            RCLCPP_WARN(this->get_logger(),"Switch to color%d",team_color);
        }
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_{nullptr};
    rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr target_sub_{nullptr};
    rclcpp::Subscription<rm_interfaces::msg::Armors>::SharedPtr armors_sub_{nullptr};
    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_sub_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_{nullptr};
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr set_color_client_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MavTest1>();
    int serial_print_count = 0;//to control the print rate
    auto last_retry_time = std::chrono::steady_clock::now();
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (!node->serial_is_init) {
            auto now = std::chrono::steady_clock::now();
            if (now - last_retry_time > std::chrono::seconds(1)) {
                node->serial_init();
                last_retry_time = now;
            }
            continue;
        }

        try {
            if(node->serial_is_init)
            {
                mavlink_message_t mav_msg;
                mavlink_status_t mav_status;
                
                int buf_len = node->ros_ser.available();
                if(buf_len==0)
                    continue;
                std::string serial_read = node->ros_ser.read(buf_len);
                //std::cout<<"read from serial "<<buf_len<<std::endl;
                for (char c : serial_read) {
                    if(mavlink_parse_char(MAVLINK_COMM_0,c,&mav_msg,&mav_status))
                    {
                        //std::cout<<"decode a message! id is:"<<(int)mav_msg.msgid<<std::endl;
                        switch (mav_msg.msgid)
                        {
                            case MAVLINK_MSG_ID_imu:
                            {
                                mavlink_imu_t mav_imu_data;
                                mavlink_msg_imu_decode(&mav_msg,&mav_imu_data);
                                tf2::Quaternion quaternion;
                                //deg to rad
                                // RCLCPP_INFO(node->get_logger(), "Decoded IMU data: roll=%.2f, pitch=%.2f, yaw=%.2f", mav_imu_data.roll, mav_imu_data.pitch, mav_imu_data.yaw);
                                mav_imu_data.roll = mav_imu_data.roll/180.0*M_PI;
                                mav_imu_data.pitch = -mav_imu_data.pitch/180.0*M_PI;
                                mav_imu_data.yaw = mav_imu_data.yaw/180.0*M_PI;
                                if(serial_print_count>100)
                                {
                                    std::cout<<"decode_success!"<<mav_imu_data.roll<<" "<<mav_imu_data.pitch<<" "<<mav_imu_data.yaw<<std::endl;
                                    serial_print_count = 0;
                                }
                                quaternion.setRPY(mav_imu_data.roll,mav_imu_data.pitch,mav_imu_data.yaw);
                                node->imu_data.orientation.x = quaternion.getX();
                                node->imu_data.orientation.y = quaternion.getY();
                                node->imu_data.orientation.z = quaternion.getZ();
                                node->imu_data.orientation.w = quaternion.getW();
                                
                                
                                serial_print_count++;
                                break;
                            }
                            case MAVLINK_MSG_ID_referee:
                            {
                                mavlink_referee_t mav_referee_data;
                                mavlink_msg_referee_decode(&mav_msg,&mav_referee_data);
                                // RCLCPP_INFO(node->get_logger(), "Decoded referee data: color=%d", mav_referee_data.is_red);
                                if(mav_referee_data.is_red  == 0)//我是蓝方
                                {
                                    node->team_color_request = TEAM_COLOR_RED;
                                }
                                else
                                {
                                    node->team_color_request = TEAM_COLOR_BULE;
                                }
                                //弹丸速度区间 10m/s-30m/s
                                if(mav_referee_data.bullet_speed>=10&&mav_referee_data.bullet_speed<30)
                                    node->bullet_speed_request = mav_referee_data.bullet_speed;
                                break;
                            }
                        
                        default:
                        std::cout<<"decode faild!"<<std::endl;
                            break;
                        }
                    }
                }
            }
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(node->get_logger(), "Read failed: %s", e.what());
            node->ros_ser.close();
            node->serial_is_init = false;
        } catch (...) {
            RCLCPP_ERROR(node->get_logger(), "Unexpected read error");
            node->ros_ser.close();
            node->serial_is_init = false;
        }
    }
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}