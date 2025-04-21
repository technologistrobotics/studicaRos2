#ifndef THREE_WHEELED_DRIVE_COMPONENT_H
#define THREE_WHEELED_DRIVE_COMPONENT_H

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "studica_odometry.h"
#include "studica_control/srv/set_data.hpp"
#include "titan.h"
#include "VMXPi.h"

namespace studica_control {

class ThreeWheelDrive : public rclcpp::Node {
public:
    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control, std::shared_ptr<ThreeWheelDriveOdometry> odom, std::shared_ptr<VMXPi> vmx);
    explicit ThreeWheeledDrive(const rclcpp::NodeOptions & options);
    ThreeWheeledDrive(
        std::shared_ptr<VMXPi> vmx,
        std::shared_ptr<studica_control::ThreeWheeledDriveOdometry> odom,
        const std::string &name,
        const uint8_t &canID,
        const uint16_t &motor_freq,
        const uint16_t &ticks_per_rotation,
        const float &wheel_radius,
        //const float &wheel_separation,
        const float &robot_radius, //new
        const uint8_t front, //new
        const uint8_t left,
        const uint8_t right,
        const bool invert_front, //new
        const bool invert_left,
        const bool invert_right
    );
    ~ThreeWheeledDrive();

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_control::ThreeWheeledDriveOdometry> odom_;
    std::shared_ptr<studica_driver::Titan> titan_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    uint8_t can_id_;
    uint16_t motor_freq_;
    float ticks_per_rotation_;
    float dist_per_tick_;
    float wheel_radius_;
    //float wheel_separation_;
    float robot_radius_, 
    uint8_t front_; //new
    uint8_t left_;
    uint8_t right_;
    
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publish_odometry();
};

}  // namespace studica_control

#endif  // TITAN_COMPONENT_H
