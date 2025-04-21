#ifndef ULTRASONIC_COMPONENT_H
#define ULTRASONIC_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "studica_control/srv/set_data.hpp"
#include "ultrasonic.h"
#include "VMXPi.h"

namespace studica_control {
    
class Ultrasonic : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);
    explicit Ultrasonic(const rclcpp::NodeOptions &options);
    Ultrasonic(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex ping, VMXChannelIndex echo, const std::string &topic);
    ~Ultrasonic();

private:
    std::shared_ptr<studica_driver::Ultrasonic> ultrasonic_;
    std::shared_ptr<VMXPi> vmx_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    VMXChannelIndex ping_;
    VMXChannelIndex echo_;
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_range();
    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // ULTRASONIC_COMPONENT_H
