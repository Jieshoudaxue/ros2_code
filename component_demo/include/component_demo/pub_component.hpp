#ifndef COMPONENT_DEMO__PUB_COMPONENT_HPP_
#define COMPONENT_DEMO__PUB_COMPONENT_HPP_

#include "component_demo/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace component_demo
{

class PubComponent : public rclcpp::Node 
{
public:
    COMPONENT_DEMO_PUBLIC
    explicit PubComponent(const rclcpp::NodeOptions & options);

protected:
    void on_timer();

private:
    size_t count_;
    std::string msg_inner_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace component_demo


#endif // COMPONENT_DEMO__PUB_COMPONENT_HPP_