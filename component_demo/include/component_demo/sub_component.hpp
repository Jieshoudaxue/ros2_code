#ifndef COMPONENT_DEMO__SUB_COMPONENT_HPP_
#define COMPONENT_DEMO__SUB_COMPONENT_HPP_

#include "component_demo/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace component_demo
{
// 注释请查看 pub_component.hpp
class SubComponent : public rclcpp::Node
{
public:
    COMPONENT_DEMO_PUBLIC
    explicit SubComponent(const rclcpp::NodeOptions & options);

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_msg_;
};


} // namespace component_demo


#endif // COMPONENT_DEMO__SUB_COMPONENT_HPP_