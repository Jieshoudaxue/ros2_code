#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "component_demo/sub_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(component_demo::SubComponent)

namespace component_demo
{
SubComponent::SubComponent(const rclcpp::NodeOptions & options)
                            : Node("sub_component", options) {
    auto msg_callback = [this](std_msgs::msg::String::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
    };

    sub_msg_ = create_subscription<std_msgs::msg::String>("hello_msg", 10, msg_callback);
}

};  // namespace component_demo