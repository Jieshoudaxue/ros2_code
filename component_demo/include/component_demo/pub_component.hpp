#ifndef COMPONENT_DEMO__PUB_COMPONENT_HPP_
#define COMPONENT_DEMO__PUB_COMPONENT_HPP_

// 必须配置可见头文件，主要是为了兼容 lunix 和 windows
#include "component_demo/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace component_demo
{

class PubComponent : public rclcpp::Node 
{
public:
    // 来自可见头文件
    COMPONENT_DEMO_PUBLIC
    // explicit 是为了防止实例化PubComponent时的隐式转换
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