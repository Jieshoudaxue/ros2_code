#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rclcpp/rclcpp.hpp"

class MiniParam : public rclcpp::Node {
public:
    MiniParam() : Node("param_monitor_cpp") {
        // set param description
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter is mine!";
        this->declare_parameter("demo_param", "set in cpp !!", param_desc);

        // used to monitor parameter changes
        param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        // 监测本节点的参数变化
        demo_param_change_cb_ = param_sub_->add_parameter_callback("demo_param", 
        [this](const rclcpp::Parameter &p) {
            RCLCPP_INFO(this->get_logger(), "received an update to : %s , type is %s, value is %s", 
                        p.get_name().c_str(),
                        p.get_type_name().c_str(),
                        p.as_string().c_str());
        });
        // 监测其他节点的参数变化
        turtlesim_node_background_b_change_cb_ = param_sub_->add_parameter_callback("background_b",
        [this](const rclcpp::Parameter &p) {
            RCLCPP_INFO(this->get_logger(), "received an update to : %s , type is %s, value is %ld", 
                        p.get_name().c_str(),
                        p.get_type_name().c_str(),
                        p.as_int());
        }, "/turtlesim_node");
    }

private:
    std::shared_ptr<rclcpp::ParameterEventHandler> param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> demo_param_change_cb_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> turtlesim_node_background_b_change_cb_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<MiniParam>());

    rclcpp::shutdown();

    return 0;
}