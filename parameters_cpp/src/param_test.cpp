#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

class MiniParam : public rclcpp::Node {
public:
    MiniParam() : Node("test_param_cpp") {
        // set param description
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter is mine!";
        // "set in cpp !!" will be cover by launch file: "set in launch !!"
        this->declare_parameter("demo_param", "set in cpp !!", param_desc);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MiniParam::timer_cb, this));
    }

    void timer_cb() {
        std::string demo_param = this->get_parameter("demo_param").as_string();
        RCLCPP_INFO(this->get_logger(), "demo_param: %s", demo_param.c_str());

        // Reset all parameters to prevent external modifications
        std::vector<rclcpp::Parameter> all_parameters;
        all_parameters.push_back(rclcpp::Parameter("demo_param", "set in cpp !!"));
        this->set_parameters(all_parameters);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<MiniParam>());

    rclcpp::shutdown();

    return 0;
}