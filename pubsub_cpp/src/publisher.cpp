#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Publisher : public rclcpp::Node 
{
public:
    Publisher() : Node("test_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("hello_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Publisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto msg = std_msgs::msg::String();
        msg.data = "hello, i am fine in cpp! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "publishing: %s", msg.data.c_str());
        publisher_->publish(msg);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();

    return 0;
}