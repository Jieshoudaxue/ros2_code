#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Subscriber : public rclcpp::Node 
{
public:
    Subscriber() : Node("test_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "hello_topic", 10, std::bind(&Subscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String &msg) const {
        RCLCPP_INFO(this->get_logger(), "i received: %s", msg.data.c_str());
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();

    return 0;
}