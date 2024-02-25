#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "builtin_interfaces/msg/duration.hpp"

class Publisher : public rclcpp::Node 
{
public:
    Publisher() : Node("test_qos_publisher"), count_(0) {
        // set qos profile
        // set history depth to 10, 10 is also the default value
        rclcpp::QoS qos_profile(10);
        // RMW_QOS_POLICY_RELIABILITY_RELIABLE(default), RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        // RMW_QOS_POLICY_DURABILITY_VOLATILE(default)
        // RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL(Only works when reliability is RELIABLE and needs history:KEEP_LAST and depth)
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        // history: only works when reliability is RELIABLE
        // RMW_QOS_POLICY_HISTORY_KEEP_LAST(default), RMW_QOS_POLICY_HISTORY_KEEP_ALL
        qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        // default is Infinite
        qos_profile.lifespan(rclcpp::Duration(5, 0));

        // default is Infinite
        qos_profile.deadline(rclcpp::Duration(5, 0));
        // RMW_QOS_POLICY_LIVELINESS_AUTOMATIC(default), RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC
        qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
        // default is Infinite
        qos_profile.liveliness_lease_duration(rclcpp::Duration(5, 0));

        publisher_ = this->create_publisher<std_msgs::msg::String>("hello_topic", qos_profile);
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