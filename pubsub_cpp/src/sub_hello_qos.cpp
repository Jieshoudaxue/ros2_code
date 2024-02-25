#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "builtin_interfaces/msg/duration.hpp"

class Subscriber : public rclcpp::Node 
{
public:
    Subscriber() : Node("test_qos_subscriber") {
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

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "hello_topic", qos_profile, std::bind(&Subscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String &msg) const {
        RCLCPP_INFO(this->get_logger(), "i received: %s", msg.data.c_str());
        sleep(2);    // test qos history and depth parameters
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