#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pubsub_mix/msg/address_book.hpp"

class Publisher : public rclcpp::Node 
{
public:
    Publisher() : Node("address_book_publisher"), count_(0) {
        publisher_ = this->create_publisher<pubsub_mix::msg::AddressBook>("address_book", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Publisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto msg = pubsub_mix::msg::AddressBook();
        msg.first_name = "yi";
        msg.last_name = "cao";
        msg.phone_number = "123456789";
        msg.phone_type = msg.PHONE_TYPE_MOBILE;

        RCLCPP_INFO(this->get_logger(), "publishing in cpp: %s %s %s %d", 
            msg.first_name.c_str(), msg.last_name.c_str(), msg.phone_number.c_str(), msg.phone_type);
        publisher_->publish(msg);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<pubsub_mix::msg::AddressBook>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();

    return 0;
}