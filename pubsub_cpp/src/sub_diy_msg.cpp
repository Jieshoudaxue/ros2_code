#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "diy_interface/msg/student.hpp"
#include "diy_interface/msg/sphere.hpp"

class Subscriber : public rclcpp::Node 
{
public:
    Subscriber() : Node("test_sub_diy_msg") {
        sub_student_ = this->create_subscription<diy_interface::msg::Student>(
            "student_topic", 10, std::bind(&Subscriber::student_topic_callback, this, std::placeholders::_1));
        sub_sphere_ = this->create_subscription<diy_interface::msg::Sphere>(
            "sphere_topic", 10, std::bind(&Subscriber::sphere_topic_callback, this, std::placeholders::_1));
    }

private:
    void student_topic_callback(const diy_interface::msg::Student &msg) const {
        RCLCPP_INFO(this->get_logger(), "i received student in cpp: %s, %d", msg.name.c_str(), msg.age);
    }
    void sphere_topic_callback(const diy_interface::msg::Sphere &msg) const {
        RCLCPP_INFO(this->get_logger(), "i received sphere in cpp: (%f, %f, %f), %f", 
            msg.center.x, msg.center.y, msg.center.z, msg.radius);
    }

private:
    rclcpp::Subscription<diy_interface::msg::Student>::SharedPtr sub_student_;
    rclcpp::Subscription<diy_interface::msg::Sphere>::SharedPtr sub_sphere_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();

    return 0;
}