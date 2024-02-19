#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "diy_interface/msg/student.hpp"
#include "diy_interface/msg/sphere.hpp"

class Publisher : public rclcpp::Node 
{
public:
    Publisher() : Node("test_pub_diy_msg"), count_(0) {
        pub_student_ = this->create_publisher<diy_interface::msg::Student>("student_topic", 10);
        pub_sphere_ = this->create_publisher<diy_interface::msg::Sphere>("sphere_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Publisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto stu_msg = diy_interface::msg::Student();
        stu_msg.name = students_[count_ % students_.size()].first;
        stu_msg.age = students_[count_ % students_.size()].second;
        RCLCPP_INFO(this->get_logger(), "publishing student: %s, %d", stu_msg.name.c_str(), stu_msg.age);
        pub_student_->publish(stu_msg);

        auto sphere_msg = diy_interface::msg::Sphere();
        sphere_msg.center.x = std::get<0>(spheres_[count_ % spheres_.size()].first);
        sphere_msg.center.y = std::get<1>(spheres_[count_ % spheres_.size()].first);
        sphere_msg.center.z = std::get<2>(spheres_[count_ % spheres_.size()].first);
        sphere_msg.radius = spheres_[count_ % spheres_.size()].second;
        RCLCPP_INFO(this->get_logger(), "publishing sphere: (%f, %f, %f), %f", 
            sphere_msg.center.x, sphere_msg.center.y, sphere_msg.center.z, sphere_msg.radius);
        pub_sphere_->publish(sphere_msg);

        count_++;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<diy_interface::msg::Student>::SharedPtr pub_student_;
    rclcpp::Publisher<diy_interface::msg::Sphere>::SharedPtr pub_sphere_;
    size_t count_;

    std::vector<std::pair<std::string, int>> students_ = 
        {{"yi", 32}, {"miao", 18}, {"bao", 3}};
    std::vector<std::pair<std::tuple<float, float, float>, float>> spheres_ = 
        {{{1.0f, 2.0f, 3.0f}, 4.0f}, {{1.1f, 2.1f, 3.1f}, 4.1f}};

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();

    return 0;
}