#include "rclcpp/rclcpp.hpp"

class HelloRos2 : public rclcpp::Node {
public:
    HelloRos2() : Node("hello_ros2") {
        RCLCPP_INFO(this->get_logger(), "start hello ros2 in cpp !!");
    }

    void run() {
        while(rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "hello ros2 in cpp !!");
            sleep(1);
        }
    }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HelloRos2>();
    
    node->run();
    
    rclcpp::shutdown();

    return 0;
}