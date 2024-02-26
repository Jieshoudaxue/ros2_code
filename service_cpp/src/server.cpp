#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class ServiceServer : public rclcpp::Node 
{
public:
    ServiceServer() : Node("service_server") {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", 
            std::bind(&ServiceServer::add_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
private:
    void add_callback(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "server: receive %ld(a) + %ld(b), send %ld(sum)", request->a, request->b, response->sum);
    }

private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServiceServer>());
    rclcpp::shutdown();

    return 0;
}