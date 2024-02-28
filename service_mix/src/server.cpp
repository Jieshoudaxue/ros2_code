#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "service_mix/srv/circle.hpp"

class ServiceServer : public rclcpp::Node 
{
public:
    ServiceServer() : Node("service_server") {
        server_ = this->create_service<service_mix::srv::Circle>("circle", 
            std::bind(&ServiceServer::area_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
private:
    void area_callback(const std::shared_ptr<service_mix::srv::Circle::Request> request,
                    std::shared_ptr<service_mix::srv::Circle::Response> response) {
        response->area = M_PI * request->radius * request->radius;
        RCLCPP_INFO(this->get_logger(), "[cpp server] receive radius: %u, return area: %lf", 
                                        request->radius, response->area);
    }

private:
    rclcpp::Service<service_mix::srv::Circle>::SharedPtr server_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServiceServer>());
    rclcpp::shutdown();

    return 0;
}