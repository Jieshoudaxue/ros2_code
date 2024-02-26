#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class ServiceClient : public rclcpp::Node 
{
public:
    ServiceClient() : Node("service_client") {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    int init() {
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return 0;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        return 0;
    }

    void run(const int pa, const int pb) {
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = pa;
        request->b = pb;
        
        auto result = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
                                                        rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "client: send %ld(a) + %ld(b), receive %ld(sum)", 
                                                    request->a, request->b, result.get()->sum);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
        } 
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceClient>();
    node->init();
    while (rclcpp::ok()) {
        srand((unsigned)time(NULL));
        node->run(rand(), rand());
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::shutdown();

    return 0;
}