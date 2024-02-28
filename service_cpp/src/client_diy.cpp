#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "diy_interface/srv/question_and_answer.hpp"

class ServiceClient : public rclcpp::Node 
{
public:
    ServiceClient() : Node("service_client") {
        client_ = this->create_client<diy_interface::srv::QuestionAndAnswer>("question_and_answer");
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

    void run(const std::string &str) {
        auto request = std::make_shared<diy_interface::srv::QuestionAndAnswer::Request>();
        request->question = str;
        
        auto result = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
                                                        rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "[cpp client] send: %s, receive: %s", 
                                                request->question.c_str(), result.get()->answer.c_str()); 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
        } 
    }

private:
    rclcpp::Client<diy_interface::srv::QuestionAndAnswer>::SharedPtr client_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceClient>();
    node->init();
    while (rclcpp::ok()) {
        std::string str = "how are you?";
        node->run(str);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::shutdown();

    return 0;
}