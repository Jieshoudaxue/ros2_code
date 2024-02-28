#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "diy_interface/srv/question_and_answer.hpp"

class ServiceServer : public rclcpp::Node 
{
public:
    ServiceServer() : Node("service_server") {
        server_ = this->create_service<diy_interface::srv::QuestionAndAnswer>("question_and_answer", 
            std::bind(&ServiceServer::answer_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
private:
    void answer_callback(const std::shared_ptr<diy_interface::srv::QuestionAndAnswer::Request> request,
                    std::shared_ptr<diy_interface::srv::QuestionAndAnswer::Response> response) {
        response->answer = "I'm fine, thank you.";
        RCLCPP_INFO(this->get_logger(), "[cpp server] receive: %s, return: %s", 
                                        request->question.c_str(), response->answer.c_str());
    }

private:
    rclcpp::Service<diy_interface::srv::QuestionAndAnswer>::SharedPtr server_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServiceServer>());
    rclcpp::shutdown();

    return 0;
}