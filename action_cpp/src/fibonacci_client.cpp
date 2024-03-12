#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "diy_interface/action/fibonacci.hpp"

namespace action_cpp {

class FibonacciActionClient : public rclcpp::Node {
public:
    using Fibonacci = diy_interface::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    FibonacciActionClient() : Node("fibonacci_action_client") {
        client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
        timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(500),
                        std::bind(&FibonacciActionClient::send_goal, this));
        RCLCPP_INFO(this->get_logger(), "Fibonacci action client has been started");
    }

    void send_goal() {
        // 取消定时器，因此只调用一次，发送一次action goal 给 server
        this->timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "waiting for action server");

        if (!client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "action server is available");

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;
        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&FibonacciActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&FibonacciActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&FibonacciActionClient::result_callback, this, std::placeholders::_1);
        client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "send goal to server, order is %d", goal_msg.order);
    }

private:
    void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "goal accepted by server, waiting for result");
        }
    }

    // 第一个参数虽然没有使用，但是必须写上，否则编译器会报错
    void feedback_callback(GoalHandleFibonacci::SharedPtr,
                        const std::shared_ptr<const Fibonacci::Feedback> feedback) {
        std::stringstream ss;
        ss << "next number in sequence received: ";
        for (auto number : feedback->partial_sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "unknown result code");
                return;
            
            std::stringstream ss;
            ss << "result received: ";
            for (auto number : result.result->sequence) {
                ss << number << " ";
            }
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            rclcpp::shutdown();
            return;
        }
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

}; // class FibonacciActionClient


} // namespace action_cpp

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<action_cpp::FibonacciActionClient>());

    rclcpp::shutdown();

    return 0;
}