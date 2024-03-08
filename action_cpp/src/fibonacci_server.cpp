#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "diy_interface/action/fibonacci.hpp"
namespace action_cpp {

class FibonacciActionServer : public rclcpp::Node {
public:
    using Fibonacci = diy_interface::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
    FibonacciActionServer() : Node("fibonacci_action_server") {
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&FibonacciActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FibonacciActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FibonacciActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Fibonacci action server has been started");
    }

private:
    // uuid 参数虽然没有使用，但是必须写上，否则编译器会报错
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, 
                                            std::shared_ptr<const Fibonacci::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // goal_handle 参数虽然没有使用，但是必须写上，否则编译器会报错
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
        std::thread{std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Fibonacci::Result>();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        feedback->partial_sequence.push_back(0);
        feedback->partial_sequence.push_back(1);

        for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
            if (goal_handle->is_canceling()) {
                // 取消任务时，把 feedback 的斐波那契数组保存到 result 的斐波那契数组中
                result->sequence = feedback->partial_sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            feedback->partial_sequence.push_back(feedback->partial_sequence[i] + feedback->partial_sequence[i-1]);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "publish feedback");
            loop_rate.sleep();
        }

        if (rclcpp::ok()) {
            // 任务结束时，把 feedback 的斐波那契数组保存到 result 的斐波那契数组中
            result->sequence = feedback->partial_sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};  // class FibonacciActionServer

}   // namespace action_cpp

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<action_cpp::FibonacciActionServer>());

    rclcpp::shutdown();

    return 0;
}