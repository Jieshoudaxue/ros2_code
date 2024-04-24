#include "learning_tf2_cpp/turtle_tf_listener.hpp"

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(learning_tf2_cpp::TurtleTfListener)

namespace learning_tf2_cpp
{

TurtleTfListener::TurtleTfListener(const rclcpp::NodeOptions & options)
                                        : Node("turtle_tf_listener", options) {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is used to pass in the name of the target turtle";
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1", param_desc);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    spawn_turtle_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
    turtle_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TurtleTfListener::on_timer, this));


    RCLCPP_INFO(this->get_logger(), "TurtleTfListener has been started");
}

void TurtleTfListener::on_timer() {
    if (false == turtle_spawning_service_ready_) {
        if (false == spawn_turtle_client_->service_is_ready()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the spawn service to be ready");
            return;
        }
        turtle_spawning_service_ready_ = true;
    }

    if (false == turtle_spawned_) {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 5.0;
        request->y = 5.0;
        request->name = "turtle2";

        using ServiceResponseFuture = rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            if (strcmp(result->name.c_str(), "turtle2") == 0) {
                  turtle_spawned_ = true;
                  RCLCPP_INFO(this->get_logger(), "Spawn turtle2 successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
            }
        };
        auto result = spawn_turtle_client_->async_send_request(request, response_received_callback);
        return;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform("turtle2", target_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not lookup transform: %s", ex.what());
        return;
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.5 * std::sqrt(pow(tf_stamped.transform.translation.x, 2) + pow(tf_stamped.transform.translation.y, 2));
    cmd_vel.angular.z = 1.0 * std::atan2(tf_stamped.transform.translation.y, tf_stamped.transform.translation.x);

    RCLCPP_INFO(this->get_logger(), "send cmd_vel to turtle2: linear.x: %f, angular.z: %f", cmd_vel.linear.x, cmd_vel.angular.z);
    turtle_cmd_pub_->publish(cmd_vel);
}


}