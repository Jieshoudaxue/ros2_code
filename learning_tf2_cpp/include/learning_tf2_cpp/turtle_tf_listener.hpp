#ifndef LEARNING_TF2_CPP__TURTLE_TF_LISTENER_HPP_
#define LEARNING_TF2_CPP__TURTLE_TF_LISTENER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "learning_tf2_cpp/visibility_control.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"


namespace learning_tf2_cpp
{

class TurtleTfListener : public rclcpp::Node
{
public:
    LEARNING_TF2_CPP_PUBLIC
    explicit TurtleTfListener(const rclcpp::NodeOptions & options);

private:
    void on_timer();

private:
    bool turtle_spawning_service_ready_ = false;
    bool turtle_spawned_ = false;

    std::string target_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_turtle_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};



} // namespace learning_tf2_cpp


#endif // LEARNING_TF2_CPP__TURTLE_TF_LISTENER_HPP_