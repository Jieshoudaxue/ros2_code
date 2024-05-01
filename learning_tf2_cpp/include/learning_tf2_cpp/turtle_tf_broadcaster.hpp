#ifndef LEARNING_TF2_CPP__TURTLE_TF_BROADCASTER_HPP_
#define LEARNING_TF2_CPP__TURTLE_TF_BROADCASTER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "learning_tf2_cpp/visibility_control.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

namespace learning_tf2_cpp
{

class TurtleTfBroadcaster : public rclcpp::Node
{
public:
    LEARNING_TF2_CPP_PUBLIC
    explicit TurtleTfBroadcaster(const rclcpp::NodeOptions & options);

private:
    void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg);

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string turtle_name_;
};



} // namespace learning_tf2_cpp


#endif // LEARNING_TF2_CPP__TURTLE_TF_BROADCASTER_HPP_