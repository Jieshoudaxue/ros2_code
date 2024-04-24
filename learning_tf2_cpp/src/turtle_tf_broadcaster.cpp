#include "learning_tf2_cpp/turtle_tf_broadcaster.hpp"

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(learning_tf2_cpp::TurtleTfBroadcaster)

namespace learning_tf2_cpp
{

TurtleTfBroadcaster::TurtleTfBroadcaster(const rclcpp::NodeOptions & options)
                                        : Node("turtle_tf_broadcaster", options) {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is used to pass in the name of the turtle";
    turtle_name_ = this->declare_parameter<std::string>("turtle_name", "turtle1", param_desc);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    std::string topic_name = "/" + turtle_name_ + "/pose";
    turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        topic_name, 10, std::bind(&TurtleTfBroadcaster::handle_turtle_pose, this, std::placeholders::_1));

}

void TurtleTfBroadcaster::handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg) {
    geometry_msgs::msg::TransformStamped tf_stamped;

    tf_stamped.header.stamp = this->now();
    tf_stamped.header.frame_id = "world";
    tf_stamped.child_frame_id = this->turtle_name_;

    tf_stamped.transform.translation.x = msg->x;
    tf_stamped.transform.translation.y = msg->y;
    tf_stamped.transform.translation.z = 0.0;

    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 0, msg->theta);
    
    tf_stamped.transform.rotation.x = tf_q.x();
    tf_stamped.transform.rotation.y = tf_q.y();
    tf_stamped.transform.rotation.z = tf_q.z();
    tf_stamped.transform.rotation.w = tf_q.w();
    
    this->tf_broadcaster_->sendTransform(tf_stamped);
}

} // namespace learning_tf2_cpp