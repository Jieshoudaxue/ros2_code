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

// 小乌龟 TF 广播器的核心是通过订阅小乌龟的 Pose 信息，将 Pose 信息经过加工得到小乌龟相对 world 坐标系的 TF 坐标变换，
// 坐标变换主要分为两部分，一是平移变换，二是旋转变换，然后通过 TF 广播器将 TF 信息广播出去。
// 关于 Pose 信息，欧拉角以及四元数的理解，请参考：https://blog.csdn.net/cy1641395022/article/details/131236155
// 关于计算过程的理解，请参考：https://blog.csdn.net/cy1641395022/article/details/131236155
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