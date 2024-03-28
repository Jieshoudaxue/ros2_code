#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "component_demo/pub_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(component_demo::PubComponent)

namespace component_demo
{

PubComponent::PubComponent(const rclcpp::NodeOptions & options)
                            : Node("pub_component", options), count_(0) {
    msg_inner_ = "从前有位程序员，名叫杰克。杰克在一家知名科技公司工作，以出神入化的编程技巧而闻名。"
                "可即便是杰克，也有他的死穴—他没有咖啡就无法编码。"
                "某天早上，杰克来到了办公室，兴冲冲地准备开始新的一天。"
                "他按下了咖啡机的开关，却发现这台终年不离不弃的老伙计竟然罢工了！"
                "杰克惊慌失措，因为他知道没有咖啡，他的大脑就像没有安装操作系统的电脑一样毫无用处。"
                "需要紧急解决方案！于是他做了什么每个程序员在遇到问题时都会做的事——Google搜索“没有咖啡如何生存”。"
                "成百上千的建议涌现在屏幕上，但都不对味。就在此时，他注意到了一篇奇怪的文章，标题是：“用代码唤醒你的大脑”。"
                "这篇文章提到了一种传说中的编程技术，可以让程序员通过编写代码来生成身体所需的能量。"
                "虽然看起来很荒谬，但杰克决定尝试一下。"
                "他迅速打开了他的编程环境，并开始编写一些非常复杂的函数。"
                "随着每一个键击，他能感觉到一股虚拟的“能量”在体内流淌。"
                "一小时后，杰克完成了他的作品，一个模拟咖啡因分子结构的程序。"
                "他按下运行键，闭上眼睛，希望奇迹发生。"
                "几秒钟后，他睁开眼睛，感觉... 完全一样。"
                "显然，代码并不能真正取代咖啡。"
                "杰克失望极了，但他还是决定去办公楼下的咖啡店买一杯咖啡来解决问题。。。。";
    pub_msg_ = create_publisher<std_msgs::msg::String>("hello_msg", 10);
    // 帧率设置为 200 hz，较高的帧率，可以比较明显的看出合并进程与分离进程负载的变化
    timer_ = create_wall_timer(std::chrono::milliseconds(5), std::bind(&PubComponent::on_timer, this));
}

void PubComponent::on_timer() {
    // 如果想使用 ros2 的 intra-process 通信，最好使用 unique_ptr，然后发送时使用std::move()转移所有权
    // 这种方式可以明确告诉 ros2 , 这个消息你拥有唯一的所有权，可以使用 intra-process 直接传递指针进行通信
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = msg_inner_ + std::to_string(++count_);
    pub_msg_->publish(std::move(msg));
}


} // namespace component_demo