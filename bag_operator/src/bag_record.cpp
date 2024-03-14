#include <ctime>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include "bag_operator/cxxopts.hpp"

class BagRecord : public rclcpp::Node {
public:
    BagRecord(const std::string& output_name, const std::vector<std::string>& topic_list = {}) : 
            Node("bag_record"), writer_(std::make_unique<rosbag2_cpp::Writer>()) {
        init(output_name, get_topic_msg_pair(topic_list));
    }

private:
    std::vector<std::pair<std::string, std::string>> get_topic_msg_pair(const std::vector<std::string>& topic_list) {
        bool is_all = topic_list.empty();
        std::vector<std::pair<std::string, std::string>> topic_msg_pair;     
        auto topics_and_types = this->get_topic_names_and_types();
        for (const auto &topic_info : topics_and_types) {
            topic_msg_pair.push_back(std::make_pair(topic_info.first, topic_info.second[0]));
        }

        std::vector<std::pair<std::string, std::string>> tmp_pair;
        if (is_all) {
            tmp_pair =  topic_msg_pair;
        } else {
            for (const auto &topic_name : topic_list) {
                auto iter = std::find_if(topic_msg_pair.begin(), topic_msg_pair.end(),
                                [&topic_name](const auto& pair) { return pair.first == topic_name; });
                if (iter != topic_msg_pair.end()) {
                    tmp_pair.emplace_back(*iter);
                    continue;
                }
            }
        }
        return tmp_pair;  
    }

    void init(const std::string& output_name, std::vector<std::pair<std::string, std::string>> topic_msg_pair) {
        writer_->open(output_name);
        for (const auto &topic_info : topic_msg_pair) {
            RCLCPP_INFO(this->get_logger(), "record topic list: [%s, %s]", topic_info.first.c_str(), topic_info.second.c_str());
            auto sub = this->create_generic_subscription(
                topic_info.first, topic_info.second, 10, 
                std::bind(&BagRecord::topic_callback, this, std::placeholders::_1, topic_info.first, topic_info.second), 
                rclcpp::SubscriptionOptions());
            sub_list_.push_back(sub);
        }
    }

    void topic_callback(const std::shared_ptr<rclcpp::SerializedMessage>& msg, 
                        const std::string& topic_name, const std::string& msg_type) {
        rclcpp::Time time_stamp = this->now();

        // 写入数据到bag文件，不再指定具体的消息类型
        writer_->write(msg, topic_name, msg_type, time_stamp);
    }

private:
    std::vector<rclcpp::SubscriptionBase::SharedPtr> sub_list_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // https://github.com/jarro2783/cxxopts
    cxxopts::Options options(argv[0], "parse cmd line");
    options.add_options()
        ("o,output", "Name of the bag", cxxopts::value<std::string>())
        ("t,topic", "topic list", cxxopts::value<std::vector<std::string>>())
        ("a,all", "all topic")
        ("h,help", "show help");
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        RCLCPP_INFO(rclcpp::get_logger("bag_record"), "%s", options.help().c_str());
        return 0;
    }

    if (!result.count("topic") && !result.count("all")) {
        RCLCPP_ERROR(rclcpp::get_logger("bag_record"), "please specify topic using -t or -a");
        return -1;
    }

    std::time_t current_time = std::time(nullptr);
    std::tm* time_info = std::localtime(&current_time);
    char buffer[50];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", time_info);
    std::string bag_name = "./" + std::string(buffer) + "-bag";
    if (result.count("output")) {
        bag_name = result["output"].as<std::string>();
    }

    std::shared_ptr<BagRecord> bag_record_ptr;
    if (result.count("all")) {
        bag_record_ptr = std::make_shared<BagRecord>(bag_name);
    } else if (result.count("topic")) {
        bag_record_ptr = std::make_shared<BagRecord>(bag_name, result["topic"].as<std::vector<std::string>>());
    }

    rclcpp::spin(bag_record_ptr);
    rclcpp::shutdown();

    return 0;
}