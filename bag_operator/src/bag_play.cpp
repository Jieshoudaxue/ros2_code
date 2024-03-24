#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include "rosbag2_storage/storage_options.hpp"

#include "bag_operator/cxxopts.hpp"

class BagPlay : public rclcpp::Node {
public:
    BagPlay(const std::string& bag_name, 
            int start_offset_sec,
            double play_delay_sec,
            double play_rate,
            const std::vector<std::string>& topic_list) : 
            Node("bag_play"), reader_(std::make_unique<rosbag2_cpp::Reader>()) {
        // 打开bag文件，后面将获取bag的metadata信息，以及bag的序列化数据
        reader_->open(bag_name);
        // start_offset_sec_ 是回放的起点偏移，play_rate_ 是回放的倍率（浮点数）
        start_offset_sec_ = start_offset_sec;
        play_rate_ = play_rate;
        // 根据用户指定的topic，创建发送句柄，如果不指定，默认发送所有topic
        topic_msg_pair_ = get_topic_msg_pair(topic_list);
        for (const auto &topic_info : topic_msg_pair_) {
            RCLCPP_INFO(this->get_logger(), "play topic list: [%s, %s]", topic_info.first.c_str(), topic_info.second.c_str());
            auto pub = this->create_generic_publisher(topic_info.first, topic_info.second, 10);
            pub_map_[topic_info.first] = pub;
        }
        // 获取topic的集合，用于判断是否是需要回放的topic
        topic_set_ = get_topic_set();
        // 创建定时器，并固定两秒以后开始回放，整个程序只回放一次
        timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(int(play_delay_sec * 1000.0)),
                        std::bind(&BagPlay::playback_cb, this));
        RCLCPP_INFO(this->get_logger(), "start play bag after %f sec...", play_delay_sec);
    }

private:
    void playback_cb() {
        // 取消定时器，因为只回放一次
        this->timer_->cancel();
        if (play_rate_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "play rate must be greater than 0");
            return;
        }

        // 获取bag的metadata信息
        auto metadata = reader_->get_metadata();
        // 获取距离纪元的时间长度
        auto epoch = metadata.starting_time.time_since_epoch();
        // 将时间长度转换为纳秒，注意这里使用的是int64_t，长度也足够了
        int64_t bag_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
        // 计算回放起点，在这里进行起点偏移
        int64_t play_start_ns = bag_start_ns + start_offset_sec_ * 1e9;

        // fix_delta 是当前系统时间与 bag 时间的固定差值，用于计算每条消息需要等待的时间
        // sum_delta 是消息的累计等待时间，用于计算下一条消息的等待时间
        int64_t fix_delta = ros_clock_.now().nanoseconds() - play_start_ns;
        int64_t sum_delta = 0ULL;
        while (reader_->has_next()) {
            rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

            if (msg->time_stamp >= play_start_ns) {
                // 计算当前 msg 需要等待的时间间隔，并sleep，直到时间到达
                // 这里是控制回放帧率的关键，支持调整回放倍率
                int64_t cur_msg_delta = static_cast<int64_t>(static_cast<double>(msg->time_stamp - play_start_ns) / play_rate_);
                while ((play_start_ns + cur_msg_delta + fix_delta) > ros_clock_.now().nanoseconds()) {
                    uint64_t wait_ns = cur_msg_delta - sum_delta;
                    rclcpp::Duration wait_du = rclcpp::Duration(std::chrono::nanoseconds(wait_ns));
                    rclcpp::sleep_for(wait_du.to_chrono<std::chrono::nanoseconds>());
                    sum_delta += wait_ns;
                }

                printf("[RUNNING] Bag Time: %.6f  Duration: %.6f\r", static_cast<double>(msg->time_stamp)/1e9, static_cast<double>(msg->time_stamp - play_start_ns)/1e9);
                fflush(stdout);

                // 判断是否是需要回放的topic，如果是，获取msg的序列化数据，通过pub句柄发送
                if (findTopic(msg->topic_name)) {
                    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(
                        *reinterpret_cast<rmw_serialized_message_t *>(msg->serialized_data.get())
                    );
                    pub_map_[msg->topic_name]->publish(*serialized_msg);
                }
            }

            if (!rclcpp::ok()) {
                break;
            }
        }
        printf("\n");
    }

    bool findTopic(const std::string& topic_name) {
        return topic_set_.find(topic_name) != topic_set_.end();        
    }

    std::vector<std::pair<std::string, std::string>> get_topic_msg_pair(const std::vector<std::string>& topic_list) {
        bool is_all = topic_list.empty();
        std::vector<std::pair<std::string, std::string>> topic_msg_pair;        
        auto metadata = reader_->get_metadata();
        for (const auto &topic_info : metadata.topics_with_message_count) {
            topic_msg_pair.push_back(std::make_pair(topic_info.topic_metadata.name, topic_info.topic_metadata.type));
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

    std::unordered_set<std::string> get_topic_set() {
        std::unordered_set<std::string> topic_set;

        for (const auto &topic_info : topic_msg_pair_) {
            topic_set.insert(topic_info.first);
        }

        return topic_set;  
    }

private:
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    rclcpp::Clock ros_clock_;    
    int start_offset_sec_;
    double play_rate_;
    std::vector<std::pair<std::string, std::string>> topic_msg_pair_;
    std::map<std::string, rclcpp::GenericPublisher::SharedPtr> pub_map_;
    std::unordered_set<std::string> topic_set_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // https://github.com/jarro2783/cxxopts
    cxxopts::Options options(argv[0], "parse cmd line");
    options.add_options()
        ("b,bag", "name of the bag", cxxopts::value<std::string>())
        ("s,start_offset_sec", "sec offset of start time", cxxopts::value<int>())
        ("r,rate", "player rate", cxxopts::value<double>())
        ("t,topic", "topic list", cxxopts::value<std::vector<std::string>>())
        ("h,help", "show help");
    auto result = options.parse(argc, argv);  

    if (result.count("help")) {
        RCLCPP_INFO(rclcpp::get_logger("bag_play"), "%s", options.help().c_str());
        return 0;
    }

    if (result.count("bag") == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("bag_record"), "please specify bag using -b");
        return -1;
    }

    std::string bag_name;
    int start_offset_sec = 0;
    double play_delay_sec = 2.0;
    double play_rate = 1.0;
    std::vector<std::string> topic_list;
    if (result.count("bag")) {
        bag_name = result["bag"].as<std::string>();
    }  
    if (result.count("start_offset_sec")) {
        start_offset_sec = result["start_offset_sec"].as<int>();
    }
    if (result.count("rate")) {
        play_rate = result["rate"].as<double>();
    }  
    if (result.count("topic")) {
        topic_list = result["topic"].as<std::vector<std::string>>();
    } 

    // 检查 bag是否存在
    if (access(bag_name.c_str(), F_OK)) {
        RCLCPP_ERROR(rclcpp::get_logger("bag_record"), "bag file %s not exist", bag_name.c_str());
        return -1;
    }

    rclcpp::spin(std::make_shared<BagPlay>(bag_name, start_offset_sec, play_delay_sec, play_rate, topic_list));
    rclcpp::shutdown();

    return 0;
}