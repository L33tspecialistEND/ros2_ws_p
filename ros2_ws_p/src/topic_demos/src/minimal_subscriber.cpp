#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <functional>       // For std::bind and std::placeholders
#include <memory>           // For std::make_shared

class MinimalSubscriber : public rclcpp::Node
{
    public:
        MinimalSubscriber() : Node("minimal_subscriber")
        {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "/topic_publisher",
                10,
                std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "[MinimalSubscriber] Node started. Subscribing "
                "to topic: /topic_publisher");
        }

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->data.c_str());
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();

    rclcpp::spin(node);
    
    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "[MinimalSubscriber] Node shutting down.");
    rclcpp::shutdown();

    return 0;
}