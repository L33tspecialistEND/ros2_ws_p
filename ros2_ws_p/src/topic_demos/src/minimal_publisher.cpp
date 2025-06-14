#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>   // For std::chrono_literals
#include <memory>   // For std::make_shared

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
    public:
        MinimalPublisher() : Node("minimal_publisher"), count_(0)
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>(
                "topic_publisher",
                10
            );

            timer_ = this->create_wall_timer(
                1000ms,
                std::bind(&MinimalPublisher::timer_callback, this)
            );

            RCLCPP_INFO(this->get_logger(), "[MinimalPublisher] Node started.");
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        void timer_callback()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello ROS2. This is message " + std::to_string(++count_);

            RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
            publisher_->publish(message);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}