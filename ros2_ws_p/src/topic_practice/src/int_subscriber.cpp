#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <functional>       // For std::bind and std::placeholders
#include <memory>           // For std::make_shared

class IntSubscriber : public rclcpp::Node
{
    public:
        IntSubscriber() : Node("int_subscriber")
        {
            subscription_ = this->create_subscription<std_msgs::msg::Int32>(
                "/random_numbers",
                10,
                std::bind(&IntSubscriber::topic_callback, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "[IntSubscriber] Node started. Listening on "
                "topic: /random_numbers");
        }

    private:
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

        void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: %d", msg->data);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IntSubscriber>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "[IntSubscriber] Node shutting down.");
    rclcpp::shutdown();

    return 0;
}