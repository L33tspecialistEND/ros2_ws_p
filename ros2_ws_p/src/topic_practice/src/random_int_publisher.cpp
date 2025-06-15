#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>       // For std::chrono_literals
#include <memory>       // For std::make_shared
#include <functional>   // For std::bind
#include "../include/topic_practice/Random.h"

using namespace std::chrono_literals;

class RandomIntPublisher : public rclcpp::Node
{
    public:
        RandomIntPublisher() : Node("random_int_publisher"), random_num_(0)
        {
            publisher_ = this->create_publisher<std_msgs::msg::Int32>(
                "/random_numbers",
                10
            );
            timer_ = this->create_wall_timer(
                200ms,
                std::bind(&RandomIntPublisher::timer_callback, this)
            );
            RCLCPP_INFO(this->get_logger(), "[RandomIntPublisher] Node started. Timer: 0.2s.");
        }

    private:
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        int32_t random_num_;

        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "[Timer] callback started.");
            srand(time(0));
            random_num_ = Random::get(0, 100);
            auto message = std_msgs::msg::Int32();
            message.data = random_num_;
            RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
            publisher_->publish(message);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomIntPublisher>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "[RandomIntPublisher] Node shutting down.");
    rclcpp::shutdown();

    return 0;
}