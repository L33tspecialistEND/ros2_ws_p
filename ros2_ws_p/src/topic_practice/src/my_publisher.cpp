#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class MyPublisher : public rclcpp::Node
{
    public:
        MyPublisher() : Node("my_publisher"), count_(0)
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>(
                "my_output_topic",
                10
            );
            timer_ = this->create_wall_timer(
                1000ms,
                std::bind(&MyPublisher::timer_callback, this)
            );
            RCLCPP_INFO(this->get_logger(), "[MyPublisher] Node started.");
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        void timer_callback()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello from my publisher! " + std::to_string(++count_);
            RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
            publisher_->publish(message);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyPublisher>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "[MyPublisher] Node shutting down.");
    rclcpp::shutdown();

    return 0;
}