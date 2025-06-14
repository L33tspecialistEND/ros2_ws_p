#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class FastTimerNode : public rclcpp::Node
{
    public:
        FastTimerNode() : Node("fast_timer_node")
        {
            timer_ = this->create_wall_timer(
                500ms,
                std::bind(&FastTimerNode::timer_callback, this)
            );
            RCLCPP_INFO(this->get_logger(), "[FastTimerNode] started. Timer set to 0.5s.");
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;

        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "[FastTimerNode] callback started!");
            // No work is done so no blocking occurs
        }
};

class SlowTimerNode : public rclcpp::Node
{
    public:
        SlowTimerNode() : Node("slow_timer_node")
        {
            timer_ = this->create_wall_timer(
                1000ms,
                std::bind(&SlowTimerNode::timer_callback, this)
            );
            RCLCPP_INFO(this->get_logger(), "[SlowTimerNode] started. Timer set to 1.0s.");
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;

        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "[SlowTimerNode] callback started!");

            // Simulate working, which will block the thread this callback 
            // runs on
            std::this_thread::sleep_for(700ms);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto fast_node = std::make_shared<FastTimerNode>();
    auto slow_node = std::make_shared<SlowTimerNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(fast_node);
    executor.add_node(slow_node);

    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "Starting MultiThreadedExecutor with two nodes.");
    executor.spin();

    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "MultiThreadedExecutor shutting down...");
    rclcpp::shutdown();

    return 0;
}