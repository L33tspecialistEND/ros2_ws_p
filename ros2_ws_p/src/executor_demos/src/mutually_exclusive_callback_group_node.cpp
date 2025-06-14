#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class MutuallyExclusiveNode : public rclcpp::Node
{
    public:
        MutuallyExclusiveNode() : Node("mutually_exclusive_node")
        {
            // Create a mutually exclusive callback group
            mutually_exclusive_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // Timer1
            timer1_ = this->create_wall_timer(
                1000ms,
                std::bind(&MutuallyExclusiveNode::timer_callback1, this),
                mutually_exclusive_group_
            );

            // Timer2
            timer2_ = this->create_wall_timer(
                500ms,
                std::bind(&MutuallyExclusiveNode::timer_callback2, this),
                mutually_exclusive_group_
            );

            RCLCPP_INFO(this->get_logger(), "[MutuallyExclusiveNode] started. Timer1: 1.0s. Timer2: 0.5s");
        }

    private:
        rclcpp::TimerBase::SharedPtr timer1_;
        rclcpp::TimerBase::SharedPtr timer2_;
        rclcpp::CallbackGroup::SharedPtr mutually_exclusive_group_;

        void timer_callback1()
        {
            RCLCPP_INFO(this->get_logger(), "[Timer1] callback started.");

            // Simulate 1200ms of work
            std::this_thread::sleep_for(1200ms);
            RCLCPP_INFO(this->get_logger(), "[Timer1] callback finished.");
        }

        void timer_callback2()
        {
            RCLCPP_INFO(this->get_logger(), "[Timer2] callback started.");

            // Simulate 100ms of brief work
            std::this_thread::sleep_for(100ms);
            RCLCPP_INFO(this->get_logger(), "[Timer2] callback finished.");
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MutuallyExclusiveNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "[MutuallyExclusiveNode] shutting down.");
    rclcpp::shutdown();

    return 0;
}