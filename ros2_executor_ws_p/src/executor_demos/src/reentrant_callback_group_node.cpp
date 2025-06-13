#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class ReentrantNode : public rclcpp::Node
{
    public:
        ReentrantNode() : Node("reentrant_node")
        {
            // Create a Reentrant callback group
            reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            // Timer1
            timer1_ = this->create_wall_timer(
                1000ms,
                std::bind(&ReentrantNode::timer_callback1, this),
                reentrant_group_
            );

            // Timer2
            timer2_ = this->create_wall_timer(
                500ms,
                std::bind(&ReentrantNode::timer_callback2, this),
                reentrant_group_
            );

            RCLCPP_INFO(this->get_logger(), "[ReentrantNode] started. Timer1: 1.0s. Timer2: 0.5s.");
        }

    private:
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;
        rclcpp::TimerBase::SharedPtr timer1_;
        rclcpp::TimerBase::SharedPtr timer2_;
        

        void timer_callback1()
        {
            RCLCPP_INFO(this->get_logger(), "[Timer1] callback started. Simulating 1200ms of work.");
            std::this_thread::sleep_for(1200ms);
            RCLCPP_INFO(this->get_logger(), "[Timer1] callback finished.");
        }

        void timer_callback2()
        {
            RCLCPP_INFO(this->get_logger(), "[Timer2] callback started. Simulating brief work.");
            std::this_thread::sleep_for(100ms);
            RCLCPP_INFO(this->get_logger(), "[Timer2] callback finished.");
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReentrantNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "[ReentrantNode] shutting down.");
    rclcpp::shutdown();

    return 0;
}