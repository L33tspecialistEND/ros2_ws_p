#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class TwoTimerNode : public rclcpp::Node
{
    public:
        TwoTimerNode() : Node("single_threaded_two_callbacks_node")
        {
            timer1_ = this->create_wall_timer(
                500ms,
                std::bind(&TwoTimerNode::timer_callback1, this)
            );
            timer2_ = this->create_wall_timer(
                1000ms,
                std::bind(&TwoTimerNode::timer_callback2, this)
            );

            RCLCPP_INFO(this->get_logger(), "TwoTimerNode started! Timer1: 0.5s, Timer2: 1.0s");
        }

    private:
        rclcpp::TimerBase::SharedPtr timer1_;
        rclcpp::TimerBase::SharedPtr timer2_;

        void timer_callback1()
        {
            RCLCPP_INFO(this->get_logger(), "[Timer1] callback started! Now simulating 300ms of work.");
            std::this_thread::sleep_for(300ms);
            RCLCPP_INFO(this->get_logger(), "[Timer1] callback finished!");
        }

        void timer_callback2()
        {
            RCLCPP_INFO(this->get_logger(), "[Timer2] callback started!");
            RCLCPP_INFO(this->get_logger(), "[Timer2] callback finished!");
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwoTimerNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(node->get_logger(), "TwoTimerNode is shutting down.");
    rclcpp::shutdown();

    return 0;
}