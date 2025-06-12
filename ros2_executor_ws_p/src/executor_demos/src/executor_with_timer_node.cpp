#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class TimerNode : public rclcpp::Node
{
    public:
        TimerNode(): Node("executor_with_timer_node")
        {
            timer_ = this->create_wall_timer(500ms, std::bind(&TimerNode::timer_callback, this));
            RCLCPP_INFO(this->get_logger(), "Executor With Timer Node has started!");
        }

    private:
        void timer_callback()
        {
            std::this_thread::sleep_for(200ms);

            RCLCPP_INFO(this->get_logger(), "Timer callback executed!");
        }

        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TimerNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(node->get_logger(), "Executor With Timer Node is shutting down");
    rclcpp::shutdown();

    return 0;
}