#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>
#include <future>   // For std::promise and std::future

using namespace std::chrono_literals;

// Node for executor 1
class Executor1Node : public rclcpp::Node
{
    public:
        Executor1Node() : Node("executor1_node")
        {
            timer_ = this->create_wall_timer(
                500ms,
                std::bind(&Executor1Node::timer_callback, this)
            );
            RCLCPP_INFO(this->get_logger(), "[Executor1Node] Node started. Timer: 0.5s.");
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;

        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "[Executor1Node] Callback started.");

            // Simulate a brief blocking task
            std::this_thread::sleep_for(100ms);
            RCLCPP_INFO(this->get_logger(), "[Executor1Node] Callback finished.");
        }

};

// Node for executor 2
class Executor2Node : public rclcpp::Node
{
    public:
        Executor2Node() : Node("executor2_node")
        {
            timer_ = this->create_wall_timer(
                1000ms,
                std::bind(&Executor2Node::timer_callback, this)
            );
            RCLCPP_INFO(this->get_logger(), "[Executor2Node] started. Timer: 1.0s.");
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;

        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "[Executor2Node] Callback started.");

            // Simulate a longer blocking task
            std::this_thread::sleep_for(800ms);
            RCLCPP_INFO(this->get_logger(), "[Executor2Node] Callback finished.");
        }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node1 = std::make_shared<Executor1Node>();
    auto node2 = std::make_shared<Executor2Node>();

    rclcpp::executors::SingleThreadedExecutor executor1;
    rclcpp::executors::SingleThreadedExecutor executor2;

    executor1.add_node(node1);
    executor2.add_node(node2);

    // Promise and future for graceful shutdown
    std::promise<void> exit_signal;
    std::shared_future<void> future_obj =  exit_signal.get_future();

    // Spin executor1 in a separate thread
    std::thread t1([&]()
    {
        RCLCPP_INFO(rclcpp::get_logger("main_logger"), "Spinning Executor1 (separate thread)...");
        executor1.spin();
        RCLCPP_INFO(rclcpp::get_logger("separate_logger"), "Executor1 thread finished spinning.");
    });

    // Spin executor2 until the future completes
    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "Spinning Executor2 (main thread)...");
    executor2.spin_until_future_complete(future_obj);
    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "Executor2 (main thread) finished spinning.");

    t1.join();

    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "All executors done. Shutting down ROS.");
    rclcpp::shutdown();

    return 0;
}