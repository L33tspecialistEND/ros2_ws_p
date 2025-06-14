#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("explicit_single_threaded_node");
    RCLCPP_INFO(node->get_logger(), "Explicit Single-Threaded Node has started!");

    // Instead of rclcpp::spin(), this explicitly creates a single-threaded executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // The node to be run must be added to the executor. I guess this is so it 
    // knows what to run
    executor.add_node(node);

    // Then the node is spun and since it is single-threaded, main() is blocked
    // until the node is shutdown
    executor.spin();

    RCLCPP_INFO(node->get_logger(), "Explicit Single-Threaded Node is shutting down");
    rclcpp::shutdown();

    return 0;
}