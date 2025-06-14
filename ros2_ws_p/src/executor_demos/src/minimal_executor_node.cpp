#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("minimal_executor_node");
    RCLCPP_INFO(node->get_logger(), "Minimal Executor Node has started!");
    rclcpp::spin(node);     // Creates the default single-threaded executor
    RCLCPP_INFO(node->get_logger(), "Hello there, do you have the high ground?");
    rclcpp::shutdown();

    return 0;
}