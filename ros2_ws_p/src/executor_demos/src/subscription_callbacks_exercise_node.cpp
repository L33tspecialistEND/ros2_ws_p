#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class ImageCommandProcessor : public rclcpp::Node
{
    public:
        ImageCommandProcessor() : Node("image_command_processor")
        {
            // Callback group for image and command callbacks
            image_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            command_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // Subscription options needed to set callback group for subscribers
            rclcpp::SubscriptionOptions image_options_;
            image_options_.callback_group = image_group_;

            rclcpp::SubscriptionOptions command_options_;
            command_options_.callback_group = command_group_;

            image_ = this->create_subscription<std_msgs::msg::Int32>(
                "/image_topic", // Topic name
                10, // Queue size
                std::bind(&ImageCommandProcessor::image_callback, this, std::placeholders::_1),
                image_options_
            );

            command_ = this->create_subscription<std_msgs::msg::String>(
                "/command_topic", // Topic name
                10, // Queue size
                std::bind(&ImageCommandProcessor::command_callback, this, std::placeholders::_1),
                command_options_
            );
            RCLCPP_INFO(this->get_logger(), "[ImageCommandProcessor] Node started.");
        }

    private:
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr image_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_;

        rclcpp::CallbackGroup::SharedPtr image_group_;
        rclcpp::CallbackGroup::SharedPtr command_group_;

        void image_callback(const std_msgs::msg::Int32::ConstSharedPtr& msg)
        {
            RCLCPP_INFO(this->get_logger(), "[Image] callback running. Received: %d", msg->data);
            // Simulate 500ms of image processing
            std::this_thread::sleep_for(500ms);
        }

        void command_callback(const std_msgs::msg::String::ConstSharedPtr& msg)
        {
            RCLCPP_INFO(this->get_logger(), "[Command] callback running. Received: %s", msg->data.c_str());
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageCommandProcessor>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "[ImageCommandProcessor] shutting down.");
    rclcpp::shutdown();

    return 0;
}