#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/robot_status.hpp"
#include <functional>
#include <memory>

class RobotStatusSubscriber : public rclcpp::Node
{
    public:
        RobotStatusSubscriber() : Node("robot_status_subscriber")
        {
            subscription_ = this->create_subscription<robot_interfaces::msg::RobotStatus>(
                "/my_robot_status",
                10,
                std::bind(&RobotStatusSubscriber::topic_callback, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "[RobotStatusSubscriber] Node started.");
        }

    private:
        rclcpp::Subscription<robot_interfaces::msg::RobotStatus>::SharedPtr subscription_;

        void topic_callback(const robot_interfaces::msg::RobotStatus::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Battery voltage: %f", msg->battery_voltage);
            RCLCPP_INFO(this->get_logger(), "Mode: Moving" );
            RCLCPP_INFO(this->get_logger(), "Status message: %s", msg->status_message.c_str());
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStatusSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();    

    return 0;
}