#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/robot_status.hpp"
#include "std_msgs/msg/header.hpp"
#include <string>
#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;

class RobotStatusPublisher : public rclcpp::Node
{
    public:
        RobotStatusPublisher() : Node("robot_status_publisher")
        {
            publisher_ = this->create_publisher<robot_interfaces::msg::RobotStatus>(
                "/my_robot_status",
                10
            );
            timer_ = this->create_wall_timer(
                500ms,
                std::bind(&RobotStatusPublisher::timer_callback, this)
            );
            RCLCPP_INFO(this->get_logger(), "[RobotStatusPublisher] Node started. Timer: 0.5s.");
        }


    private:
        rclcpp::Publisher<robot_interfaces::msg::RobotStatus>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        void timer_callback()
        {
            auto msg = robot_interfaces::msg::RobotStatus();

            // Populate the message
            msg.header.stamp = this->now();
            msg.battery_voltage = 12.3f;
            msg.mode = 1;
            msg.status_message = "All systems go!";
            msg.position[0] = 1.0;
            msg.position[1] = 2.0;
            msg.position[3] = 0.0;
            msg.error_codes = {101, 205};
            msg.is_acting = true;
            publisher_->publish(msg);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStatusPublisher>();
    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "[RobotStatusPublisher] Node shutting down.");
    rclcpp::shutdown();

    return 0;
}