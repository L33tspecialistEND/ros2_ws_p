#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class MultipleGroupsNode : public rclcpp::Node
{
    public:
        MultipleGroupsNode() : Node("mutiple_mutually_exclusive_groups_node")
        {
            // Group 1: Mutually Exclusive for sensor data processing
            sensor_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // Group 2: Mutually Exclusive for actuator commands
            actuator_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // Group 3: Reentrant for logging to console
            reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            // Timer 1: Simulate sensor data processing
            timer1_ = this->create_wall_timer(
                700ms,
                std::bind(&MultipleGroupsNode::sensor_callback, this),
                sensor_group_
            );

            // Timer 2: Simulate sending of actuator commands
            timer2_ = this->create_wall_timer(
                300ms,
                std::bind(&MultipleGroupsNode::actuator_callback, this),
                actuator_group_
            );
            RCLCPP_INFO(this->get_logger(), "[MultipleGroupsNode] started. Timer1: 0.7s. Timer2: 0.3s. Timer3: 0.1s.");

            // Timer 3: Simulate logging to the console
            timer3_ = this->create_wall_timer(
                100ms,
                std::bind(&MultipleGroupsNode::logging_callback, this),
                reentrant_group_
            );
            RCLCPP_INFO(this->get_logger(), "[MultipleGroupsNode] started. Timer1: 0.7s. Timer2: 0.3s.");
        }

    private:
        rclcpp::TimerBase::SharedPtr timer1_;
        rclcpp::TimerBase::SharedPtr timer2_;
        rclcpp::TimerBase::SharedPtr timer3_;

        rclcpp::CallbackGroup::SharedPtr sensor_group_;
        rclcpp::CallbackGroup::SharedPtr actuator_group_;
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;

        void sensor_callback()
        {
            RCLCPP_INFO(this->get_logger(), "[SensorGroup] callback started.");
            // Simulate 1000ms of work
            // Imagine it is accessing shared sensor data here
            std::this_thread::sleep_for(1000ms);
            RCLCPP_INFO(this->get_logger(), "[SensorGroup] callback finished.");
        }

        void actuator_callback()
        {
            RCLCPP_INFO(this->get_logger(), "[ActuatorGroup] callback started.");
            // Simulate 200ms of work
            // Imagine it is accessing shared actuator state here
            std::this_thread::sleep_for(200ms);
            RCLCPP_INFO(this->get_logger(), "[ActuatorGroup] callback finished.");
        }

        void logging_callback()
        {
            RCLCPP_INFO(this->get_logger(), "[ReentrantGroup] callback started.");
            // Simulate 100ms of work
            // Imagine it is logging to the console here
            std::this_thread::sleep_for(100ms);
            RCLCPP_INFO(this->get_logger(), "[ReentrantGroup] callback finished.");
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultipleGroupsNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    RCLCPP_INFO(rclcpp::get_logger("main_logger"), "[MultipleGroupsNode] shutting down.");
    rclcpp::shutdown();

    return 0;
}