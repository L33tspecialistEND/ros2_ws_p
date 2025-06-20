#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/count_until.hpp"

#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

class CountUntilClient : public rclcpp::Node
{
    public:
        // Constructor for CountUntilClient Node
        CountUntilClient() : Node("count_until_client")
        {
            // Create an action client of type robot_interfaces::action::CountUntil
            this->action_client_ = rclcpp_action::create_client<robot_interfaces::action::CountUntil>(
                this,
                "/count_until"
            );

            RCLCPP_INFO(this->get_logger(), "[CountUntilClient] Node started.");
        }

        // Method to send goal to action server
        void send_goal(int64_t target_count)
        {
            // Check if the action server is available
            if(!this->action_client_->wait_for_action_server(10s))
            {
                RCLCPP_INFO(this->get_logger(), "Action server not available after 10 seconds!");
                rclcpp::shutdown();
                return;
            }

            // Create a goal message
            auto goal_msg = robot_interfaces::action::CountUntil::Goal();
            goal_msg.target_count = target_count;

            RCLCPP_INFO(this->get_logger(), "Sending goal: Count until %ld", target_count);

            // Configure options for sending the goal (feedback and result callbacks)
            auto send_goal_options = rclcpp_action::Client<robot_interfaces::action::CountUntil>::SendGoalOptions();
            send_goal_options.feedback_callback = std::bind(
                &CountUntilClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback = std::bind(
                &CountUntilClient::result_callback, this, std::placeholders::_1);

            // Send the goal to the action server
            this->action_client_->async_send_goal(goal_msg, send_goal_options);
        }

    private:
        rclcpp_action::Client<robot_interfaces::action::CountUntil>::SharedPtr action_client_;

        // Callback function for processing feedback
        void feedback_callback(
            rclcpp_action::ClientGoalHandle<robot_interfaces::action::CountUntil>::SharedPtr goal_handle,
            const std::shared_ptr<const robot_interfaces::action::CountUntil::Feedback> feedback)
        {
            // Safety check
            if(!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Received feedback for unknown goal.");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Feedback: Current count: %ld.", feedback->current_count);
        }

        // Callback function for processing the final result
        void result_callback(
            const rclcpp_action::ClientGoalHandle<robot_interfaces::action::CountUntil>::WrappedResult& result)
        {
            switch(result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal SUCCEEDED.");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal ABORTED.");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Goal CANCELED.");
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
            }

            // Access and print the result details
            if(result.result)
            {
                RCLCPP_INFO(this->get_logger(), "Final result: %ld. Success: %s",
                                result.result->final_count,
                                result.result->success ? "true" : "false");
            }
            rclcpp::shutdown();     // Shutdown after receiving the result
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClient>();
    
    // Send a goal to count until 15
    node->send_goal(15);

    // Spin the node to process callbacks (feedback and result)
    rclcpp::spin(node);
    
    rclcpp::shutdown();

    return 0;
}