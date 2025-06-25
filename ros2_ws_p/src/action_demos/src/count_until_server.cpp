#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/count_until.hpp"

#include <thread>
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

class CountUntilServer : public rclcpp::Node
{
    public:
        using CountUntil = robot_interfaces::action::CountUntil;
        using GoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
        
        CountUntilServer() : Node("count_until_server")
        {
            // Create an action server. The callbacks are necessary to handle goal requests, 
            // cancellation requests and action execution
            this->action_server_ = rclcpp_action::create_server<CountUntil>(
                this,
                "/count_until",
                std::bind(&CountUntilServer::handle_goal, this, _1, _2),
                std::bind(&CountUntilServer::handle_cancel, this, _1),
                std::bind(&CountUntilServer::handle_execute, this, _1)
            );

            RCLCPP_INFO(this->get_logger(), "[CountUntilServer] Node started. Waiting for goal requests...");
        }

    private:
        rclcpp_action::Server<CountUntil>::SharedPtr action_server_;

        // Callback to handle goal requests (decide whether to accept or request)
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const CountUntil::Goal> goal
        )
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request. Count until %ld. (Goal UUID: %s)",
                goal->target_count,
                rclcpp_action::to_string(uuid).c_str());

            // Check if the goal is negative
            if(goal->target_count < 0)
            {
                RCLCPP_WARN(this->get_logger(), "Goal rejected. Goal cannot be negative.");
                return rclcpp_action::GoalResponse::REJECT;
            }

            // Accept the goal and schedule for execution
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // Callback to handle cancellation requests
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle
        )
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal with UUID: %s",
                    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());

            // For simplicity, we accept all cancellations.
            // In a real application, you might check if cancellation is safe/possible.
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // Callback to handle execution
        void handle_execute(
            const std::shared_ptr<GoalHandle> goal_handle
        )
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal with UUID: %s.",
                rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());

            rclcpp::Rate loop_rate(500ms);

            auto feedback_msg = std::make_shared<CountUntil::Feedback>();
            auto result_msg = std::make_shared<CountUntil::Result>();

            const int64_t target_count = goal_handle->get_goal()->target_count;
            int64_t current_count{};

            for(current_count = 0; current_count < target_count; ++current_count)
            {
                // Check if the goal has been cancelled
                if(goal_handle->is_canceling())
                {
                    result_msg->final_count = current_count;
                    result_msg->success = false;    // Indicates the goal was not completed
                    goal_handle->canceled(result_msg);  // Indicate goal as canceled

                    RCLCPP_INFO(this->get_logger(), "Goal with UUID %s was CANCELED at count %ld.",
                        rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(), current_count);
                    return;
                }

                // Publish feedback
                feedback_msg->current_count = current_count;
                goal_handle->publish_feedback(feedback_msg);

                RCLCPP_INFO(this->get_logger(), "Publishing feedback for goal %s: Current count %ld",
                    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
                    feedback_msg->current_count);

                loop_rate.sleep();
            }
            // After loop, if not cancelled, assume counting is complete
            result_msg->final_count = current_count;
            result_msg->success = true;
            goal_handle->succeed(result_msg); // Mark goal as succeeded
            RCLCPP_INFO(this->get_logger(), "Goal with UUID %s SUCCEEDED. Final count: %ld.",
                rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(), current_count);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<CountUntilServer>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}