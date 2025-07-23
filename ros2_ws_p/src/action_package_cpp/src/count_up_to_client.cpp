#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/count_up_to.hpp"
#include <functional>

using CountUpTo = robot_interfaces::action::CountUpTo;
using CountUpToGoalHandle = rclcpp_action::ClientGoalHandle<CountUpTo>;
using namespace std::placeholders;

class CountUpToClient : public rclcpp::Node
{
    public:
        CountUpToClient() : Node("count_up_to_client")
        {
            count_up_to_client_ = rclcpp_action::create_client<CountUpTo>(
                this,
                "/count_up_to");

            RCLCPP_INFO(this->get_logger(), "[CountUpToClient] Node started.");
        }

        void send_goal(int target_number, double delay)
        {
            count_up_to_client_->wait_for_action_server();
            auto goal = CountUpTo::Goal();
            goal.target_number = target_number;
            goal.delay = delay;
            
            auto options = rclcpp_action::Client<CountUpTo>::SendGoalOptions();
            options.goal_response_callback = std::bind(
                &CountUpToClient::goalResponseCallback, this, _1);
            options.result_callback = std::bind(
                &CountUpToClient::goalResultCallback, this, _1);
            options.feedback_callback = std::bind(
                &CountUpToClient::goalFeedbackCallback, this, _1, _2);

            count_up_to_client_->async_send_goal(goal, options);
        }
    private:
        rclcpp_action::Client<CountUpTo>::SharedPtr count_up_to_client_;

        void goalResponseCallback(const CountUpToGoalHandle::SharedPtr goal_handle)
        {
            if(!goal_handle)  
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server.");   
        }

        void cancelGoal()
        {
            RCLCPP_INFO(this->get_logger(), "Send a cancel goal request");
            count_up_to_client_->async_cancel_all_goals();
        }

        void goalResultCallback(const CountUpToGoalHandle::WrappedResult &result)
        {
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
                RCLCPP_INFO(this->get_logger(), "Success");
            else if(result.code == rclcpp_action::ResultCode::ABORTED)
                RCLCPP_ERROR(this->get_logger(), "Aborted");
            else if(result.code == rclcpp_action::ResultCode::CANCELED)
                RCLCPP_WARN(this->get_logger(), "Canceled");
        }

        void goalFeedbackCallback(
            const CountUpToGoalHandle::SharedPtr goal_handle,
            const std::shared_ptr<const CountUpTo::Feedback> feedback)
        {
            (void)goal_handle;
            int number = feedback->current_number ;
            RCLCPP_INFO(this->get_logger(), "Got feedback: %d", number);

            if(number >= 2)
                cancelGoal();
        }
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUpToClient>();
    node->send_goal(10, 1.0);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}