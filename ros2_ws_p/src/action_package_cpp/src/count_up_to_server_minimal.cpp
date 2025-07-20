#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/count_up_to.hpp"
#include <functional>

using CountUpTo = robot_interfaces::action::CountUpTo;
using namespace std::placeholders;
using CountUpToGoalHandle = rclcpp_action::ServerGoalHandle<CountUpTo>;

class CountUpToServer : public rclcpp::Node
{
    public:
        CountUpToServer() : Node("count_up_to_server")
        {
            count_up_to_server_ = rclcpp_action::create_server<CountUpTo>(
                this,
                "/count_up_to",
                std::bind(&CountUpToServer::goalCallback, this, _1, _2),
                std::bind(&CountUpToServer::cancelCallback, this, _1),
                std::bind(&CountUpToServer::executeCallback, this, _1));

            RCLCPP_INFO(this->get_logger(), "[CountUpToServer] Node started.");
        }

    private:
        rclcpp_action::Server<CountUpTo>::SharedPtr count_up_to_server_;

        rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const CountUpTo::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received a goal.");

            if(goal->target_number <= 0)
            {
                RCLCPP_WARN(this->get_logger(), "Rejecting the goal: target number must be positive.");
                return rclcpp_action::GoalResponse::REJECT;
            }

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancelCallback(
            const std::shared_ptr<CountUpToGoalHandle> goal_handle)
        {

        }

        void executeCallback(const std::shared_ptr<CountUpToGoalHandle> goal_handle)
        {
            int target_number = goal_handle->get_goal()->target_number;
            double delay = goal_handle->get_goal()->delay;
            auto result = std::make_shared<CountUpTo::Result>();
            int counter{};
            rclcpp::Rate loop_rate(1.0 / delay);

            RCLCPP_INFO(this->get_logger(), "Executing the goal");
            for(int i{}; i < target_number; ++i)
            {
                counter++;
                RCLCPP_INFO(this->get_logger(), "%d", counter);
                loop_rate.sleep();
            }

            result->reached_number = counter;
            goal_handle->succeed(result);
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUpToServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}