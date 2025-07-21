#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/count_up_to.hpp"
#include <functional>

using CountUpTo = robot_interfaces::action::CountUpTo;
using namespace std::placeholders;

class CountUpToClient : public rclcpp::Node
{
    public:
        CountUpToClient() : Node("count_up_to_client")
        {
            RCLCPP_INFO(this->get_logger(), "[CountUpToClient] Node started.");
        }

    private:
        rclcpp_action::Client<CountUpTo>::SharedPtr count_up_to_client_;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUpToClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}