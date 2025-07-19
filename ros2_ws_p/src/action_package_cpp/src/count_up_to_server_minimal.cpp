#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/count_up_to.hpp"

class CountUpToServer : public rclcpp::Node
{
    public:
        CountUpToServer() : Node("count_up_to_server")
        {
            RCLCPP_INFO(this->get_logger(), "[CountUpToServer] Node started.");
        }

    private:

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUpToServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}