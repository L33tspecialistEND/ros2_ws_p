#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/set_arm_position.hpp"    // This interface has not yet been created
#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class RobotArmServer : public rclcpp::Node
{
    public:
        RobotArmServer() : Node("robot_arm_server")
        {
            service_ = this->create_service<robot_interfaces::srv::SetArmPosition>(
                "/set_arm_position",
                std::bind(&RobotArmServer::robot_arm_request, this, std::placeholders::_1, std::placeholders::_2)
            );
            RCLCPP_INFO(this->get_logger(), "[RobotArmServer] service node started. Awaiting request...");
        }

    private:
        rclcpp::Service<robot_interfaces::srv::SetArmPosition>::SharedPtr service_;

        void robot_arm_request(
            const robot_interfaces::srv::SetArmPosition::Request::SharedPtr request,
        robot_interfaces::srv::SetArmPosition::Response::SharedPtr response)
        {
            RCLCPP_INFO(this->get_logger(), "Incoming request. Coordinates: x = %f, y = %f, z = %f.",
                        request->x, request->y, request->z);
            // Simulating work
            std::this_thread::sleep_for(1000ms);

            RCLCPP_INFO(this->get_logger(), "Sending response.");
            response->success = true;
            response->message = "Coordinates set.";
        }
}; 

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}