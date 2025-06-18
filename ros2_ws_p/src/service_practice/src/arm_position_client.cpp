#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/set_arm_position.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class ArmPositionClient : public rclcpp::Node
{
    public:
        ArmPositionClient() : Node("arm_position_client")
        {
            client_ = this->create_client<robot_interfaces::srv::SetArmPosition>("/set_arm_position");
        }

        void send_request(float x, float y, float z)
        {
            while(!client_->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for service. Exiting...");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Waiting for service...");
            }

            auto request = std::make_shared<robot_interfaces::srv::SetArmPosition::Request>();
            request->x = x;
            request->y = y;
            request->z = z;

            RCLCPP_INFO(this->get_logger(), "Sending request: x = %f, y = %f, z = %f", x, y, z);
            auto result = client_->async_send_request(request);

            if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 2s) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get();
                RCLCPP_INFO(this->get_logger(), "Response: %s. %s",
                    response->success ? "SUCCESS" : "FAILURE", response->message.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Service call failed or timed out.");
            }
        }

    private:
        rclcpp::Client<robot_interfaces::srv::SetArmPosition>::SharedPtr client_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ArmPositionClient>();

    if(argc != 4)
    {
        RCLCPP_INFO(rclcpp::get_logger("main_logger"), "Usage: arm_position_client X Y Z");
        return -1;
    }
    node->send_request(std::stof(argv[1]), std::stof(argv[2]), std::stof(argv[3]));
    rclcpp::shutdown();
    
    return 0;
}