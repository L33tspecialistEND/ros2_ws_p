#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <chrono>
#include <memory>
#include <future>

using namespace std::chrono_literals;

class MyClient : public rclcpp::Node
{
    public:
        MyClient() : Node("my_client")
        {
            client_ = this->create_client<example_interfaces::srv::AddTwoInts>("my_client");
        }

        void send_request(int num1, int num2)
        {
            while(!client_->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting...");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Waiting for service...");
            }

            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = num1;
            request->b = num2;

            RCLCPP_INFO(this->get_logger(), "Sending request: a = %d, b = %d", num1, num2);

            auto result = client_->async_send_request(request);

            if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 3s) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get();
                RCLCPP_INFO(this->get_logger(), "Sum: %ld", response->sum);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Couldn't call the service.");
            }
        }

    private:
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    if(argc != 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("main_logger"), "Usage: my_client X Y");   
        return 1;     
    }

    auto node = std::make_shared<MyClient>();
    node->send_request(std::stoi(argv[1]), std::stoi(argv[2]));

    rclcpp::shutdown();

    return 0;
}