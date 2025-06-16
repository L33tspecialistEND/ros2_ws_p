#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <chrono>
#include <future>
#include <memory>

using namespace std::chrono_literals;

class AddTwoIntsClient : public rclcpp::Node
{
    public:
        AddTwoIntsClient() : Node("add_two_ints_client")
        {
            // Create a client of type "example_interfaces::srv::AddTwoInts"
            client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        }

        // Method to send the request to the service
        void send_request(int a, int b)
        {
            // Check if the service is available before proceeding
            while(!client_->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for service. Exiting...");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Waiting for service to become available...");
            }

            // Create a request message
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;

            // Log the request
            RCLCPP_INFO(this->get_logger(), "Sending request: a = %d, b = %d", a, b);

            // Send the result asynchronously
            auto result = client_->async_send_request(request);

            // Wait for the result with a timeout
            if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 3s) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                // Process the response
                auto response = result.get();
                RCLCPP_INFO(this->get_logger(), "Sum: %ld", response->sum);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
            }
        }

    private:
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClient>();

    // Send a request to the service
    node->send_request(10, 5);
    rclcpp::shutdown();

    return 0;
}