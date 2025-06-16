#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <chrono>
#include <future>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class ResponsiveServiceClient : public rclcpp::Node
{
    public:
        ResponsiveServiceClient() : Node("responsive_service_client")
        {
            // A client for the add_two_ints service
            client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

            // A timer to periodically call the service
            service_call_timer_ = this->create_wall_timer(
                5s, // Call the timer every 5s
                std::bind(&ResponsiveServiceClient::service_call_timer_callback, this)
            );

            // Another timer to show the node is still responsive
            ping_timer_ = this->create_wall_timer(
                1s,
                [this]() {
                    RCLCPP_INFO(this->get_logger(), "Node is spinning...(Ping)");
                }
            );

            RCLCPP_INFO(this->get_logger(), "[ResponsiveServiceClient] Node started.");
        }

    private:
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr service_call_timer_;
        rclcpp::TimerBase::SharedPtr ping_timer_;
        int request_counter_ = 0;

        void service_call_timer_callback()
        {
            // Check if the service is available
            if(!client_->wait_for_service(1s))
            {
                RCLCPP_WARN(this->get_logger(), "Service not available. Skipping this request.");
                return;
            }

            // Create and populate the request
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = ++request_counter_;
            request->b = 100 - request_counter_;

            RCLCPP_INFO(this->get_logger(), "Sending request (a = %ld, b = %ld) from service_call_timer_callback.", request->a, request->b);

            // Send the request asynchronously
            auto result = client_->async_send_request(request);

            // spin_until_future_complete will wait for the response
            // while allowing other callbacks (like ping_timer) to be processed by the executor.
            if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 3s) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                // Receive response successfully
                auto response = result.get();
                RCLCPP_INFO(this->get_logger(), "Received response: Sum = %ld", response->sum);
            }
            else
            {
                // Service call timed out or failed
                RCLCPP_ERROR(this->get_logger(), "Service call to 'add_two_ints' failed or timed out!");
            }
            RCLCPP_INFO(this->get_logger(), "Service call processing finished for this request.");
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResponsiveServiceClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}