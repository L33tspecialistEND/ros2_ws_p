#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <memory>

class MyServer : public rclcpp::Node
{
    public:
        MyServer() : Node("my_server")
        {
            service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                "my_add_two_ints",
                std::bind(&MyServer::handle_request, this, std::placeholders::_1, std::placeholders::_2)
            );
            RCLCPP_INFO(this->get_logger(), "[MyServer] service node started. Awaiting request...");
        }

    private:
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;

        void handle_request(
            const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
            example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
            {
                RCLCPP_INFO(this->get_logger(), "Incoming request parameters: %ld, %ld.", request->a, request->b);
                response->sum = request->a + request->b;
                RCLCPP_INFO(this->get_logger(), "Sending response. The sum is: %ld.", response->sum);
            }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}