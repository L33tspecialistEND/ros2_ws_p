#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <memory>

class AddTwoIntsServer : public rclcpp::Node
{
    public:
        AddTwoIntsServer() : Node("add_two_ints_server")
        {
            service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                "add_two_ints",
            std::bind(&AddTwoIntsServer::handle_request, this, std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(this->get_logger(), "[AddTwoIntsServer] Node started. Waiting for request...");
        }

    private:
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;

        void handle_request(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
        {
            RCLCPP_INFO(this->get_logger(), "Incoming request: Add %ld and %ld.", request->a, request->b);
            response->sum = request->a + request->b;
            RCLCPP_INFO(this->get_logger(), "Sending response. Sum: %ld.", response->sum);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}