#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisher : public rclcpp::Node
{
    public:
        NumberPublisher() : Node("number_publisher")
        {
            this->declare_parameter("number", 2);
            this->declare_parameter("publish_period", 1.0);

            number_ = this->get_parameter("number").as_int();
            double timer_period = this->get_parameter("publish_period").as_double();
            

            number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>(
                "/number", 10);
            number_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(timer_period),
                std::bind(&NumberPublisher::publishNumber, this));

            RCLCPP_INFO(this->get_logger(), "[NumberPublisher] Node has been started.");
        }

    private:
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
        rclcpp::TimerBase::SharedPtr number_timer_;
        int64_t number_ = 2;

        void publishNumber()
        {
            auto msg = example_interfaces::msg::Int64();
            msg.data = number_;
            number_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published: '%ld'", msg.data);
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "[NumberPublisher] Node has been shut down.");

    return 0;
}