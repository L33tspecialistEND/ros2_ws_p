#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp> // Added for SetParametersResult
#include <functional>

using namespace std::placeholders;

class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher() : Node("number_publisher")
    {
        this->declare_parameter("number", 2);
        this->declare_parameter("publish_period", 1.0);

        number_ = this->get_parameter("number").as_int();
        double timer_period = this->get_parameter("publish_period").as_double();

        // Validate timer period
        if (timer_period <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid publish_period: %f. Using default 1.0s", timer_period);
            timer_period = 1.0;
        }

        number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("/number", 10);
        number_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period),
            std::bind(&NumberPublisher::publishNumber, this));
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&NumberPublisher::parametersCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "[NumberPublisher] Node has been started.");
    }

private:
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
    rclcpp::TimerBase::SharedPtr number_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    int64_t number_ = 2;

    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        number_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published: '%ld'", msg.data);
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "number") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                    number_ = param.as_int();
                    RCLCPP_INFO(this->get_logger(), "Updated number to: %ld", number_);
                } else {
                    result.successful = false;
                    result.reason = "Parameter 'number' must be an integer";
                    RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                }
            }
        }
        return result;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "[NumberPublisher] Node has been shut down.");
    rclcpp::shutdown();

    return 0;
}