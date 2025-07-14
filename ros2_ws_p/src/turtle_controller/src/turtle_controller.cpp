#include "rclcpp/rclcpp.hpp"

class TurtleController : public rclcpp::Node
{
    public:
        TurtleController() : Node("turtle_controller")
        {
            
        }

    private:
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}