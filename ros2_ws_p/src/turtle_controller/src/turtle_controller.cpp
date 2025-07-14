#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::placeholders;

class TurtleController : public rclcpp::Node
{
    public:
        TurtleController() : Node("turtle_controller")
        {
            move_turtle_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
                "/turtle1/cmd_vel",
                10);
            turtle_pose_subscriber_ = create_subscription<turtlesim::msg::Pose>(
                "/turtle1/pose",
                10,
                std::bind(&TurtleController::callback_pose, this, _1)
            );

            RCLCPP_INFO(this->get_logger(), "[TurtleController] Node started.");
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_turtle_publisher_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_subscriber_;

        void callback_pose(const turtlesim::msg::Pose::SharedPtr msg)
        {
            auto move_in_circle = geometry_msgs::msg::Twist();

            if (msg->x < 5.5)
            {
                move_in_circle.angular.z = 1.0;
                move_in_circle.linear.x = 1.0;
                move_turtle_publisher_->publish(move_in_circle);
                RCLCPP_INFO(this->get_logger(), "Turtle moving in the left half of the plane...");
            }
            else
            {
                move_in_circle.angular.z = 2.0;
                move_in_circle.linear.x = 2.0;
                move_turtle_publisher_->publish(move_in_circle);
                RCLCPP_INFO(this->get_logger(), "Turtle moving in the right half of the plane...");
            }
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}