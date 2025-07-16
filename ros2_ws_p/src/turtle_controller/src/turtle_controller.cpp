#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "robot_interfaces/srv/reset_counter.hpp"
#include "robot_interfaces/srv/activate_turtle.hpp"
#include <functional>
#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;
using SetPen = turtlesim::srv::SetPen;
using ActivateTurtle = robot_interfaces::srv::ActivateTurtle;


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
            turtle_pen_client_ = create_client<SetPen>("/turtle1/set_pen");
            activate_turtle_server_ = create_service<ActivateTurtle>(
                "/activate_turtle",
                std::bind(&TurtleController::callback_activate_turtle, this, _1, _2)
            );

            on_right_side_ = true;

            RCLCPP_INFO(this->get_logger(), "[TurtleController] Node started.");
        }

        void set_pen_request(uint8_t r, uint8_t g, uint8_t b, uint8_t width)
        {
            while(!turtle_pen_client_->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting...");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Waiting for service...");
            }
            auto request = std::make_shared<SetPen::Request>();
            request->r = r;
            request->g = g;
            request->b = b;
            request->width = width;

            RCLCPP_INFO(this->get_logger(), "Sending request: r: %d, g: %d, b: %d, width: %d",
                request->r, request->g, request->b, request->width);
            turtle_pen_client_->async_send_request(
                request,
                std::bind(&TurtleController::callback_set_pen_response, this, _1)
            );
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_turtle_publisher_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_subscriber_;
        rclcpp::Client<SetPen>::SharedPtr turtle_pen_client_;
        rclcpp::Service<ActivateTurtle>::SharedPtr activate_turtle_server_;
        bool on_right_side_;


        void callback_set_pen_response(rclcpp::Client<SetPen>::SharedFuture future)
        {
            auto response = future.get();
        }

        void callback_pose(const turtlesim::msg::Pose::SharedPtr msg)
        {
            auto move_in_circle = geometry_msgs::msg::Twist();

            if (msg->x > 5.5444)
            {
                move_in_circle.angular.z = 1.0;
                move_in_circle.linear.x = 1.0;
                move_turtle_publisher_->publish(move_in_circle);
                RCLCPP_INFO(this->get_logger(), "Turtle moving in the left half of the plane...");
            }
            else if(msg->x <= 5.5444)
            {
                move_in_circle.angular.z = 2.0;
                move_in_circle.linear.x = 2.0;
                move_turtle_publisher_->publish(move_in_circle);
                RCLCPP_INFO(this->get_logger(), "Turtle moving in the right half of the plane...");
            }

            if(msg->x > 5.444)
                on_right_side_ = true;
            else
                on_right_side_ = false;

            if(msg->x > 5.5444 && on_right_side_)
            {
                set_pen_request(200, 0, 0, 4);
                RCLCPP_INFO(this->get_logger(), "Setting pen to Red...");
                on_right_side_ = false;
            }
            else if(msg->x <= 5.5444 && !on_right_side_)
            {
                set_pen_request(0, 200, 0, 4);
                RCLCPP_INFO(this->get_logger(), "Setting pen to Green...");
                on_right_side_ = true;
            }
        }

        void callback_activate_turtle(
            const ActivateTurtle::Request::SharedPtr request,
            const ActivateTurtle::Response::SharedPtr response
        )
        {
            
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}