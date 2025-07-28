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
            this->declare_parameter("turtle_velocity", 1.0);
            this->declare_parameter("color_1", std::vector<int64_t>{255, 0, 0});
            this->declare_parameter("color_2", std::vector<int64_t>{0, 255, 0});

            turtle_velocity_ = this->get_parameter("turtle_velocity").as_double();
            color_1_ = this->get_parameter("color_1").as_integer_array();
            color_2_ = this->get_parameter("color_2").as_integer_array();

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
            turtle_activated_state_ = false;

            RCLCPP_INFO(this->get_logger(), "[TurtleController] Node started.");
        }

        void set_pen_request(std::vector<int64_t> color)
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
            request->r = color[0];
            request->g = color[1];
            request->b = color[2];

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
        bool turtle_activated_state_;
        double turtle_velocity_;
        std::vector<int64_t> color_1_;
        std::vector<int64_t> color_2_;

        void callback_set_pen_response(rclcpp::Client<SetPen>::SharedFuture future)
        {
            auto response = future.get();
        }

        void callback_pose(const turtlesim::msg::Pose::SharedPtr msg)
        {
            auto move_in_circle = geometry_msgs::msg::Twist();

            if(turtle_activated_state_)
            {
                if (msg->x > 5.5444)
                {
                    move_in_circle.angular.z = turtle_velocity_;
                    move_in_circle.linear.x = turtle_velocity_;
                    move_turtle_publisher_->publish(move_in_circle);
                    RCLCPP_INFO(this->get_logger(), "Turtle moving in the left half of the plane...");
                }
                else if(msg->x <= 5.5444)
                {
                    move_in_circle.angular.z = turtle_velocity_ * 2;
                    move_in_circle.linear.x = turtle_velocity_ * 2;
                    move_turtle_publisher_->publish(move_in_circle);
                    RCLCPP_INFO(this->get_logger(), "Turtle moving in the right half of the plane...");
                }

                if(msg->x > 5.5444 && on_right_side_)
                {
                    set_pen_request(color_1_);
                    RCLCPP_INFO(this->get_logger(), "Setting pen color...");
                on_right_side_ = false;
                }
                else if(msg->x <= 5.5444 && !on_right_side_)
                {
                    set_pen_request(color_2_);
                    RCLCPP_INFO(this->get_logger(), "Setting pen to Green...");
                    on_right_side_ = true;
                }
            }
        }

        void callback_activate_turtle(
            const ActivateTurtle::Request::SharedPtr request,
            const ActivateTurtle::Response::SharedPtr response
        )
        {
            if(turtle_activated_state_)
            {
                if(request->activate)
                {
                    RCLCPP_WARN(this->get_logger(), "Turtle already activated.");
                    response->success = false;
                    response->message = "Turtle already activated.";
                }
                else if(!request->activate)
                {
                    RCLCPP_INFO(this->get_logger(), "Turtle deactivating....");
                    turtle_activated_state_ = request->activate;
                    response->success = true;
                    response->message = "Turtle deactivated.";
                }
            }
            else if(!turtle_activated_state_)
            {
                if(!request->activate)
                {
                    RCLCPP_WARN(this->get_logger(), "Turtle already deactivated.");
                    response->success = false;
                    response->message = "Turtle already deactivated.";
                }
                else if(request->activate)
                {
                    RCLCPP_INFO(this->get_logger(), "Turtle activating....");
                    turtle_activated_state_ = request->activate;
                    response->success = true;
                    response->message = "Turtle activated.";
                }
            }
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