import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from robot_interfaces.action import CountUpTo

class CountUpToClient(Node):
    def __init__(self):
        super().__init__("count_up_to")

        self.count_up_to_client_ = ActionClient(
            self, CountUpTo, "/count_up_to")
        
    def send_goal(self, target_number, delay):
        self.count_up_to_client_.wait_for_server()
        goal = CountUpTo.Goal()
        goal.target_number = target_number
        goal.delay = delay
        self.count_up_to_client_.send_goal_async(
            goal, feedback_callback = self.goal_feedback_callback).add_done_callback(
                self.goal_response_callback)
    
    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number
        self.get_logger().info("Current number: " + str(number))
        
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(
                self.goal_result_callback)
        else:
            self.get_logger().info("Goal got rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warm("Cancelled")

        self.get_logger().info("Result: " + str(result.reached_number))

def main(args = None):
    rclpy.init(args = args)
    node = CountUpToClient()
    node.send_goal(10, 0.5)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()