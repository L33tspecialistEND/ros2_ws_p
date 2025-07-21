import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from robot_interfaces.action import CountUpTo

class CountUpToServer(Node):
    def __init__(self):
        super().__init__("count_up_to")

        self.count_up_to_server = ActionServer(
            self,
            CountUpTo,
            "/count_up_to",
            goal_callback = self.goal_callback,
            execute_callback = self.execute_callback,
            callback_group = ReentrantCallbackGroup())
        
        self.get_logger().info("[CountUpTo] Node has started.")

    def goal_callback(self, goal_request: CountUpTo.Goal):
        self.get_logger().info("Received a goal")
        if goal_request.target_number <= 0:
            self.get_logger().warn("Rejecting the goal: target number must be positive.")
            return GoalResponse.REJECT
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        # Here you can implement any logic needed to handle cancellation
        # For now, we just log it and return ACCEPT
        return GoalResponse.ACCEPT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        target_number = goal_handle.request.target_number
        delay = goal_handle.request.delay
        result = CountUpTo.Result()
        feedback = CountUpTo.Feedback();
        counter = 0

        self.get_logger().info("Executing the goal")
        for i in range(target_number):
            counter += 1
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(str(counter))
            time.sleep(delay)
        
        if goal_handle.is_cancel_requested:
            self.get_logger().info("Canceling goal")
            goal_handle.canceled()
            result.reached_number = counter
            return result

        goal_handle.succeed()
        result.reached_number = counter
        return result

def main(args = None):
    rclpy.init(args = args)
    node = CountUpToServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()