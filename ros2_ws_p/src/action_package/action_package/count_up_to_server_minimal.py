import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from robot_interfaces.action import CountUpTo

class CountUpTo(Node):
    def __init__(self):
        super().__init__("/count_up_to")

        self.get_logger().info("[CountUpTo] Node has started.")

def main(args = None):
    rclpy.init(args = args)
    node = CountUpTo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()