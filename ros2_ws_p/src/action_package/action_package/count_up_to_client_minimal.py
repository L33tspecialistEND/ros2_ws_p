import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from robot_interfaces.action import CountUpTo

class CountUpToClient(Node):
    def __init__(self):
        super().__init__("count_up_to")

def main(args = None):
    rclpy.init(args = args)
    node = CountUpToClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()