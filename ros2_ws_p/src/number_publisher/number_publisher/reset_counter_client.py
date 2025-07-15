import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ResetCounter

class ResetCounterClient(Node):
    def __init__(self):
        super().__init__("reset_counter_client")
    
def main(args = None):
    rclpy.init(args = args)
    node = ResetCounterClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()