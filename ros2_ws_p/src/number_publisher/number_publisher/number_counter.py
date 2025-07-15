import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberCounter(Node):
    def __init__(self):
        super().__init__("number_counter")

def main(args = None):
    rclpy.init(args = args)
    node = NumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()