import rclpy
from rclpy.node import Node

class NumberPublisher(Node):
    def __init__(self):
        super.__init__("number_publisher")

    def main(args = None):
        rclpy.init(args = args)
        node = NumberPublisher()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()