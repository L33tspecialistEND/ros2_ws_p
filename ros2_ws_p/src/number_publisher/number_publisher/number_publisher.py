import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisher(Node):
    def __init__(self):
        super.__init__("number_publisher")
        self.number_publisher_ = self.create_publisher(Int64, "/number", 10)

    def main(args = None):
        rclpy.init(args = args)
        node = NumberPublisher()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()