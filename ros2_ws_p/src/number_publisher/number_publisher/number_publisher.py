import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisher(Node):
    def __init__(self):
        super().__init__("num_pub")
        self.number_ = 5
        self.number_publisher_ = self.create_publisher(Int64, "/number", 10)
        self.number_timer_ = self.create_timer(1.0, self.publish_number)
        self.get_logger().info("Number Publisher has been started.")

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)

def main(args = None):
        rclpy.init(args = args)
        node = NumberPublisher()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
        main()