import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from rclpy.parameter import Parameter

class NumberPublisher(Node):
    def __init__(self):
        super().__init__("num_pub")
        self.declare_parameter("number", 2)
        self.declare_parameter("publish_period", 1.0)
        self.number_ = self.get_parameter("number").value
        self.timer_period = self.get_parameter("publish_period").value
        self.number_publisher_ = self.create_publisher(Int64, "/number", 10)
        self.number_timer_ = self.create_timer(self.timer_period, self.publish_number)
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info("Number Publisher has been started.")

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)

    def parameters_callback(self, params: list[Parameter]):
          for param in params:
                if param.name == "number":
                      self.number_ = param.value

def main(args = None):
        rclpy.init(args = args)
        node = NumberPublisher()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
        main()