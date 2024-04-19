import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray


class LOGGING_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # Final Data Subscriber
    def init_final_data_subscriber(self, topic):
        self.final_subscriber = self.create_subscription(
            rosarray, topic, self.receive_final_data, 10
        )
        self.final_subscriber  # prevent unused variable warning
        self.final_sub_data = None

    def receive_final_data(self, msg):
        self.final_sub_data = msg.data


def main(args=None):
    rclpy.init(args=args)

    logging_node = LOGGING_NODE("logging_node")
    logging_node.init_final_data_subscriber("final_data")

    while rclpy.ok():
        rclpy.spin_once(logging_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    logging_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

