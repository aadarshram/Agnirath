import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray

class CAN_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # CAN Subscriber
    def init_can_subscriber(self, topic):
        self.can_subscriber = self.create_subscription(
            rosarray, topic, self.receive_can_data, 10
        )
        self.can_subscriber  # prevent unused variable warning
        self.can_sub_data = None

    def receive_can_data(self, msg):
        self.can_sub_data = msg.data


def main(args=None):
    rclpy.init(args=args)

    can_node = CAN_NODE("can_tx_node")
    can_node.init_can_subscriber("can_tx_data")

    while rclpy.ok():
        rclpy.spin_once(can_node)

        if can_node.can_sub_data is not None:
            print(can_node.can_sub_data)        

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
