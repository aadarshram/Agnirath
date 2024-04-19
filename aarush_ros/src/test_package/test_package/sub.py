import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray


class sample_subscriber(Node):
    def __init__(self,node):
        super().__init__(node)
        self.subscription = self.create_subscription(
            rosarray, "/control_data", self.receive_data, 10
        )
        self.subscription  # prevent unused variable warning
        self.latest_data = None

    def receive_data(self, msg):
        self.latest_data = msg.data
        print("SUB:",self.latest_data)


def main(args=None):
    rclpy.init(args=args)

    control_s_node = sample_subscriber("sample_sub")

    while rclpy.ok():
        rclpy.spin_once(control_s_node)

        # latest_sub_data = control_s_node.latest_data

        # if latest_sub_data is not None:
        #     print(latest_sub_data)
    control_s_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
