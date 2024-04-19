import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random


class sample_publisher(Node):
    def __init__(self,node):
        super().__init__(node)
        self.publisher_ = self.create_publisher(rosarray,"final_data", 10)
        self.timer_period = 1 # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_data)

        self.pub_data = None

    def publish_data(self):
        msg = rosarray()
        msg.data = self.pub_data
        self.publisher_.publish(msg)
        self.pub_data = msg.data
        print("PUB:",self.pub_data)


def main(args=None):
    rclpy.init(args=args)

    sample_pub_node = sample_publisher("sample_pub")

    while rclpy.ok():
        sample_pub_node.pub_data = [
            random.randint(0, 9)
        ]

        rclpy.spin_once(sample_pub_node)
        # latest_data = sample_pub_node.latest_data
        # if latest_data is not None:
        #     print(latest_data)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sample_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
