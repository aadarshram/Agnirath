import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random

class sample_publisher_subscriber(Node):
    def __init__(self,node,timer_period):
        super().__init__(node)
        

        self.publisher_ = self.create_publisher(rosarray,"sample_data", 10)
        self.timer_period = timer_period # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_data)
        self.data = None
        self.latest_pub_data = None

        self.subscription = self.create_subscription(
            rosarray, "sample_data", self.receive_data, 10
        )
        self.subscription  # prevent unused variable warning
        self.latest_sub_data = None

    def publish_data(self):
        self.pub_msg = rosarray()
        self.pub_msg.data = self.data
        self.publisher_.publish(self.pub_msg)
        self.latest_pub_data = self.pub_msg.data
        print("PUB:",self.latest_pub_data)

    def receive_data(self, sub_msg):
        self.latest_sub_data = sub_msg.data


def main(args=None):
    rclpy.init(args=args)

    pub_sub_node = sample_publisher_subscriber("sample_node",1)

    while rclpy.ok():

        pub_sub_node.data = [
            random.randint(0, 1),
            random.randint(0, 1),
            random.randint(0, 1),
        ]

        rclpy.spin_once(pub_sub_node)

        latest_pub_data = pub_sub_node.latest_pub_data

        # if latest_pub_data is not None:
        #     print("PUB:",latest_pub_data)

        latest_sub_data = pub_sub_node.latest_sub_data

        if latest_sub_data is not None:
            print("SUB:",latest_sub_data)

        

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub_sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
