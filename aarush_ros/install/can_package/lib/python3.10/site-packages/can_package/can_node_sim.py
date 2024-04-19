import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random

class CAN_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # CAN publisher
    def init_can_publisher(self, topic, timer_period):
        self.can_publisher = self.create_publisher(rosarray, topic, 10)
        self.can_timer = self.create_timer(timer_period, self.publish_can_data)
        self.can_pub_data = None

    def publish_can_data(self):
        data = [
            [0x601,23942,4.5,random.randint(1,40)],
            [0x604,52341,4.5,random.randint(1,40)]
            # [0x602,random.randint(3,5),random.randint(3,5),random.randint(3,5),random.randint(3,5)],
            # [0x603,random.randint(3,5),random.randint(3,5),random.randint(3,5),random.randint(3,5)],
            # [0x604,78322,random.randint(1,40),random.randint(1,40)],
            # [0x605,random.randint(3,5),random.randint(3,5),random.randint(3,5),random.randint(3,5)],
            # [0x606,random.randint(3,5),random.randint(3,5),random.randint(3,5),random.randint(3,5)],
            # [0x607,81923,random.randint(1,40),random.randint(1,40)],
            # [0x608,random.randint(3,5),random.randint(3,5),random.randint(3,5),random.randint(3,5)],
            # [0x609,random.randint(3,5),random.randint(3,5),random.randint(3,5),random.randint(3,5)],
            # [0x610,27492,random.randint(1,40),random.randint(1,40)],
            # [0x611,random.randint(3,5),random.randint(3,5),random.randint(3,5),random.randint(3,5)],
            # [0x612,random.randint(3,5),random.randint(3,5),random.randint(3,5),random.randint(3,5)],
            # [0x613,45621,random.randint(1,40),random.randint(1,40)],
            # [0x614,random.randint(3,5),random.randint(3,5),random.randint(3,5),random.randint(3,5)],
            # [0x615,random.randint(3,5),random.randint(3,5),random.randint(3,5),random.randint(3,5)],
            # [0x6F4,random.randint(0,100),random.randint(0,100)],
            # [0x6F5,random.randint(0,10),random.randint(0,10)],

        ]
        
        self.can_pub_data = data[random.randint(0,len(data)-1)]

        self.can_pub_msg = rosarray()
        self.can_pub_msg.data = self.can_pub_data
        if self.can_pub_data is not None:
            self.can_publisher.publish(self.can_pub_msg)
        self.can_pub_data = self.can_pub_msg.data
        print("PUB:", self.can_pub_data)

def main(args=None):
    rclpy.init(args=args)

    can_node = CAN_NODE("can_node")
    can_node.init_can_publisher("can_rx_data", 0.0001)

    while rclpy.ok():
        rclpy.spin_once(can_node)

        

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
