import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random
import datetime
import time
import can

can.rc['interface'] = 'socketcan_ctypes'
from can.interfaces.interface import Bus
from can import Message


class can_publisher(Node):

    def __init__(self):
        super().__init__('can_node')
        self.publisher_ = self.create_publisher(rosarray, 'can_data', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_data)
        self.latest_data = None
        self.can_interface = 'can0'
        self.bus = Bus(self.can_interface) 

    def publish_data(self):
        
        msg = rosarray()
        Message = self.bus.recv(0.0) # 2 parts - Arbitration ID and Data Array
    
        if Message:
            now = datetime.datetime.now()
            msg.data = [Message.arbitration_id,Message.data[0]]

        self.publisher_.publish(msg)
        self.latest_data = msg.data
        
    def get_latest_data(self):
        return self.latest_data


def main(args=None):
    rclpy.init(args=args)

    can_node = can_publisher()

    while rclpy.ok():
        rclpy.spin_once(can_node)
        latest_data = can_node.get_latest_data()
        if latest_data is not None:
            print(latest_data)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_node.bus.shutdown()
    can_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()