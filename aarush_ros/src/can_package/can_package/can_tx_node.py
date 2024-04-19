import rclpy
from rclpy.node import Node
import struct
import can
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
        self.bus = can.interface.Bus('can0', bustype='socketcan', bitrate=500000)
        self.can_sub_data = None

    def receive_can_data(self, msg):
        self.can_sub_data = msg.data

    def send_two_floats(self, can_id, float1, float2):
        try:
            # Pack the two floats into bytes (little-endian format)
            data_bytes = struct.pack('<ff', float1, float2)

            # Create a CAN message with the specified ID and data
            message = can.Message(arbitration_id=can_id, data=data_bytes, extended_id=False,dlc=8)

            # Send the CAN message
            self.bus.send(message)
            print(f"Sent CAN message: {message}")

        except Exception as e:
            print(f"Error: {e}")

    
    def send_four_int(self,message_id, data1, data2, data3, data4):
        try:
            # Create a CAN message
            message = can.Message(arbitration_id=message_id, data=[data1 & 0xFF, (data1 >> 8) & 0xFF,
                                                                data2 & 0xFF, (data2 >> 8) & 0xFF,
                                                                data3 & 0xFF, (data3 >> 8) & 0xFF,
                                                                data4 & 0xFF, (data4 >> 8) & 0xFF],
                                dlc=8)

            # Send the CAN message
            self.bus.send(message)
            print(f"Sent CAN message with ID {message_id}")

        except Exception as e:
            print(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)

    can_node = CAN_NODE("can_tx_node")
    can_node.init_can_subscriber("can_tx_data")

    while rclpy.ok():
        rclpy.spin_once(can_node)

        if can_node.can_sub_data is not None:
            print(can_node.can_sub_data) 
            control_flags = can_node.can_sub_data

            for data in control_flags:
                if data in range(6,11) or data in range(16,102) or data == 103 or data == 105 or data == 106:
                    can_node.send_four_int(0x505, 0x0000, 0x0000, 0x0000, 0x0000)
                     
                     

                if data in range(107,111) or data in range(113,115) or data == 116 or data == 118 or data in range(121,124):
                    can_node.send_four_int(0x104, 1, 1, 1, 1)

                if data == 111 or data == 125:
                    can_node.send_two_floats(0x530, 0, 0)

                if data in range(119,121):
                    can_node.send_four_int(0x505, 0x0000, 0x0000, 0x0000, 0x0000)

                #MPPT 1 datas

                if data in range(128,132) or data == 142 or data == 143 or data == 145 or data == 149:
                    can_node.send_four_int(0x108, 1, 1, 1, 1)

                #MPPT 2 data
                    
                if data in range(151,155) or data == 165 or data == 166 or data == 168 or data == 172:
                    can_node.send_four_int(0x108, 1, 1, 1, 1)

                #MPPT 3 errror

                if data in range(174,178) or data == 188 or data == 189 or data == 191 or data == 195:
                    can_node.send_four_int(0x108, 1, 1, 1, 1)

                #MPPT 4 data

                if data in range(197,201) or data == 211 or data == 212 or data == 214 or data == 218:
                    can_node.send_four_int(0x108, 1, 1, 1, 1)

                if data in range(1,6) or data in range(11,16):
                    can_node.send_four_int(0x124, 50, 50, 50, 0)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

