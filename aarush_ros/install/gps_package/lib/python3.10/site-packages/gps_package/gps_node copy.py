import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import serial

# TOKEN KEY
# self.time_utc = tokens[1]
# self.latitude = tokens[2]
# self.lat_direction = tokens[3]
# self.longitude = tokens[4]
# self.lon_direction = tokens[5]
# self.fix_quality = tokens[6]
# self.num_satellites = tokens[7]
# self.hdop = tokens[8]
# self.altitude = tokens[9]
# self.altitude_unit = tokens[10]
# self.geoid_separation = tokens[11]
# self.geoid_separation_unit = tokens[12]
# self.age_of_dgps = tokens[13]
# self.dgps_reference_id = tokens[14]


class gps_publisher(Node):
    def __init__(self, port, baud_rate):
        super().__init__("gps_node")
        self.publisher_ = self.create_publisher(rosarray, "gps_data", 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.read_serial_data)
        self.latest_data = None

        self.port = port
        self.baud_rate = baud_rate
        self.serial_port = None
        self.received_data = None

        # Output data variables
        self.geo_list = None

    def initialize(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1.0)
            return True
        except serial.SerialException:
            return False

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def read_serial_data(self):
        try:
            received_data = self.serial_port.read(256).decode("utf-8")
            if received_data:
                self.received_data += received_data

                sentence_end = self.received_data.find("\r\n")
                while sentence_end != -1:
                    sentence = self.received_data[:sentence_end]
                    self.received_data = self.received_data[sentence_end + 2 :]

                    self.parse_gga(sentence)

                    sentence_end = self.received_data.find("\r\n")
        except serial.SerialException as e:
            print("Error reading serial data:", str(e))

    def parse_gga(self, sentence):
        tokens = sentence.split(",")
        if len(tokens) >= 15 and tokens[0] == "$GPGGA":
            self.geo_list = [
                tokens[0],
                tokens[1],
                tokens[2],
                tokens[3],
                tokens[4],
                tokens[5],
                tokens[6],
                tokens[7],
                tokens[8],
                tokens[9],
                tokens[10],
                tokens[11],
                tokens[12],
                tokens[13],
                tokens[14],
            ]
            msg = rosarray()
            msg.data = self.geo_list
            self.publisher_.publish(msg)
            self.latest_data = msg.data

    def get_latest_data(self):
        return self.latest_data


def main(args=None):
    rclpy.init(args=args)

    gps_node = gps_publisher("/dev/ttyS1", 9600)

    if gps_node.initalize():
        print("GPS Active")
    else:
        print("GPS not working")

    while rclpy.ok():
        rclpy.spin_once(gps_node)
        latest_data = gps_node.get_latest_data()
        if latest_data is not None:
            print(latest_data)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_node.close()
    gps_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
