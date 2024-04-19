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


class GPS_NODE(Node):
    def __init__(self, node):
        super().__init__(node)
        self.gps_node = node

    def init_gps_publisher(self, topic, timer_period, port, baud_rate):
        self.gps_pub = self.create_publisher(rosarray, topic, 10)
        self.gps_timer = self.create_timer(timer_period, self.parse_gngga)
        self.gps_pub_data = []
        self.port = port
        self.baud_rate = baud_rate
        self.serial_port = None

        try:
            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1.0)
            print("GPS is connected and working")
        except serial.SerialException:
            print("GPS is not working")

    def convert_utc_time_to_int(self,time_str):
    # Parse the time string in HHMMSS.SS format
        hh = int(time_str[:2])
        mm = int(time_str[2:4])
        ss = int(time_str[4:6])
        
        # Convert to seconds since midnight
        time_int = (hh * 3600) + (mm * 60) + ss
        return time_int

    def parse_gngga(self):
        try:
        # Open the serial port
            with serial.Serial(self.port, self.baud_rate) as ser:
                while True:
                    sentence = ser.readline().decode('utf-8').strip()
    # Check if the sentence starts with "$GNGGA"
                    if sentence.startswith("$GNGGA"):
                        data = sentence.split(',')
                        if len(data) >= 15:
                            sentence_id = data[0]
                            time = data[1]
                            latitude = data[2]
                            lat_hemisphere = data[3]
                            longitude = data[4]
                            lon_hemisphere = data[5]
                            gps_quality = int(data[6])
                            num_satellites = int(data[7])
                            hdop = float(data[8])
                            altitude = float(data[9])
                            alt_unit = data[10]
                            geoidal_sep = float(data[11])
                            geoidal_sep_unit = data[12]
                            age_diff_data = data[13]
                            ref_station_id = data[14]

                            hh = int(time[:2])
                            mm = int(time[2:4])
                            ss = int(time[4:6])
                            
                            # Convert to seconds since midnight
                            time_int = (hh * 3600) + (mm * 60) + ss

                            # Convert latitude and longitude from degrees and decimal minutes to decimal degrees
                            latitude_decimal = float(latitude[:2]) + float(latitude[2:]) / 60.0
                            if lat_hemisphere == 'S':
                                latitude_decimal = -latitude_decimal
                                
                            longitude_decimal = float(longitude[:3]) + float(longitude[3:]) / 60.0
                            if lon_hemisphere == 'W':
                                longitude_decimal = -longitude_decimal

                            # print(f"Sentence ID: {sentence_id}")
                            # print(f"UTC Time: {time}")
                            # print(f"Latitude (Decimal Degrees): {latitude_decimal:.6f}")
                            # print(f"Longitude (Decimal Degrees): {longitude_decimal:.6f}")
                            # print(f"GPS Quality: {gps_quality}")
                            # print(f"Number of Satellites: {num_satellites}")
                            # print(f"HDOP: {hdop}")
                            # print(f"Altitude: {altitude} {alt_unit}")
                            # print(f"Geoidal Separation: {geoidal_sep} {geoidal_sep_unit}")
                            # print(f"Age of Differential GPS Data: {age_diff_data}")
                            # print(f"Differential Reference Station ID: {ref_station_id}")
                            # print("-----")

                            self.gps_pub_data=[
                                time_int,
                                latitude_decimal ,
                                longitude_decimal,
                                gps_quality,
                                num_satellites,
                                hdop,
                                altitude,
                                geoidal_sep,
                            ]
                            # print(self.gps_pub_data)
                            if self.gps_pub_data is not None:
                                self.gps_pub_msg = rosarray()
                                self.gps_pub_msg.data = self.gps_pub_data
                                self.gps_pub.publish(self.gps_pub_msg)
                                self.gps_pub_data = self.gps_pub_msg.data
                                print("PUB to", self.gps_node, ":", self.gps_pub_data)

            
        except KeyboardInterrupt:
            print("GPS reading stopped.")
        except serial.SerialException as e:
            print(f"Serial port error: {e}")

        

    def publish_gps_data(self):
        data =self.read_serial_data()
        print(data)
        if self.gps_pub_data is not None:
            self.gps_pub_msg = rosarray()
            self.gps_pub_msg.data = self.gps_pub_data
            self.gps_pub.publish(self.gps_pub_msg)
            self.gps_pub_data = self.gps_pub_msg.data
            print("PUB to", self.gps_node, ":", self.gps_pub_data)


def main(args=None):
    rclpy.init(args=args)

    gps_node = GPS_NODE("gps_node")
    gps_node.init_gps_publisher("gps_data", 1, "/dev/ttyACM0", 9600)

    while rclpy.ok():
        rclpy.spin_once(gps_node)

    gps_node.close()
    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
