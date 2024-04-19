import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random

"""
KEY FOR DATA LISTS:

Final Data list:[
    
    [0-13]

    time_utc
    latitude
    lat_direction
    longitude
    lon_direction
    fix_quality
    num_satellites
    hdop
    altitude
    altitude_unit
    geoid_separation
    geoid_separation_unit
    age_of_dgps
    dgps_reference_id
    
    
    -------------

    [14-19]
    X Acceleration
    Y Acceleration
    Z Acceleration
    X Angular Acceleration
    Y Angular Acceleration
    Z Angular Acceleration

    -----------------
    [20-22]
    Cabin Temperature
    Cabin Altitude
    Cabin Pressure

    ----------------
    [23]
    Cabin 02 Level

    -----------------
    [24-25]
    Cabin C02 Level
    Cabin TVOC

    ------------------
    [26-27]
    Cabin Temperature
    Cabin Humidity

    -----------------
    [28-29]
    BMU device ID
    BMU Serial Number

    [30-40]
    CMU1 Serial Number
    CMU1 PCB Temperature
    CMU1 Cell Temperature
    CMU1 Cell 0 Voltage
    CMU1 Cell 1 Voltage
    CMU1 Cell 2 Voltage
    CMU1 Cell 3 Voltage
    CMU1 Cell 4 Voltage
    CMU1 Cell 5 Voltage
    CMU1 Cell 6 Voltage
    CMU1 Cell 7 Voltage

    [41-51]
    CMU2 Serial Number
    CMU2 PCB Temperature
    CMU2 Cell Temperature
    CMU2 Cell 0 Voltage
    CMU2 Cell 1 Voltage
    CMU2 Cell 2 Voltage
    CMU2 Cell 3 Voltage
    CMU2 Cell 4 Voltage
    CMU2 Cell 5 Voltage
    CMU2 Cell 6 Voltage
    CMU2 Cell 7 Voltage

    [52-62]
    CMU3 Serial Number
    CMU3 PCB Temperature
    CMU3 Cell Temperature
    CMU3 Cell 0 Voltage
    CMU3 Cell 1 Voltage
    CMU3 Cell 2 Voltage
    CMU3 Cell 3 Voltage
    CMU3 Cell 4 Voltage
    CMU3 Cell 5 Voltage
    CMU3 Cell 6 Voltage
    CMU3 Cell 7 Voltage

    [63-73]
    CMU4 Serial Number
    CMU4 PCB Temperature
    CMU4 Cell Temperature
    CMU4 Cell 0 Voltage
    CMU4 Cell 1 Voltage
    CMU4 Cell 2 Voltage
    CMU4 Cell 3 Voltage
    CMU4 Cell 4 Voltage
    CMU4 Cell 5 Voltage
    CMU4 Cell 6 Voltage
    CMU4 Cell 7 Voltage
    
    [74-84]
    CMU5 Serial Number
    CMU5 PCB Temperature
    CMU5 Cell Temperature
    CMU5 Cell 0 Voltage
    CMU5 Cell 1 Voltage
    CMU5 Cell 2 Voltage
    CMU5 Cell 3 Voltage
    CMU5 Cell 4 Voltage
    CMU5 Cell 5 Voltage
    CMU5 Cell 6 Voltage
    CMU5 Cell 7 Voltage

    [85-86]
    SOC(Ah) Used
    SOC percentage

    [87-88]
    Balance SOC(Ah) Used
    Balance SOC percentage

    [89-92]
    Charging Cell Voltage Error
    Cell Temperature Margin
    Dischaging Cell Voltage Error
    Total Pack Capacity

    [93-97]
    Precharge contactor driver status
    Precharge state
    12v Contactor supply voltage
    Precharge timer activity
    Precharge Timer Counter

    [98-103]
    Minimum cell voltage
    Maximum cell voltage
    CMU number that has the minimum cell voltage
    Cell number in CMU that is the minimum cell voltage
    CMU number that has the maximum cell voltage
    Cell number in CMU that is the maximum cell voltage

    [104-107]
    Minimum cell temperature
    Maximum cell temperature
    CMU number that has the minimum cell temperature
    CMU number that has the maximum cell temperature

    [108-109]
    Battery Voltage
    Battery Current

    [110-114]
    Balance voltage threshold
    Balance voltage threshold
    Status Flags
    BMS CMU count
    BMS BMU Firmware Build Number

    [115-118]
    Fan speed 0
    Fan speed 1
    12V current consumption of fans + contactors
    12V current consumption of CMUs

    [119-121]
    Status Flags
    BMU Hardware version
    BMU Model ID

    [122]
    EVDC Switch Position

    -------------------
    
    [123-141]
    MPPT1 Input Voltage - below 20 show message
    MPPT1 Input Current - >7A module contactor open MMPT
    MPPT1 Output Voltage - >175 Motor Current 0
    MPPT1 Output Current - >
    MPPT1 Mosfet Temperature - Open Contactor pre mmpt 
    MPPT1 Controller Temperature - Open Contactor pre mmpt 
    MPPT1 12V Aux Supply - error
    MPPT1 3V Aux Supply - error
    MPPT1 Max Output Voltage - log
    MPPT1 Max Input Current - log
    MPPT1 CAN RX error counter - Log
    MPPT1 CAN TX error counter - Log
    MPPT1 CAN TX overfow counter - Log
    MPPT1 error flag - error
    MPPT1 limit flags 
        7. Display <20 open contactor pre mppt
        6. Open Contactor pre mppt
        5. Display
        4. Open Contactor pre mppt
        3. Display
        2. Reserved
        1. Display
        0. above 175 open post mppt


    MPPT1 Mode
    MPPT1 Reserved
    MPPT1 Test Counter
    MPPT1 Output Voltage

    MPPT2 Input Voltage
    MPPT2 Input Current
    MPPT2 Output Voltage
    MPPT2 Output Current
    MPPT2 Mosfet Temperature
    MPPT2 Controller Temperature
    MPPT2 12V Aux Supply
    MPPT2 3V Aux Supply
    MPPT2 Max Output Voltage
    MPPT2 Max Input Voltage
    MPPT2 CAN RX error counter 
    MPPT2 CAN TX error counter
    MPPT2 CAN TX overfow counter
    MPPT2 error flag
    MPPT2 limit flags
    MPPT2 Mode
    MPPT2 Reserved
    MPPT2 Test Counter
    MPPT2 Output Voltage

    MPPT3 Input Voltage
    MPPT3 Input Current
    MPPT3 Output Voltage
    MPPT3 Output Current
    MPPT3 Mosfet Temperature
    MPPT3 Controller Temperature
    MPPT3 12V Aux Supply
    MPPT3 3V Aux Supply
    MPPT3 Max Output Voltage
    MPPT3 Max Input Voltage
    MPPT3 CAN RX error counter 
    MPPT3 CAN TX error counter
    MPPT3 CAN TX overfow counter
    MPPT3 error flag
    MPPT3 limit flags
    MPPT3 Mode
    MPPT3 Reserved
    MPPT3 Test Counter
    MPPT3 Output Voltage

    MPPT4 Input Voltage
    MPPT4 Input Current
    MPPT4 Output Voltage
    MPPT4 Output Current
    MPPT4 Mosfet Temperature
    MPPT4 Controller Temperature
    MPPT4 12V Aux Supply
    MPPT4 3V Aux Supply
    MPPT4 Max Output Voltage
    MPPT4 Max Input Voltage
    MPPT4 CAN RX error counter 
    MPPT4 CAN TX error counter
    MPPT4 CAN TX overfow counter
    MPPT4 error flag
    MPPT4 limit flags
    MPPT4 Mode
    MPPT4 Reserved
    MPPT4 Test Counter
    MPPT4 Output Voltage

    --------

    Motor Current
    Motor Velocity
    Bus Current

    Motor Controller Serial Number - NA
    Motor Controller Prohelion ID - NA

    Recieve Error Count - NA
    Transmit Error Count - NA
    Active Motor - NA
    Error Flags
        8. Motor Over Speed -> Set motor v to 0 -> set current
        7.
        6. Open Contactor for MC and Motor
        5. 
        4.
        3. Ignore
        2. Set Motor Currnet 0 -> Open Contactor for MC and Motor
        1. Open Contactor for MC and Motor
        0. Open Contactor for MC and Motor
         
    Limit Flags
        6.
        5.
        4.
        3.
        2.
        1.

    Bus Current - 
        Limit A - Drop motor current, Drop velocity
        Limit B - Open Contactor for MC and Motor
    Bus Voltage - Open Contactor for MC and Motor
    Vehicle Velocity - NA
    Motor Velocity - NA
    Phase C Current
        Limit A - Limit motor current
        Limit B - Open Contactor for Motor
    Phase B Current
        Limit A - Limit motor current
        Limit B - Open Contactor for Motor
    Vd - NA
    Vq - NA
    Id - NA
    Iq - NA
    BEMFd - Open Contactor for Motor
    BEMFq - Open Contactor for Motor
    15V Supply - If low Open Contactor for MC and Motor
    3.3V Supply - Open Contactor for MC and Motor
    1.9V Supply - Open Contactor for MC and Motor
    Heat Sink Temp - Slow Down
        
    Motor Temp - Drop Motor Current, slow down
    DSP Board Temp -  Slow down
    DC Bus Ah - NA
    Odometer - NA
    Slip Speed - Dont Care
    Active Motor - Dont Care
    ---------
    ]
          
"""


class MAIN_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # # Reference
    # def init_pub(self,topic,timer_period):
    #     self.publisher = self.create_publisher(rosarray, topic, 10)
    #     self.timer = self.create_timer(timer_period, self.publish_data)
    #     self.pub_data = None

    # def publish_data(self):
    #     self.pub_msg = rosarray()
    #     self.pub_msg.data = self.data
    #     self.publisher.publish(self.pub_msg)
    #     self.pub_data = self.pub_msg.data
    #     print("PUB:", self.pub_data)

    # def init_sub(self,topic):
    #     self.subscription = self.create_subscription(rosarray, topic, self.receive_data, 10)
    #     self.subscription  # prevent unused variable warning
    #     self.latest_sub_data = None

    # def receive_data(self, sub_msg):
    #     self.latest_sub_data = sub_msg.data

    # Control Subscriber
    def init_control_subscriber(self, topic):
        self.control_subscription = self.create_subscription(
            rosarray, topic, self.receive_control_data, 10
        )
        self.control_subscription  # prevent unused variable warning
        self.control_sub_data = []

    def receive_control_data(self, msg):
        self.control_sub_data = msg.data

    # CAN Subscriber
    def init_can_subscriber(self, topic):
        self.can_subscription = self.create_subscription(
            rosarray, topic, self.receive_can_data, 10
        )
        self.can_subscription  # prevent unused variable warning
        self.can_sub_data = []

    def receive_can_data(self, msg):
        self.can_sub_data = msg.data

    # GPS Subscriber
    def init_gps_subscriber(self, topic):
        self.gps_subscription = self.create_subscription(
            rosarray, topic, self.receive_gps_data, 10
        )
        self.gps_subscription  # prevent unused variable warning
        self.gps_sub_data = []

    def receive_gps_data(self, msg):
        self.gps_sub_data = msg.data

        # IMU Subscriber

    def init_imu_subscriber(self, topic):
        self.imu_subscription = self.create_subscription(
            rosarray, topic, self.receive_imu_data, 10
        )
        self.imu_subscription  # prevent unused variable warning
        self.imu_sub_data = []

    def receive_imu_data(self, msg):
        self.imu_sub_data = msg.data

    # CAN publisher
    def init_can_publisher(self, topic, timer_period):
        self.can_pub = self.create_publisher(rosarray, topic, 10)
        self.can_timer = self.create_timer(timer_period, self.publish_can_data)
        self.can_pub_data = []

    def publish_can_data(self):
        self.can_pub_msg = rosarray()
        self.can_pub_msg.data = self.can_pub_data
        self.can_pub.publish(self.can_pub_msg)
        self.can_pub_data = self.can_pub_msg.data
        print("PUB:", self.can_pub_data)

    # Final Data publisher
    def init_final_data_publisher(self, topic, timer_period):
        self.final_data_pub = self.create_publisher(rosarray, topic, 10)
        self.final_data_timer = self.create_timer(timer_period, self.publish_final_data)
        self.final_data_pub_data = []

    def publish_final_data(self):
        self.final_data_pub_msg = rosarray()
        self.final_data_pub_msg.data = self.final_data_pub_data
        self.final_data_pub.publish(self.final_data_pub_msg)
        self.final_data_pub_data = self.final_data_pub_msg.data
        print("PUB:", self.final_data_pub_data)

    # Parsed publisher
    def init_parsed_data_publisher(self, topic, timer_period):
        self.parsed_data_pub = self.create_publisher(rosarray, topic, 10)
        self.parsed_data_timer = self.create_timer(
            timer_period, self.publish_parsed_data
        )
        self.parsed_data_pub_data = []

    def publish_parsed_data(self):
        self.parsed_data_pub_msg = rosarray()
        self.parsed_data_pub_msg.data = self.parsed_data_pub_data
        self.parsed_data_pub.publish(self.parsed_data_pub_msg)
        self.parsed_data_pub_data = self.parsed_data_pub_msg.data
        print("PUB:", self.parsed_data_pub_data)


def main(args=None):
    rclpy.init(args=args)

    main_node = MAIN_NODE("main_node")
    main_node.init_control_subscriber("control_data")
    main_node.init_can_subscriber("can_data")
    main_node.init_gps_subscriber("gps_data")
    main_node.init_imu_subscriber("imu_data")

    main_node.init_can_publisher("can_rx_data", 1)
    #main_node.init_final_data_publisher("final_data", 1)
    #main_node.init_parsed_data_publisher("parsed_data", 1)
    try:
        while rclpy.ok():
            main_node.can_pub_data = [
                0x23,
                random.randint(0, 9),
                random.randint(0, 9),
                random.randint(0, 9),
                random.randint(0, 9),
                random.randint(0, 9),
                random.randint(0, 9),
                random.randint(0, 9),
                random.randint(0, 9),
                
            ]

            rclpy.spin_once(main_node)

            control_sub_data = main_node.control_sub_data
            can_sub_data = main_node.can_sub_data
            gps_sub_data = main_node.gps_sub_data
            imu_sub_data = main_node.imu_sub_data

            if control_sub_data is not None:
                print("SUB:", control_sub_data)
            if can_sub_data is not None:
                print("SUB:", can_sub_data)
            if gps_sub_data is not None:
                print("SUB:", gps_sub_data)
            if imu_sub_data is not None:
                print("SUB:", imu_sub_data)
    except KeyboardInterrupt:
        print("Program Interupted.Exiting...")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    main_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
