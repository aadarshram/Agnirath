import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random

'''
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
          
'''

class control_subscriber(Node):

    def __init__(self):
        super().__init__('control_s_node')
        self.subscription = self.create_subscription(rosarray,'control_data',self.recieve_data,10)
        self.subscription  # prevent unused variable warning
        self.latest_data = None

    def recieve_data(self, msg):
        self.latest_data = msg.data
    
    def get_latest_data(self):
        return self.latest_data
    

class can_subscriber(Node):

    def __init__(self):
        super().__init__('can_s_node')
        self.subscription = self.create_subscription(rosarray,'can_data',self.recieve_data,10)
        self.subscription  # prevent unused variable warning
        self.latest_data = None

    def recieve_data(self, msg):
        self.latest_data = msg.data
    
    def get_latest_data(self):
        return self.latest_data
    
class gps_subscriber(Node):

    def __init__(self):
        super().__init__('gps_s_node')
        self.subscription = self.create_subscription(rosarray,'gps_data',self.recieve_data,10)
        self.subscription  # prevent unused variable warning
        self.latest_data = None

    def recieve_data(self, msg):
        self.latest_data = msg.data
    
    def get_latest_data(self):
        return self.latest_data
    
class imu_subscriber(Node):

    def __init__(self):
        super().__init__('imu_s_node')
        self.subscription = self.create_subscription(rosarray,'imu_data',self.recieve_data,10)
        self.subscription  # prevent unused variable warning
        self.latest_data = None

    def recieve_data(self, msg):
        self.latest_data = msg.data
    
    def get_latest_data(self):
        return self.latest_data
    
class can_publisher(Node):

    def __init__(self):
        super().__init__('can_p_node')
        self.publisher_ = self.create_publisher(rosarray, 'control_data', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.recieve_data)
        self.data = [0,0,0,0,0]
        self.latest_data = None

    def recieve_data(self):
        msg = rosarray()
        msg.data = self.data
        self.publisher_.publish(msg)
        self.data = [random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1)]
        self.latest_data = msg.data
        
    def get_latest_data(self):
        return self.latest_data
    
class parsed_data_publisher(Node):

    def __init__(self):
        super().__init__('parsed_data_p_node')
        self.publisher_ = self.create_publisher(rosarray, 'parsed_data', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.recieve_data)
        self.data = [0,0,0,0,0]
        self.latest_data = None

    def recieve_data(self):
        msg = rosarray()
        msg.data = self.data
        self.publisher_.publish(msg)
        self.data = [random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1)]
        self.latest_data = msg.data
        
    def get_latest_data(self):
        return self.latest_data
    
class final_data_publisher(Node):

    def __init__(self):
        super().__init__('final_data_p_node')
        self.publisher_ = self.create_publisher(rosarray, 'final_data', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.recieve_data)
        self.data = [0,0,0,0,0]
        self.latest_data = None

    def recieve_data(self):
        msg = rosarray()
        msg.data = self.data
        self.publisher_.publish(msg)
        self.data = [random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1)]
        self.latest_data = msg.data
        
    def get_latest_data(self):
        return self.latest_data
    

def main(args=None):
    rclpy.init(args=args)

    gps_s_node = gps_subscriber()
    imu_s_node = can_subscriber()
    control_s_node = control_subscriber()
    can_s_node = can_subscriber()


    can_p_node = can_publisher()
    parsed_data_p_node = parsed_data_publisher()
    final_data_p_node = final_data_publisher()

    final_data = [0]*200
    parsed_data = []
    CAN_data = []

    while rclpy.ok():
        rclpy.spin_once(control_s_node)
        rclpy.spin_once(can_s_node)

        latest_gps_data = gps_s_node.get_latest_data()
        latest_imu_data = imu_s_node.get_latest_data()
        latest_control_data = control_s_node.get_latest_data()
        latest_can_data = can_s_node.get_latest_data()

        if None not in latest_gps_data:
            # GPS Data
            final_data[0:14] = latest_gps_data
        if None not in latest_imu_data:
            # IMU Data
            final_data[14:20] = latest_imu_data
        if None not in latest_can_data:
            # BMU Serial Number and ID
            if latest_can_data[0] == 0x600:
                final_data[28-30] = latest_can_data[1:]
            # CMU 1
            if latest_can_data[0] == 0x601:
                final_data[30-33] = latest_can_data[1:]
            if latest_can_data[0] == 0x602:
                final_data[33-37] = latest_can_data[1:]
            if latest_can_data[0] == 0x603:
                final_data[37-41] = latest_can_data[1:]
            # CMU 2
            if latest_can_data[0] == 0x604:
                final_data[41-44] = latest_can_data[1:]
            if latest_can_data[0] == 0x605:
                final_data[44-48] = latest_can_data[1:]
            if latest_can_data[0] == 0x606:
                final_data[48-52] = latest_can_data[1:]
            # CMU 3
            if latest_can_data[0] == 0x607:
                final_data[30-33] = latest_can_data[1:]
            if latest_can_data[0] == 0x608:
                final_data[33-37] = latest_can_data[1:]
            if latest_can_data[0] == 0x609:
                final_data[37-41] = latest_can_data[1:]
            # CMU 4
            if latest_can_data[0] == 0x610:
                final_data[30-33] = latest_can_data[1:]
            if latest_can_data[0] == 0x611:
                final_data[33-37] = latest_can_data[1:]
            if latest_can_data[0] == 0x612:
                final_data[37-41] = latest_can_data[1:]
            # CMU 5

            
            



        if latest_control_data is not None:
            print(latest_control_data)

        print(final_data)

        

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_s_node.destroy_node()
    can_s_node.destroy_node()
    gps_s_node.destroy_node()
    imu_s_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()