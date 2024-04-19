import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray

class BATTERY_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # Battery Subscriber
    def init_battery_subscriber(self, topic):
        self.battery_subscriber = self.create_subscription(
            rosarray, topic, self.receive_battery_data, 10
        )
        self.battery_subscriber  # prevent unused variable warning
        self.battery_sub_data = None

    def receive_battery_data(self, msg):
        self.battery_sub_data = msg.data

    # Peripheral Subscriber
    def init_peripheral_subscriber(self, topic):
        self.peripheral_subscriber = self.create_subscription(
            rosarray, topic, self.receive_peripheral_data, 10
        )
        self.peripheral_subscriber  # prevent unused variable warning
        self.peripheral_sub_data = None

    def receive_peripheral_data(self, msg):
        self.peripheral_sub_data = msg.data

    # Control Publisher
    def init_control_data_publisher(self, topic, timer_period):
        self.control_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.control_data_timer = self.create_timer(
            timer_period, self.publish_control_data
        )
        self.control_pub_data = None

    def publish_control_data(self):
        if self.control_pub_data is not None:
            self.control_data_pub_msg = rosarray()
            self.control_data_pub_msg.data = self.control_pub_data
            self.control_data_publisher.publish(self.control_data_pub_msg)
            self.control_pub_data = self.control_data_pub_msg.data
            print("PUB:", self.control_pub_data)


def main(args=None):
    rclpy.init(args=args)

    battery_node = BATTERY_NODE("battery_node")
    battery_node.init_battery_subscriber("battery_data")
    battery_node.init_peripheral_subscriber("peripheral_data")
    battery_node.init_control_data_publisher("control_data", 1)



    while rclpy.ok():
        rclpy.spin_once(battery_node)

        # Peripheral Status 
        # Assume we have DPSTs for MPPTs, Triple Pole Contactor from MC to Motor
        peripherals = [
            0,0,0,0,0,0,0,0, #8x MPPT
            0,0,0,0, # MPPT Precharge
            0,0,0,0,  # MC Precharge
            0,         # Motor Contactor
            0, #Battery Contactor
                       ]

        # Parsing the Data
        control_data_list = []

        # CMU 1 Data
        cmu_1_pcb_temp = battery_node.battery_sub_data[0]*10
        cmu_1_cell_temp = battery_node.battery_sub_data[1]
        cmu_1_cell_0_voltage = battery_node.battery_sub_data[2]
        cmu_1_cell_1_voltage = battery_node.battery_sub_data[3]
        cmu_1_cell_2_voltage = battery_node.battery_sub_data[4]
        cmu_1_cell_3_voltage = battery_node.battery_sub_data[5]
        cmu_1_cell_4_voltage = battery_node.battery_sub_data[6]
        cmu_1_cell_5_voltage = battery_node.battery_sub_data[7]
        cmu_1_cell_6_voltage = battery_node.battery_sub_data[8]
        cmu_1_cell_7_voltage = battery_node.battery_sub_data[9]

        # CMU 2 Data
        cmu_2_pcb_temp = battery_node.battery_sub_data[10]
        cmu_2_cell_temp = battery_node.battery_sub_data[11]
        cmu_2_cell_0_voltage = battery_node.battery_sub_data[12]
        cmu_2_cell_1_voltage = battery_node.battery_sub_data[13]
        cmu_2_cell_2_voltage = battery_node.battery_sub_data[14]
        cmu_2_cell_3_voltage = battery_node.battery_sub_data[15]
        cmu_2_cell_4_voltage = battery_node.battery_sub_data[16]
        cmu_2_cell_5_voltage = battery_node.battery_sub_data[17]
        cmu_2_cell_6_voltage = battery_node.battery_sub_data[18]
        cmu_2_cell_7_voltage = battery_node.battery_sub_data[19]

        # CMU 3 Data
        cmu_3_pcb_temp = battery_node.battery_sub_data[20]
        cmu_3_cell_temp = battery_node.battery_sub_data[21]
        cmu_3_cell_0_voltage = battery_node.battery_sub_data[22]
        cmu_3_cell_1_voltage = battery_node.battery_sub_data[23]
        cmu_3_cell_2_voltage = battery_node.battery_sub_data[24]
        cmu_3_cell_3_voltage = battery_node.battery_sub_data[25]
        cmu_3_cell_4_voltage = battery_node.battery_sub_data[26]
        cmu_3_cell_5_voltage = battery_node.battery_sub_data[27]
        cmu_3_cell_6_voltage = battery_node.battery_sub_data[28]
        cmu_3_cell_7_voltage = battery_node.battery_sub_data[29]

        # CMU 4 Data
        cmu_4_pcb_temp = battery_node.battery_sub_data[30]
        cmu_4_cell_temp = battery_node.battery_sub_data[31]
        cmu_4_cell_0_voltage = battery_node.battery_sub_data[32]
        cmu_4_cell_1_voltage = battery_node.battery_sub_data[33]
        cmu_4_cell_2_voltage = battery_node.battery_sub_data[34]
        cmu_4_cell_3_voltage = battery_node.battery_sub_data[35]
        cmu_4_cell_4_voltage = battery_node.battery_sub_data[36]
        cmu_4_cell_5_voltage = battery_node.battery_sub_data[37]
        cmu_4_cell_6_voltage = battery_node.battery_sub_data[38]
        cmu_4_cell_7_voltage = battery_node.battery_sub_data[39]

        # CMU 5 Data
        cmu_5_pcb_temp = battery_node.battery_sub_data[40]
        cmu_5_cell_temp = battery_node.battery_sub_data[41]
        cmu_5_cell_0_voltage = battery_node.battery_sub_data[42]
        cmu_5_cell_1_voltage = battery_node.battery_sub_data[43]
        cmu_5_cell_2_voltage = battery_node.battery_sub_data[44]
        cmu_5_cell_3_voltage = battery_node.battery_sub_data[45]
        cmu_5_cell_4_voltage = battery_node.battery_sub_data[46]
        cmu_5_cell_5_voltage = battery_node.battery_sub_data[47]
        cmu_5_cell_6_voltage = battery_node.battery_sub_data[48]
        cmu_5_cell_7_voltage = battery_node.battery_sub_data[49]

        # SOC Data
        battery_soc_ah = battery_node.battery_sub_data[50]
        battery_soc_percent = battery_node.battery_sub_data[51]
    
        # Cell Voltage Data
        min_cell_voltage = battery_node.battery_sub_data[52]
        max_cell_voltage = battery_node.battery_sub_data[53]

        # Cell Temperature Data
        min_cell_temp = battery_node.battery_sub_data[54]
        max_cell_temp = battery_node.battery_sub_data[55]

        # Battery Data
        battery_voltage = battery_node.battery_sub_data[56]
        battery_current = battery_node.battery_sub_data[57]

        # Battery Status
        battery_status = battery_node.battery_sub_data[58]

        # CMU PCB Temperature Error - Temperature is in 1/10th of C
        #print(cmu_1_pcb_temp)
        if cmu_1_pcb_temp > 40 and cmu_1_pcb_temp < 50:
            control_data_list.append(1)

        if cmu_2_pcb_temp*10 > 40 and cmu_2_pcb_temp*10 < 50:
            control_data_list.append(2)

        if cmu_3_pcb_temp*10 > 40 and cmu_3_pcb_temp*10 < 50:
            control_data_list.append(3)

        if cmu_4_pcb_temp*10 > 40 and cmu_4_pcb_temp*10 < 50:
            control_data_list.append(4)

        if cmu_5_pcb_temp*10 > 40 and cmu_5_pcb_temp*10 < 50:
            control_data_list.append(5)

        if cmu_1_pcb_temp*10 > 50:
            control_data_list.append(6)

        if cmu_2_pcb_temp*10 > 50:
            control_data_list.append(7)
        
        if cmu_3_pcb_temp*10 > 50:
            control_data_list.append(8)

        if cmu_4_pcb_temp*10 > 50:
            control_data_list.append(9)

        if cmu_5_pcb_temp*10 > 50:
            control_data_list.append(10)

        # CMU Cell Temperature Error - Temperature is in 1/10th of C
 
        if cmu_1_cell_temp*10 > 40 and cmu_1_cell_temp*10 < 50:
            control_data_list.append(11)

        if cmu_2_cell_temp*10 > 40 and cmu_2_cell_temp*10 < 50:
            control_data_list.append(12)

        if cmu_3_cell_temp*10 > 40 and cmu_3_cell_temp*10 < 50:
            control_data_list.append(13)

        if cmu_4_cell_temp*10 > 40 and cmu_4_cell_temp*10 < 50:
            control_data_list.append(14)

        if cmu_5_cell_temp*10 > 40 and cmu_5_cell_temp*10 < 50:
            control_data_list.append(15)

        if cmu_1_cell_temp*10 > 50:
            control_data_list.append(16)

        if cmu_2_cell_temp*10 > 50:
            control_data_list.append(17)

        if cmu_3_cell_temp*10 > 50:
            control_data_list.append(18)

        if cmu_4_cell_temp*10 > 50:
            control_data_list.append(19)

        if cmu_5_cell_temp*10 > 50:
            control_data_list.append(20)

        # Voltages are in MilliVolts
        # CMU_1 Cell Undervoltage or Overvoltage Error

        if cmu_1_cell_0_voltage*1000 < 3:
            control_data_list.append(21)

        if cmu_1_cell_1_voltage*1000 < 3:
            control_data_list.append(22)

        if cmu_1_cell_2_voltage*1000 < 3:
            control_data_list.append(23)

        if cmu_1_cell_3_voltage*1000 < 3:
            control_data_list.append(24)

        if cmu_1_cell_4_voltage*1000 < 3:
            control_data_list.append(25)
            
        if cmu_1_cell_5_voltage*1000 < 3:
            control_data_list.append(26)

        if cmu_1_cell_6_voltage*1000 < 3:
            control_data_list.append(27)

        if cmu_1_cell_7_voltage*1000 < 3:
            control_data_list.append(28)

        if cmu_1_cell_0_voltage*1000 > 4:
            control_data_list.append(29)

        if cmu_1_cell_1_voltage*1000 > 4:
            control_data_list.append(30)

        if cmu_1_cell_2_voltage*1000 > 4:
            control_data_list.append(31)

        if cmu_1_cell_3_voltage*1000 > 4:
            control_data_list.append(32)

        if cmu_1_cell_4_voltage*1000 > 4:
            control_data_list.append(33)

        if cmu_1_cell_5_voltage*1000 > 4:
            control_data_list.append(34)

        if cmu_1_cell_6_voltage*1000 > 4:
            control_data_list.append(35)

        if cmu_1_cell_7_voltage*1000 > 4:
            control_data_list.append(36)

        # CMU_2 Cell Undervoltage or Overvoltage Error

        if cmu_2_cell_0_voltage*1000 < 3:
            control_data_list.append(37)

        if cmu_2_cell_1_voltage*1000 < 3:
            control_data_list.append(38)

        if cmu_2_cell_2_voltage*1000 < 3:
            control_data_list.append(39)

        if cmu_2_cell_3_voltage*1000 < 3:
            control_data_list.append(40)

        if cmu_2_cell_4_voltage*1000 < 3:
            control_data_list.append(41)

        if cmu_2_cell_5_voltage*1000 < 3:
            control_data_list.append(42)

        if cmu_2_cell_6_voltage*1000 < 3:
            control_data_list.append(43)

        if cmu_2_cell_7_voltage*1000 < 3:
            control_data_list.append(44)
                
        if cmu_2_cell_0_voltage*1000 > 4:
            control_data_list.append(45)

        if cmu_2_cell_1_voltage*1000 > 4:
            control_data_list.append(46)

        if cmu_2_cell_2_voltage*1000 > 4:
            control_data_list.append(47)

        if cmu_2_cell_3_voltage*1000 > 4:
            control_data_list.append(48)

        if cmu_2_cell_4_voltage*1000 > 4:
            control_data_list.append(49)

        if cmu_2_cell_5_voltage*1000 > 4:
            control_data_list.append(50)
        
        if cmu_2_cell_6_voltage*1000 > 4:
            control_data_list.append(51)

        if cmu_2_cell_7_voltage*1000 > 4:
            control_data_list.append(52)

        # CMU_3 Cell Undervoltage or Overvoltage Error
        if cmu_3_cell_0_voltage*1000 < 3:
            control_data_list.append(53)

        if cmu_3_cell_1_voltage*1000 < 3:
            control_data_list.append(54)

        if cmu_3_cell_2_voltage*1000 < 3:
            control_data_list.append(55)

        if cmu_3_cell_3_voltage*1000 < 3:
            control_data_list.append(56)

        if cmu_3_cell_4_voltage*1000 < 3:
            control_data_list.append(57)

        if cmu_3_cell_5_voltage*1000 < 3:
            control_data_list.append(58)

        if cmu_3_cell_6_voltage*1000 < 3:
            control_data_list.append(59)

        if cmu_3_cell_7_voltage*1000 < 3:
            control_data_list.append(60)

        if cmu_3_cell_0_voltage*1000 > 4:
            control_data_list.append(61)

        if cmu_3_cell_1_voltage*1000 > 4:
            control_data_list.append(62)

        if cmu_3_cell_2_voltage*1000 > 4:
            control_data_list.append(63)

        if cmu_3_cell_3_voltage*1000 > 4:
            control_data_list.append(64)

        if cmu_3_cell_4_voltage*1000 > 4:
            control_data_list.append(65)

        if cmu_3_cell_5_voltage*1000 > 4:
            control_data_list.append(66)

        if cmu_3_cell_6_voltage*1000 > 4:
            control_data_list.append(67)

        if cmu_3_cell_7_voltage*1000 > 4:
            control_data_list.append(68)

        # CMU_4 Cell Undervoltage or Overvoltage Error
        if cmu_4_cell_0_voltage*1000 < 3:
            control_data_list.append(69)

        if cmu_4_cell_1_voltage*1000 < 3:
            control_data_list.append(70)

        if cmu_4_cell_2_voltage*1000 < 3:
            control_data_list.append(71)

        if cmu_4_cell_3_voltage*1000 < 3:
            control_data_list.append(72)

        if cmu_4_cell_4_voltage*1000 < 3:
            control_data_list.append(73)

        if cmu_4_cell_5_voltage*1000 < 3:
            control_data_list.append(74)

        if cmu_4_cell_6_voltage*1000 < 3:
            control_data_list.append(75)

        if cmu_4_cell_7_voltage*1000 < 3:
            control_data_list.append(76)

        if cmu_4_cell_0_voltage*1000 > 4:
            control_data_list.append(77)

        if cmu_4_cell_1_voltage*1000 > 4:
            control_data_list.append(78)

        if cmu_4_cell_2_voltage*1000 > 4:
            control_data_list.append(79)

        if cmu_4_cell_3_voltage*1000 > 4:
            control_data_list.append(80)

        if cmu_4_cell_4_voltage*1000 > 4:
            control_data_list.append(81)

        if cmu_4_cell_5_voltage*1000 > 4:
            control_data_list.append(82)

        if cmu_4_cell_6_voltage*1000 > 4:
            control_data_list.append(83)

        if cmu_4_cell_7_voltage*1000 > 4:
            control_data_list.append(84)

        # CMU_5 Cell Undervoltage or Overvoltage Error - CMU 5 only has 6 cells
        if cmu_5_cell_0_voltage*1000 < 3:
            control_data_list.append(85)

        if cmu_5_cell_1_voltage*1000 < 3:
            control_data_list.append(86)

        if cmu_5_cell_2_voltage*1000 < 3:
            control_data_list.append(87)

        if cmu_5_cell_3_voltage*1000 < 3:
            control_data_list.append(88)

        if cmu_5_cell_4_voltage*1000 < 3:
            control_data_list.append(89)
            
        if cmu_5_cell_5_voltage*1000 < 3:
            control_data_list.append(90)

        if cmu_5_cell_0_voltage*1000 > 4:
            control_data_list.append(91)

        if cmu_5_cell_1_voltage*1000 > 4:
            control_data_list.append(92)
        
        if cmu_5_cell_2_voltage*1000 > 4:
            control_data_list.append(93)

        if cmu_5_cell_3_voltage*1000 > 4:
            control_data_list.append(94)

        if cmu_5_cell_4_voltage*1000 > 4:
            control_data_list.append(95)

        if cmu_5_cell_5_voltage*1000 > 4:
            control_data_list.append(96)

            
        if battery_soc_ah > 114: # 114 Ah i g
            control_data_list.append(97)

        if battery_soc_percent < 10:
            control_data_list.append(98)

        if battery_soc_percent > 90:
            control_data_list.append(99)

        if min_cell_voltage*1000 < 3:
            control_data_list.append(100)

        if max_cell_voltage*1000 > 4:
            control_data_list.append(101)

        if min_cell_temp*10 > 40 and min_cell_temp*10 < 50:
            control_data_list.append(102)

        if min_cell_temp*10 > 50:
            control_data_list.append(103)

        if max_cell_temp*10 > 40 and max_cell_temp*10 < 50:
            control_data_list.append(104)

        if max_cell_temp*10 > 50:
            control_data_list.append(105)

        if battery_voltage*1000 > 160:
            control_data_list.append(106)

        if battery_voltage*1000 < 120:
            control_data_list.append(107)

        if battery_current*1000 > 30:
            control_data_list.append(108)

        if battery_status == 0x01:
            control_data_list.append(109)

        if battery_status == 0x02:
            control_data_list.append(110)

        if battery_status == 0x04:
            control_data_list.append(111)

        if  battery_status == 0x08:
            control_data_list.append(112)

        if  battery_status == 0x10:
            control_data_list.append(113)

        if  battery_status == 0x20:
            control_data_list.append(114)

        if  battery_status == 0x40:
            control_data_list.append(115)

        if  battery_status == 0x80:
            control_data_list.append(116)

        battery_node.control_pub_data = control_data_list

    battery_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
