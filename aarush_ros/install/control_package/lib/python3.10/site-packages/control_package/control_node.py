import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray


class CONTROL_NODE(Node):

    def __init__(self,node):
        super().__init__(node)
    
    # Parsed Data Subscriber
    def init_parsed_data_subscriber(self, topic):
        self.parsed_data_subscriber = self.create_subscription(
            rosarray, topic, self.receive_parsed_data, 10
        )
        self.parsed_data_subscriber  # prevent unused variable warning
        self.parsed_sub_data = None

    def receive_parsed_data(self, msg):
        self.parsed_sub_data = msg.data
        #print("SUB:",self.parsed_sub_data)

    def init_control_data_publisher(self, topic, timer_period):
        self.control_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.control_data_timer = self.create_timer(timer_period, self.publish_control_data)
        self.control_pub_data = None

    def publish_control_data(self):
        if self.control_pub_data is not None:
            self.control_data_pub_msg = rosarray()
            self.control_data_pub_msg.data = self.control_pub_data
            self.control_data_publisher.publish(self.control_data_pub_msg)
            self.control_pub_data = self.control_data_pub_msg.data
            print("CONTROL_PUB:", self.control_pub_data)


def main(args=None):
    rclpy.init(args=args)

    control_node = CONTROL_NODE("control_node")
    control_node.init_parsed_data_subscriber("/parsed_data")
    control_node.init_control_data_publisher("control_data",1)

    while rclpy.ok():
        rclpy.spin_once(control_node)

        if control_node.parsed_sub_data is not None:
            battery_data = control_node.parsed_sub_data[0:59]
            mppt_data = control_node.parsed_sub_data[59:163]
            mc_data = control_node.parsed_sub_data[163:184]
        

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
        cmu_1_pcb_temp = battery_data[0]*10
        cmu_1_cell_temp = battery_data[1]
        cmu_1_cell_0_voltage = battery_data[2]
        cmu_1_cell_1_voltage = battery_data[3]
        cmu_1_cell_2_voltage = battery_data[4]
        cmu_1_cell_3_voltage = battery_data[5]
        cmu_1_cell_4_voltage = battery_data[6]
        cmu_1_cell_5_voltage = battery_data[7]
        cmu_1_cell_6_voltage = battery_data[8]
        cmu_1_cell_7_voltage = battery_data[9]

        # CMU 2 Data
        cmu_2_pcb_temp = battery_data[10]
        cmu_2_cell_temp = battery_data[11]
        cmu_2_cell_0_voltage = battery_data[12]
        cmu_2_cell_1_voltage = battery_data[13]
        cmu_2_cell_2_voltage = battery_data[14]
        cmu_2_cell_3_voltage = battery_data[15]
        cmu_2_cell_4_voltage = battery_data[16]
        cmu_2_cell_5_voltage = battery_data[17]
        cmu_2_cell_6_voltage = battery_data[18]
        cmu_2_cell_7_voltage = battery_data[19]

        # CMU 3 Data
        cmu_3_pcb_temp = battery_data[20]
        cmu_3_cell_temp = battery_data[21]
        cmu_3_cell_0_voltage = battery_data[22]
        cmu_3_cell_1_voltage = battery_data[23]
        cmu_3_cell_2_voltage = battery_data[24]
        cmu_3_cell_3_voltage = battery_data[25]
        cmu_3_cell_4_voltage = battery_data[26]
        cmu_3_cell_5_voltage = battery_data[27]
        cmu_3_cell_6_voltage = battery_data[28]
        cmu_3_cell_7_voltage = battery_data[29]

        # CMU 4 Data
        cmu_4_pcb_temp = battery_data[30]
        cmu_4_cell_temp = battery_data[31]
        cmu_4_cell_0_voltage = battery_data[32]
        cmu_4_cell_1_voltage = battery_data[33]
        cmu_4_cell_2_voltage = battery_data[34]
        cmu_4_cell_3_voltage = battery_data[35]
        cmu_4_cell_4_voltage = battery_data[36]
        cmu_4_cell_5_voltage = battery_data[37]
        cmu_4_cell_6_voltage = battery_data[38]
        cmu_4_cell_7_voltage = battery_data[39]

        # CMU 5 Data
        cmu_5_pcb_temp = battery_data[40]
        cmu_5_cell_temp = battery_data[41]
        cmu_5_cell_0_voltage = battery_data[42]
        cmu_5_cell_1_voltage = battery_data[43]
        cmu_5_cell_2_voltage = battery_data[44]
        cmu_5_cell_3_voltage = battery_data[45]
        cmu_5_cell_4_voltage = battery_data[46]
        cmu_5_cell_5_voltage = battery_data[47]
        cmu_5_cell_6_voltage = battery_data[48]
        cmu_5_cell_7_voltage = battery_data[49]

        # SOC Data
        battery_soc_ah = battery_data[50]
        battery_soc_percent = battery_data[51]
    
        # Cell Voltage Data
        min_cell_voltage = battery_data[52]
        max_cell_voltage = battery_data[53]

        # Cell Temperature Data
        min_cell_temp = battery_data[54]
        max_cell_temp = battery_data[55]

        # Battery Data
        battery_voltage = battery_data[56]
        battery_current = battery_data[57]

        # Battery Status
        battery_status = battery_data[58]

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


        mppt_input_volt = [mppt_data[0], mppt_data[26], mppt_data[52], mppt_data[78]]
        mppt_input_current = [mppt_data[1], mppt_data[27], mppt_data[53], mppt_data[79]]
        mppt_output_volt = [mppt_data[2], mppt_data[28], mppt_data[54], mppt_data[80]]
        mppt_output_current = [mppt_data[3], mppt_data[29], mppt_data[55], mppt_data[81]]
        mppt_mosfet_temp = [mppt_data[4], mppt_data[30], mppt_data[56], mppt_data[82]]
        mppt_controller_temp = [mppt_data[5], mppt_data[31], mppt_data[57], mppt_data[83]]
        mppt_12V_aux_supply = [mppt_data[6], mppt_data[32], mppt_data[58], mppt_data[84]]
        mppt_3V_aux_supply = [mppt_data[7], mppt_data[33], mppt_data[59], mppt_data[85]]
        mppt_low_pwr_array = [mppt_data[8], mppt_data[34], mppt_data[60], mppt_data[86]]
        mppt_mosfet_ovrht = [mppt_data[9], mppt_data[35], mppt_data[61], mppt_data[87]]
        mppt_batt_low = [mppt_data[10], mppt_data[36], mppt_data[62], mppt_data[88]]
        mppt_batt_full = [mppt_data[11], mppt_data[37], mppt_data[63], mppt_data[89]]
        mppt_12_undervolt = [mppt_data[12], mppt_data[38], mppt_data[64], mppt_data[90]]
        mppt_hw_overcurrent = [mppt_data[13], mppt_data[39], mppt_data[65], mppt_data[91]]
        mppt_hw_overvolt = [mppt_data[14], mppt_data[40], mppt_data[66], mppt_data[92]]
        mppt_ip_curr_min = [mppt_data[15], mppt_data[41], mppt_data[67], mppt_data[93]]
        mppt_ip_curr_max = [mppt_data[16], mppt_data[42], mppt_data[68], mppt_data[94]]
        mppt_op_volt_max = [mppt_data[17], mppt_data[43], mppt_data[69], mppt_data[95]]
        mppt_mosfet_temp_limit = [mppt_data[18], mppt_data[44], mppt_data[70], mppt_data[96]]
        mppt_duty_cycle_min = [mppt_data[19], mppt_data[45], mppt_data[71], mppt_data[97]]
        mppt_duty_cycle_max = [mppt_data[20], mppt_data[46], mppt_data[72], mppt_data[98]]
        mppt_local_mppt = [mppt_data[21], mppt_data[47], mppt_data[73], mppt_data[99]]
        mppt_global_mppt = [mppt_data[22], mppt_data[48], mppt_data[74], mppt_data[100]]
        mppt_mode = [mppt_data[23], mppt_data[49], mppt_data[75], mppt_data[101]]
        mppt_battery_side_op_volt = [mppt_data[24], mppt_data[50], mppt_data[76], mppt_data[102]]
        mppt_power_conn_temp = [mppt_data[25], mppt_data[51], mppt_data[77], mppt_data[103]]

        control_data_list = []

        base_val = 127
        #Fill in the limiting values
        min_input_volt = 20
        max_input_current = 7
        max_output_volt = 175
        max_mosfet_temp = 50
        max_controller_temp =50
        supply_12V_tol = 1
        supply_3V_tol = 1
        
        #MPPT_1
        if mppt_input_volt[0] < min_input_volt:
            control_data_list.append(base_val + 0)

        if mppt_input_current[0] > max_input_current:
            control_data_list.append(base_val + 1)
        
        if mppt_output_volt[0] > max_output_volt:
            control_data_list.append(base_val + 2)

        if mppt_mosfet_temp[0] > max_mosfet_temp:
            control_data_list.append(base_val + 3)

        if mppt_controller_temp[0] > max_controller_temp:
            control_data_list.append(base_val + 4)
        
        if mppt_12V_aux_supply[0] < 12 - supply_12V_tol:
            control_data_list.append(base_val + 5)

        if mppt_3V_aux_supply[0] < 3 - supply_12V_tol:
            control_data_list.append(base_val + 6)

        if mppt_low_pwr_array[0] == 1.0:
            control_data_list.append(base_val + 7)

        if mppt_mosfet_ovrht[0] == 1.0:
            control_data_list.append(base_val + 8)

        if mppt_batt_low[0] == 1.0:
            control_data_list.append(base_val + 9)

        if mppt_batt_full[0] == 1.0:
            control_data_list.append(base_val + 10)

        if mppt_12_undervolt[0] == 1.0:
            control_data_list.append(base_val + 11)

        if mppt_hw_overcurrent[0] == 1.0:
            control_data_list.append(base_val + 12)

        if mppt_hw_overcurrent[0] == 1.0:
            control_data_list.append(base_val + 13)

        if mppt_hw_overvolt[0] == 1.0:
            control_data_list.append(base_val + 14)
        
        
        if mppt_ip_curr_min[0] == 1.0:
            control_data_list.append(base_val + 15)

        if mppt_ip_curr_max[0] == 1.0:
            control_data_list.append(base_val + 16)

        if mppt_op_volt_max[0] == 1.0:
            control_data_list.append(base_val + 17)

        if mppt_mosfet_temp_limit[0] == 1.0:
            control_data_list.append(base_val + 18)

        if mppt_duty_cycle_min[0] == 1.0:
            control_data_list.append(base_val + 19)

        if mppt_duty_cycle_max[0] == 1.0:
            control_data_list.append(base_val + 20)

        if mppt_local_mppt[0] == 1.0:
            control_data_list.append(base_val + 21)

        if mppt_global_mppt[0] == 1.0:
            control_data_list.append(base_val + 22)


        #MPPT_2
        if mppt_input_volt[1] < min_input_volt:
            control_data_list.append(base_val + 23)

        if mppt_input_current[1] > max_input_current:
            control_data_list.append(base_val + 24)
        

        if mppt_output_volt[1] > max_output_volt:
            control_data_list.append(base_val + 25)

        if mppt_mosfet_temp[1] > max_mosfet_temp:
            control_data_list.append(base_val + 26)

        if mppt_controller_temp[1] > max_controller_temp:
            control_data_list.append(base_val + 27)
        
        if mppt_12V_aux_supply[1] < 12 - supply_12V_tol:
            control_data_list.append(base_val + 28)

        if mppt_3V_aux_supply[1] < 3 - supply_12V_tol:
            control_data_list.append(base_val + 29)


        if mppt_low_pwr_array[1] == 1.0:
            control_data_list.append(base_val + 30)

        if mppt_mosfet_ovrht[1] == 1.0:
            control_data_list.append(base_val + 31)

        if mppt_batt_low[1] == 1.0:
            control_data_list.append(base_val + 32)

        if mppt_batt_full[1] == 1.0:
            control_data_list.append(base_val + 33)

        if mppt_12_undervolt[1] == 1.0:
            control_data_list.append(base_val + 34)

        if mppt_hw_overcurrent[1] == 1.0:
            control_data_list.append(base_val + 35)

        if mppt_hw_overcurrent[1] == 1.0:
            control_data_list.append(base_val + 36)

        if mppt_hw_overvolt[1] == 1.0:
            control_data_list.append(base_val + 37)
        
        
        if mppt_ip_curr_min[1] == 1.0:
            control_data_list.append(base_val + 38)

        if mppt_ip_curr_max[1] == 1.0:
            control_data_list.append(base_val + 39)

        if mppt_op_volt_max[1] == 1.0:
            control_data_list.append(base_val + 40)

        if mppt_mosfet_temp_limit[1] == 1.0:
            control_data_list.append(base_val + 41)

        if mppt_duty_cycle_min[1] == 1.0:
            control_data_list.append(base_val + 42)

        if mppt_duty_cycle_max[1] == 1.0:
            control_data_list.append(base_val + 43)

        if mppt_local_mppt[1] == 1.0:
            control_data_list.append(base_val + 44)

        if mppt_global_mppt[1] == 1.0:
            control_data_list.append(base_val + 45)

        #MPPT_3
        if mppt_input_volt[2] < min_input_volt:
            control_data_list.append(base_val + 46)

        if mppt_input_current[2] > max_input_current:
            control_data_list.append(base_val + 47)
        

        if mppt_output_volt[2] > max_output_volt:
            control_data_list.append(base_val + 48)

        if mppt_mosfet_temp[2] > max_mosfet_temp:
            control_data_list.append(base_val + 49)

        if mppt_controller_temp[2] > max_controller_temp:
            control_data_list.append(base_val + 50)
        
        if mppt_12V_aux_supply[2] < 12 - supply_12V_tol:
            control_data_list.append(base_val + 51)

        if mppt_3V_aux_supply[2] < 3 - supply_12V_tol:
            control_data_list.append(base_val + 52)


        if mppt_low_pwr_array[2] == 1.0:
            control_data_list.append(base_val + 53)

        if mppt_mosfet_ovrht[2] == 1.0:
            control_data_list.append(base_val + 54)

        if mppt_batt_low[2] == 1.0:
            control_data_list.append(base_val + 55)

        if mppt_batt_full[2] == 1.0:
            control_data_list.append(base_val + 56)

        if mppt_12_undervolt[2] == 1.0:
            control_data_list.append(base_val + 57)

        if mppt_hw_overcurrent[2] == 1.0:
            control_data_list.append(base_val + 58)

        if mppt_hw_overcurrent[2] == 1.0:
            control_data_list.append(base_val + 59)

        if mppt_hw_overvolt[2] == 1.0:
            control_data_list.append(base_val + 60)
        

        if mppt_ip_curr_min[2] == 1.0:
            control_data_list.append(base_val + 61)

        if mppt_ip_curr_max[2] == 1.0:
            control_data_list.append(base_val + 62)

        if mppt_op_volt_max[2] == 1.0:
            control_data_list.append(base_val + 63)

        if mppt_mosfet_temp_limit[2] == 1.0:
            control_data_list.append(base_val + 64)

        if mppt_duty_cycle_min[2] == 1.0:
            control_data_list.append(base_val + 65)

        if mppt_duty_cycle_max[2] == 1.0:
            control_data_list.append(base_val + 66)

        if mppt_local_mppt[2] == 1.0:
            control_data_list.append(base_val + 67)

        if mppt_global_mppt[2] == 1.0:
            control_data_list.append(base_val + 68)
        
        #MPPT_4
        if mppt_input_volt[3] < min_input_volt:
            control_data_list.append(base_val + 69)

        if mppt_input_current[3] > max_input_current:
            control_data_list.append(base_val + 70)
        

        if mppt_output_volt[3] > max_output_volt:
            control_data_list.append(base_val + 71)

        if mppt_mosfet_temp[3] > max_mosfet_temp:
            control_data_list.append(base_val + 72)

        if mppt_controller_temp[3] > max_controller_temp:
            control_data_list.append(base_val + 73)
        
        if mppt_12V_aux_supply[3] < 12 - supply_12V_tol:
            control_data_list.append(base_val + 74)

        if mppt_3V_aux_supply[3] < 3 - supply_12V_tol:
            control_data_list.append(base_val + 75)


        if mppt_low_pwr_array[3] == 1.0:
            control_data_list.append(base_val + 76)

        if mppt_mosfet_ovrht[3] == 1.0:
            control_data_list.append(base_val + 77)

        if mppt_batt_low[3] == 1.0:
            control_data_list.append(base_val + 78)

        if mppt_batt_full[3] == 1.0:
            control_data_list.append(base_val + 79)

        if mppt_12_undervolt[3] == 1.0:
            control_data_list.append(base_val + 80)

        if mppt_hw_overcurrent[3] == 1.0:
            control_data_list.append(base_val + 81)

        if mppt_hw_overcurrent[3] == 1.0:
            control_data_list.append(base_val + 82)

        if mppt_hw_overvolt[3] == 1.0:
            control_data_list.append(base_val + 83)
        
        
        if mppt_ip_curr_min[3] == 1.0:
            control_data_list.append(base_val + 84)

        if mppt_ip_curr_max[3] == 1.0:
            control_data_list.append(base_val + 85)

        if mppt_op_volt_max[3] == 1.0:
            control_data_list.append(base_val + 86)

        if mppt_mosfet_temp_limit[3] == 1.0:
            control_data_list.append(base_val + 87)

        if mppt_duty_cycle_min[3] == 1.0:
            control_data_list.append(base_val + 88)

        if mppt_duty_cycle_max[3] == 1.0:
            control_data_list.append(base_val + 89)

        if mppt_local_mppt[3] == 1.0:
            control_data_list.append(base_val + 90)

        if mppt_global_mppt[3] == 1.0:
            control_data_list.append(base_val + 91)

        mc_error_flag_list = [0.0]*9

        mc_error_flag_list = mc_data[0:9]
        mc_bus_current = mc_data[10]
        mc_bus_voltage = mc_data[9]
        mc_phase_c_current = mc_data[12]
        mc_phase_b_current = mc_data[11]
        mc_BEMFd = mc_data[14]
        mc_BEMFq = mc_data[13]
        mc_15_supply = mc_data[15]
        mc_3v3_supply = mc_data[17]
        mc_1v9_supply = mc_data[16]
        mc_heat_sink_temp = mc_data[19]
        mc_motor_temp = mc_data[18]
        mc_dsp_board_temp = mc_data[20]

        control_data_list = []

        base_val = 107 #first error code in the csv file for the motor controller
        #Fill in limiting values
        bus_current_limA = 1
        bus_current_limB = 2
        bus_voltage_lim = 150
        phaseC_current_limA = 1
        phaseC_current_limB = 2
        phaseB_current_limA = 1
        phaseB_current_limB = 2
        BEMFd_lim = 75
        BEMFq_lim = 75
        supply_15_tol = 2 #Power supply tolerance
        supply_3v3_tol = 1
        supply_1v9_tol = 0.1
        heat_sink_temp_lim = 50
        motor_temp_lim = 40
        dsp_board_temp_lim = 40

        err_flg = {0:0,1:1,2:2,6:3,8:4}
        rel_flags = err_flg.keys()

        #Error flags - refer datasheet

        for i in rel_flags:
            if(mc_error_flag_list[i] == 1.0):
                control_data_list.append(base_val + err_flg[i])

        #Bus current limits
        if bus_current_limB > mc_bus_current > bus_current_limA:
            control_data_list.append(base_val + 5)

        elif bus_current_limB < mc_bus_current:
            control_data_list.append(base_val + 6)

        if mc_bus_voltage > bus_voltage_lim:
            control_data_list.append(base_val + 7)

        if phaseC_current_limB > mc_phase_c_current > phaseC_current_limA:
            control_data_list.append(base_val + 8)

        elif phaseC_current_limB < mc_phase_c_current:
            control_data_list.append(base_val + 9)
        
        if phaseB_current_limB > mc_phase_b_current > phaseB_current_limA:
            control_data_list.append(base_val + 10)

        elif phaseB_current_limB < mc_phase_b_current:
            control_data_list.append(base_val + 11)
        
        #Confirm if BEMF value has to be less than or greater than limit in normal condition
        if mc_BEMFd > BEMFd_lim:
            control_data_list.append(base_val + 12)
        
        if mc_BEMFq > BEMFq_lim:
            control_data_list.append(base_val + 13)

        if 15 - supply_15_tol > mc_15_supply:
            control_data_list.append(base_val + 14)
                

        if 3.3 - supply_3v3_tol > mc_3v3_supply:
            control_data_list.append(base_val + 15)
        
        if 1.9 - supply_1v9_tol > mc_1v9_supply:
            control_data_list.append(base_val + 16)
        
        if mc_heat_sink_temp > heat_sink_temp_lim:
            control_data_list.append(base_val + 17)
        
        if mc_motor_temp > motor_temp_lim:
            control_data_list.append(base_val + 18)
        
        if mc_dsp_board_temp > dsp_board_temp_lim:
            control_data_list.append(base_val + 19)



        control_node.control_pub_data = control_data_list
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
