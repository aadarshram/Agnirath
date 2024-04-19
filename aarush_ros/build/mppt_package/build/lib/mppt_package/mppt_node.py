import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray

class MPPT_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # MPPT Subscriber
    def init_mppt_subscriber(self, topic):
        self.mppt_subscriber = self.create_subscription(
            rosarray, topic, self.receive_mppt_data, 10
        )
        self.mppt_subscriber  # prevent unused variable warning
        self.mppt_sub_data = None

    def receive_mppt_data(self, msg):
        self.mppt_sub_data = msg.data

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
            self.control_pub_data = []
            


def main(args=None):
    threshold_crossing_time = 1

    rclpy.init(args=args)

    mppt_node = MPPT_NODE("mppt_node")
    mppt_node.init_mppt_subscriber("mppt_data")
    mppt_node.init_peripheral_subscriber("peripheral_data")
    mppt_node.init_control_data_publisher("control_data", 1)

    while rclpy.ok():
        rclpy.spin_once(mppt_node)
        if mppt_node.mppt_sub_data:
            # Peripheral Status (assumed standard across all control nodes)
            # Assume we have DPSTs for MPPTs, Triple Pole Contactor from MC to Motor
            peripherals = [
                0,0,0,0,0,0,0,0, #8x MPPT
                0,0,0,0, # MPPT Precharge
                0,0,0,0,  # MC Precharge
                0,         # Motor Contactor
                0, #Battery Contactor
                        ]

            # Parsing the Data
            #list combined parameters from all mppts

            mppt_input_volt = [mppt_node.mppt_sub_data[0], mppt_node.mppt_sub_data[26], mppt_node.mppt_sub_data[52], mppt_node.mppt_sub_data[78]]
            mppt_input_current = [mppt_node.mppt_sub_data[1], mppt_node.mppt_sub_data[27], mppt_node.mppt_sub_data[53], mppt_node.mppt_sub_data[79]]
            mppt_output_volt = [mppt_node.mppt_sub_data[2], mppt_node.mppt_sub_data[28], mppt_node.mppt_sub_data[54], mppt_node.mppt_sub_data[80]]
            mppt_output_current = [mppt_node.mppt_sub_data[3], mppt_node.mppt_sub_data[29], mppt_node.mppt_sub_data[55], mppt_node.mppt_sub_data[81]]
            mppt_mosfet_temp = [mppt_node.mppt_sub_data[4], mppt_node.mppt_sub_data[30], mppt_node.mppt_sub_data[56], mppt_node.mppt_sub_data[82]]
            mppt_controller_temp = [mppt_node.mppt_sub_data[5], mppt_node.mppt_sub_data[31], mppt_node.mppt_sub_data[57], mppt_node.mppt_sub_data[83]]
            mppt_12V_aux_supply = [mppt_node.mppt_sub_data[6], mppt_node.mppt_sub_data[32], mppt_node.mppt_sub_data[58], mppt_node.mppt_sub_data[84]]
            mppt_3V_aux_supply = [mppt_node.mppt_sub_data[7], mppt_node.mppt_sub_data[33], mppt_node.mppt_sub_data[59], mppt_node.mppt_sub_data[85]]
            mppt_low_pwr_array = [mppt_node.mppt_sub_data[8], mppt_node.mppt_sub_data[34], mppt_node.mppt_sub_data[60], mppt_node.mppt_sub_data[86]]
            mppt_mosfet_ovrht = [mppt_node.mppt_sub_data[9], mppt_node.mppt_sub_data[35], mppt_node.mppt_sub_data[61], mppt_node.mppt_sub_data[87]]
            mppt_batt_low = [mppt_node.mppt_sub_data[10], mppt_node.mppt_sub_data[36], mppt_node.mppt_sub_data[62], mppt_node.mppt_sub_data[88]]
            mppt_batt_full = [mppt_node.mppt_sub_data[11], mppt_node.mppt_sub_data[37], mppt_node.mppt_sub_data[63], mppt_node.mppt_sub_data[89]]
            mppt_12_undervolt = [mppt_node.mppt_sub_data[12], mppt_node.mppt_sub_data[38], mppt_node.mppt_sub_data[64], mppt_node.mppt_sub_data[90]]
            mppt_hw_overcurrent = [mppt_node.mppt_sub_data[13], mppt_node.mppt_sub_data[39], mppt_node.mppt_sub_data[65], mppt_node.mppt_sub_data[91]]
            mppt_hw_overvolt = [mppt_node.mppt_sub_data[14], mppt_node.mppt_sub_data[40], mppt_node.mppt_sub_data[66], mppt_node.mppt_sub_data[92]]
            mppt_ip_curr_min = [mppt_node.mppt_sub_data[15], mppt_node.mppt_sub_data[41], mppt_node.mppt_sub_data[67], mppt_node.mppt_sub_data[93]]
            mppt_ip_curr_max = [mppt_node.mppt_sub_data[16], mppt_node.mppt_sub_data[42], mppt_node.mppt_sub_data[68], mppt_node.mppt_sub_data[94]]
            mppt_op_volt_max = [mppt_node.mppt_sub_data[17], mppt_node.mppt_sub_data[43], mppt_node.mppt_sub_data[69], mppt_node.mppt_sub_data[95]]
            mppt_mosfet_temp_limit = [mppt_node.mppt_sub_data[18], mppt_node.mppt_sub_data[44], mppt_node.mppt_sub_data[70], mppt_node.mppt_sub_data[96]]
            mppt_duty_cycle_min = [mppt_node.mppt_sub_data[19], mppt_node.mppt_sub_data[45], mppt_node.mppt_sub_data[71], mppt_node.mppt_sub_data[97]]
            mppt_duty_cycle_max = [mppt_node.mppt_sub_data[20], mppt_node.mppt_sub_data[46], mppt_node.mppt_sub_data[72], mppt_node.mppt_sub_data[98]]
            mppt_local_mppt = [mppt_node.mppt_sub_data[21], mppt_node.mppt_sub_data[47], mppt_node.mppt_sub_data[73], mppt_node.mppt_sub_data[99]]
            mppt_global_mppt = [mppt_node.mppt_sub_data[22], mppt_node.mppt_sub_data[48], mppt_node.mppt_sub_data[74], mppt_node.mppt_sub_data[100]]
            mppt_mode = [mppt_node.mppt_sub_data[23], mppt_node.mppt_sub_data[49], mppt_node.mppt_sub_data[75], mppt_node.mppt_sub_data[101]]
            mppt_battery_side_op_volt = [mppt_node.mppt_sub_data[24], mppt_node.mppt_sub_data[50], mppt_node.mppt_sub_data[76], mppt_node.mppt_sub_data[102]]
            mppt_power_conn_temp = [mppt_node.mppt_sub_data[25], mppt_node.mppt_sub_data[51], mppt_node.mppt_sub_data[77], mppt_node.mppt_sub_data[103]]

            error_codes = []

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
                error_codes.append(base_val + 0)

            if mppt_input_current[0] > max_input_current:
                error_codes.append(base_val + 1)
            

            if mppt_output_volt[0] > max_output_volt:
                error_codes.append(base_val + 2)

            if mppt_mosfet_temp[0] > max_mosfet_temp:
                error_codes.append(base_val + 3)

            if mppt_controller_temp[0] > max_controller_temp:
                error_codes.append(base_val + 4)
            
            if mppt_12V_aux_supply[0] < 12 - supply_12V_tol:
                error_codes.append(base_val + 5)

            if mppt_3V_aux_supply[0] < 3 - supply_12V_tol:
                error_codes.append(base_val + 6)

            if mppt_low_pwr_array[0] == 1.0:
                error_codes.append(base_val + 7)

            if mppt_mosfet_ovrht[0] == 1.0:
                error_codes.append(base_val + 8)

            if mppt_batt_low[0] == 1.0:
                error_codes.append(base_val + 9)

            if mppt_batt_full[0] == 1.0:
                error_codes.append(base_val + 10)

            if mppt_12_undervolt[0] == 1.0:
                error_codes.append(base_val + 11)

            if mppt_hw_overcurrent[0] == 1.0:
                error_codes.append(base_val + 12)

            if mppt_hw_overcurrent[0] == 1.0:
                error_codes.append(base_val + 13)

            if mppt_hw_overvolt[0] == 1.0:
                error_codes.append(base_val + 14)
            
            
            if mppt_ip_curr_min[0] == 1.0:
                error_codes.append(base_val + 15)

            if mppt_ip_curr_max[0] == 1.0:
                error_codes.append(base_val + 16)

            if mppt_op_volt_max[0] == 1.0:
                error_codes.append(base_val + 17)

            if mppt_mosfet_temp_limit[0] == 1.0:
                error_codes.append(base_val + 18)

            if mppt_duty_cycle_min[0] == 1.0:
                error_codes.append(base_val + 19)

            if mppt_duty_cycle_max[0] == 1.0:
                error_codes.append(base_val + 20)

            if mppt_local_mppt[0] == 1.0:
                error_codes.append(base_val + 21)

            if mppt_global_mppt[0] == 1.0:
                error_codes.append(base_val + 22)


            #MPPT_2
            if mppt_input_volt[1] < min_input_volt:
                error_codes.append(base_val + 23)

            if mppt_input_current[1] > max_input_current:
                error_codes.append(base_val + 24)
            

            if mppt_output_volt[1] > max_output_volt:
                error_codes.append(base_val + 25)

            if mppt_mosfet_temp[1] > max_mosfet_temp:
                error_codes.append(base_val + 26)

            if mppt_controller_temp[1] > max_controller_temp:
                error_codes.append(base_val + 27)
            
            if mppt_12V_aux_supply[1] < 12 - supply_12V_tol:
                error_codes.append(base_val + 28)

            if mppt_3V_aux_supply[1] < 3 - supply_12V_tol:
                error_codes.append(base_val + 29)


            if mppt_low_pwr_array[1] == 1.0:
                error_codes.append(base_val + 30)

            if mppt_mosfet_ovrht[1] == 1.0:
                error_codes.append(base_val + 31)

            if mppt_batt_low[1] == 1.0:
                error_codes.append(base_val + 32)

            if mppt_batt_full[1] == 1.0:
                error_codes.append(base_val + 33)

            if mppt_12_undervolt[1] == 1.0:
                error_codes.append(base_val + 34)

            if mppt_hw_overcurrent[1] == 1.0:
                error_codes.append(base_val + 35)

            if mppt_hw_overcurrent[1] == 1.0:
                error_codes.append(base_val + 36)

            if mppt_hw_overvolt[1] == 1.0:
                error_codes.append(base_val + 37)
            
            
            if mppt_ip_curr_min[1] == 1.0:
                error_codes.append(base_val + 38)

            if mppt_ip_curr_max[1] == 1.0:
                error_codes.append(base_val + 39)

            if mppt_op_volt_max[1] == 1.0:
                error_codes.append(base_val + 40)

            if mppt_mosfet_temp_limit[1] == 1.0:
                error_codes.append(base_val + 41)

            if mppt_duty_cycle_min[1] == 1.0:
                error_codes.append(base_val + 42)

            if mppt_duty_cycle_max[1] == 1.0:
                error_codes.append(base_val + 43)

            if mppt_local_mppt[1] == 1.0:
                error_codes.append(base_val + 44)

            if mppt_global_mppt[1] == 1.0:
                error_codes.append(base_val + 45)

            #MPPT_3
            if mppt_input_volt[2] < min_input_volt:
                error_codes.append(base_val + 46)

            if mppt_input_current[2] > max_input_current:
                error_codes.append(base_val + 47)
            

            if mppt_output_volt[2] > max_output_volt:
                error_codes.append(base_val + 48)

            if mppt_mosfet_temp[2] > max_mosfet_temp:
                error_codes.append(base_val + 49)

            if mppt_controller_temp[2] > max_controller_temp:
                error_codes.append(base_val + 50)
            
            if mppt_12V_aux_supply[2] < 12 - supply_12V_tol:
                error_codes.append(base_val + 51)

            if mppt_3V_aux_supply[2] < 3 - supply_12V_tol:
                error_codes.append(base_val + 52)


            if mppt_low_pwr_array[2] == 1.0:
                error_codes.append(base_val + 53)

            if mppt_mosfet_ovrht[2] == 1.0:
                error_codes.append(base_val + 54)

            if mppt_batt_low[2] == 1.0:
                error_codes.append(base_val + 55)

            if mppt_batt_full[2] == 1.0:
                error_codes.append(base_val + 56)

            if mppt_12_undervolt[2] == 1.0:
                error_codes.append(base_val + 57)

            if mppt_hw_overcurrent[2] == 1.0:
                error_codes.append(base_val + 58)

            if mppt_hw_overcurrent[2] == 1.0:
                error_codes.append(base_val + 59)

            if mppt_hw_overvolt[2] == 1.0:
                error_codes.append(base_val + 60)
            
            
            if mppt_ip_curr_min[2] == 1.0:
                error_codes.append(base_val + 61)

            if mppt_ip_curr_max[2] == 1.0:
                error_codes.append(base_val + 62)

            if mppt_op_volt_max[2] == 1.0:
                error_codes.append(base_val + 63)

            if mppt_mosfet_temp_limit[2] == 1.0:
                error_codes.append(base_val + 64)

            if mppt_duty_cycle_min[2] == 1.0:
                error_codes.append(base_val + 65)

            if mppt_duty_cycle_max[2] == 1.0:
                error_codes.append(base_val + 66)

            if mppt_local_mppt[2] == 1.0:
                error_codes.append(base_val + 67)

            if mppt_global_mppt[2] == 1.0:
                error_codes.append(base_val + 68)
            
            #MPPT_4
            if mppt_input_volt[3] < min_input_volt:
                error_codes.append(base_val + 69)

            if mppt_input_current[3] > max_input_current:
                error_codes.append(base_val + 70)
            

            if mppt_output_volt[3] > max_output_volt:
                error_codes.append(base_val + 71)

            if mppt_mosfet_temp[3] > max_mosfet_temp:
                error_codes.append(base_val + 72)

            if mppt_controller_temp[3] > max_controller_temp:
                error_codes.append(base_val + 73)
            
            if mppt_12V_aux_supply[3] < 12 - supply_12V_tol:
                error_codes.append(base_val + 74)

            if mppt_3V_aux_supply[3] < 3 - supply_12V_tol:
                error_codes.append(base_val + 75)


            if mppt_low_pwr_array[3] == 1.0:
                error_codes.append(base_val + 76)

            if mppt_mosfet_ovrht[3] == 1.0:
                error_codes.append(base_val + 77)

            if mppt_batt_low[3] == 1.0:
                error_codes.append(base_val + 78)

            if mppt_batt_full[3] == 1.0:
                error_codes.append(base_val + 79)

            if mppt_12_undervolt[3] == 1.0:
                error_codes.append(base_val + 80)

            if mppt_hw_overcurrent[3] == 1.0:
               error_codes.append(base_val + 81)

            if mppt_hw_overcurrent[3] == 1.0:
                error_codes.append(base_val + 82)

            if mppt_hw_overvolt[3] == 1.0:
                error_codes.append(base_val + 83)
            
            
            if mppt_ip_curr_min[3] == 1.0:
                error_codes.append(base_val + 84)

            if mppt_ip_curr_max[3] == 1.0:
                error_codes.append(base_val + 85)

            if mppt_op_volt_max[3] == 1.0:
                error_codes.append(base_val + 86)

            if mppt_mosfet_temp_limit[3] == 1.0:
                error_codes.append(base_val + 87)

            if mppt_duty_cycle_min[3] == 1.0:
                error_codes.append(base_val + 88)

            if mppt_duty_cycle_max[3] == 1.0:
                error_codes.append(base_val + 89)

            if mppt_local_mppt[3] == 1.0:
                error_codes.append(base_val + 90)

            if mppt_global_mppt[3] == 1.0:
                error_codes.append(base_val + 91)
            
            mppt_node.control_pub_data = error_codes


if __name__== '__main__':
    main()