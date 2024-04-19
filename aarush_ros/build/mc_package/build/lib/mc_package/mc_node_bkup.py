import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray


class MC_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # MC Subscriber
    def init_mc_subscriber(self, topic):
        self.mc_subscriber = self.create_subscription(
            rosarray, topic, self.receive_mc_data, 10
        )
        self.mc_subscriber  # prevent unused variable warning
        self.mc_sub_data = None

    def receive_mc_data(self, msg):
        self.mc_sub_data = msg.data

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

    mc_node = MC_NODE("mc_node")
    mc_node.init_mc_subscriber("mc_data")
    mc_node.init_peripheral_subscriber("peripheral_data")
    mc_node.init_control_data_publisher("control_data", 1)

    while rclpy.ok():
        rclpy.spin_once(mc_node)

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

        mc_error_flag = mc_node.mc_sub_data[0]
        mc_bus_current = mc_node.mc_sub_data[1]
        mc_bus_voltage = mc_node.mc_sub_data[2]
        mc_phase_c_current = mc_node.mc_sub_data[3]
        mc_phase_b_current = mc_node.mc_sub_data[4]
        mc_BEMFd = mc_node.mc_sub_data[5]
        mc_BEMFq = mc_node.mc_sub_data[6]
        mc_15_supply = mc_node.mc_sub_data[7]
        mc_3v3_supply = mc_node.mc_sub_data[8]
        mc_1v9_supply = mc_node.mc_sub_data[9]
        mc_heat_sink_temp = mc_node.mc_sub_data[10]
        mc_motor_temp = mc_node.mc_sub_data[11]
        mc_dsp_board_temp = mc_node.mc_sub_data[12]


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

        #Error flags - refer datasheet
        if mc_error_flag in err_flg.keys():
            mc_node.control_pub_data = base_val + err_flg[mc_error_flag]
        
        #Bus current limits
        if bus_current_limB > mc_bus_current > bus_current_limA:
            mc_node.control_pub_data = base_val + 5
        elif bus_current_limB < mc_bus_current:
            mc_node.control_pub_data = base_val + 6

        if mc_bus_voltage > bus_voltage_lim:
            mc_node.control_pub_data = base_val + 7

        if phaseC_current_limB > mc_phase_c_current > phaseC_current_limA:
            mc_node.control_pub_data = base_val + 8
        elif phaseC_current_limB < mc_phase_c_current:
            mc_node.control_pub_data = base_val + 9
        
        if phaseB_current_limB > mc_phase_b_current > phaseB_current_limA:
            mc_node.control_pub_data = base_val + 10
        elif phaseB_current_limB < mc_phase_b_current:
            mc_node.control_pub_data = base_val + 11
        
        #Confirm if BEMF value has to be less than or greater than limit in normal condition
        if mc_BEMFd > BEMFd_lim:
            mc_node.control_pub_data = base_val + 12
        
        if mc_BEMFq > BEMFq_lim:
            mc_node.control_pub_data = base_val + 13

        if 15 - supply_15_tol > mc_15_supply:
            mc_node.control_pub_data = base_val + 14

        if 3.3 - supply_3v3_tol > mc_3v3_supply:
            mc_node.control_pub_data = base_val + 15
        
        if 1.9 - supply_1v9_tol > mc_1v9_supply:
            mc_node.control_pub_data = base_val + 16
        
        if mc_heat_sink_temp > heat_sink_temp_lim:
            mc_node.control_pub_data = base_val + 17
        
        if mc_motor_temp > motor_temp_lim:
            mc_node.control_pub_data = base_val + 18
        
        if mc_dsp_board_temp > dsp_board_temp_lim:
            mc_node.control_pub_data = base_val + 19

    mc_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
