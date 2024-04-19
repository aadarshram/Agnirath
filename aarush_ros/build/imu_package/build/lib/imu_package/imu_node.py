import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import smbus


class IMU_NODE(Node):
    def __init__(self, node):
        super().__init__(node)
        self.imu_node = node

    def init_imu_publisher(self,topic, timer_period, MPU_BUS_NO):

        # MPU6050 registers addresses
        self.MPU6050_ADDR = 0x68
        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.ACCEL_XOUT = 0x3B
        self.ACCEL_YOUT = 0x3D
        self.ACCEL_ZOUT = 0x3F
        self.GYRO_XOUT = 0x43
        self.GYRO_YOUT = 0x45
        self.GYRO_ZOUT = 0x47

        self.MPU_BUS_NO = MPU_BUS_NO
        self.MPU_BUS = smbus.SMBus(self.MPU_BUS_NO)
        self.MPU_BUS.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0x00)
        # Configure the accelerometer and gyroscope
        self.MPU_BUS.write_byte_data(self.MPU6050_ADDR, self.SMPLRT_DIV, 0x07)
        self.MPU_BUS.write_byte_data(self.MPU6050_ADDR, self.CONFIG, 0x06)
        self.MPU_BUS.write_byte_data(self.MPU6050_ADDR, self.GYRO_CONFIG, 0x18)
        self.MPU_BUS.write_byte_data(self.MPU6050_ADDR, self.ACCEL_CONFIG, 0x01)

        self.imu_pub = self.create_publisher(rosarray, topic, 10)
        self.imu_timer = self.create_timer(timer_period, self.publish_imu_data)
        self.imu_pub_data = []

    def read_raw_data(self, addr):
        self.high = self.MPU_BUS.read_byte_data(self.MPU6050_ADDR, addr)
        self.low = self.MPU_BUS.read_byte_data(self.MPU6050_ADDR, addr + 1)
        self.value = (self.high << 8) + self.low

        if self.value > 32767:
            self.value = self.value - 65536
        return self.value

    def get_data(self):
        self.accel_x = self.read_raw_data(self.ACCEL_XOUT)
        self.accel_y = self.read_raw_data(self.ACCEL_YOUT)
        self.accel_z = self.read_raw_data(self.ACCEL_ZOUT)
        self.gyro_x = self.read_raw_data(self.GYRO_XOUT)
        self.gyro_y = self.read_raw_data(self.GYRO_YOUT)
        self.gyro_z = self.read_raw_data(self.GYRO_ZOUT)

        self.imu_pub_data= [
            self.accel_x,
            self.accel_y,
            self.accel_z,
            self.gyro_x,
            self.gyro_y,
            self.gyro_z,
        ]

    def publish_imu_data(self):
        self.imu_pub_msg = rosarray()
        self.imu_pub_msg.data = self.imu_pub_data
        self.imu_pub.publish(self.imu_pub_msg)
        self.imu_pub_data = self.imu_pub_msg.data
        print("PUB to", self.imu_node, ":", self.imu_pub_data)
    
def main(args=None):
    rclpy.init(args=args)

    imu_node = IMU_NODE("imu")
    imu_node.init_imu_publisher("imu_data",1,2)
    
    while rclpy.ok():
        rclpy.spin_once(imu_node)

    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
