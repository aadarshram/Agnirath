import sys
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5.QtGui import QPixmap, QIcon, QImage, QPalette, QBrush, QPainter, QFont
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray

class Display_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # Final Subscriber
    def init_final_subscriber(self, topic):
        self.final_subscriber = self.create_subscription(
            rosarray, topic, self.receive_final_data, 10
        )
        self.final_sub_data = None

    def receive_final_data(self, msg):
        self.final_sub_data = msg.data
        return self.final_sub_data
    
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.setFixedSize(1024, 600)

        pixmap = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Bg.png")
        self.central_widget.setAutoFillBackground(True)
        palette = self.central_widget.palette()
        palette.setBrush(QPalette.Window, QBrush(pixmap))
        self.central_widget.setPalette(palette)

def ros_main(args=None):
    print('Hi from display_package.')
    rclpy.init(args=args)
    final_node = Display_NODE("display_node")
    final_node.init_final_subscriber("final_data")
    while rclpy.ok():
        rclpy.spin_once(final_node)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    ros_thread = threading.Thread(target=ros_main)
    ros_thread.start()

    sys.exit(app.exec_())
