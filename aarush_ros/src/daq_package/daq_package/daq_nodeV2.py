import sys
import numpy as np
import random
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QWidget, QTextEdit, QSpacerItem, QStackedWidget, QLineEdit, QSizePolicy, QTableWidget, QTableWidgetItem, QGridLayout
from PyQt5.QtGui import QColor, QPalette, QPixmap, QIcon, QPainter, QFont, QTransform
from PyQt5.QtCore import Qt, QRect, QUrl, QTimer, QThread
from PyQt5.QtWebEngineWidgets import QWebEngineView
import pyqtgraph as pg
import datetime
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import serial


def timestamp():
    return int(time.mktime(datetime.datetime.now().timetuple()))


class TimeAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setLabel(text='Time', units=None)
        self.enableAutoSIPrefix(False)

    def tickStrings(self, values, scale, spacing):
        return [datetime.datetime.fromtimestamp(value).strftime("%H:%M:%S") for value in values]
    
class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self.node)

class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.data = [0,]*296
        print(self.data)
        self.topic_name = '/final_data'
        # self.connect_ros()

        # Set background color
        background_color = QColor(0x1C, 0x1B, 0x1A)
        palette = QPalette()
        palette.setColor(QPalette.Window, background_color)
        self.setPalette(palette)

        self.start_stack = QStackedWidget()
        self.setCentralWidget(self.start_stack)

        self.start_qwidget = QWidget()
        self.start_layout = QHBoxLayout()
        self.start_qwidget.setLayout(self.start_layout)
        self.start_stack.addWidget(self.start_qwidget)
        
        ros_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/ros.png")
        serial_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/serial.png")

        self.ros_button = QPushButton(self)
        self.ros_button.setIcon(QIcon(ros_image))
        self.ros_button.setIconSize(ros_image.size())
        self.ros_button.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.ros_button.clicked.connect(self.ros)

        self.serial_button = QPushButton(self)
        self.serial_button.setIcon(QIcon(serial_image))
        self.serial_button.setIconSize(serial_image.size())
        self.serial_button.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.serial_button.clicked.connect(self.serial)

        self.start_layout.addWidget(self.ros_button)
        self.start_layout.addWidget(self.serial_button)

        self.central_widget = QWidget()
        # self.setCentralWidget(self.central_widget)
        main_layout = QHBoxLayout()
        self.central_widget.setLayout(main_layout)
        self.start_stack.addWidget(self.central_widget)

        #BUTTONS VBoxlayout
        button_layout = QVBoxLayout()
        main_layout.addLayout(button_layout)
        button1_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/home.png")
        button2_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/volt_amps.png")
        button3_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Temps.png")
        button4_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/cabin.png")
        button5_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/circuit.png")

        self.button1 = QPushButton(self)
        self.button1.setIcon(QIcon(button1_image))
        self.button1.setIconSize(button1_image.size())
        self.button1.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button1.clicked.connect(self.home)

        self.button2 = QPushButton(self)
        self.button2.setIcon(QIcon(button2_image))
        self.button2.setIconSize(button2_image.size())
        self.button2.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button2.clicked.connect(self.voltamps)

        self.button3 = QPushButton(self)
        self.button3.setIcon(QIcon(button3_image))
        self.button3.setIconSize(button3_image.size())
        self.button3.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button3.clicked.connect(self.temps)

        self.button4 = QPushButton(self)
        self.button4.setIcon(QIcon(button4_image))
        self.button4.setIconSize(button4_image.size())
        self.button4.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button4.clicked.connect(self.cabin)

        self.button5 = QPushButton(self)
        self.button5.setIcon(QIcon(button5_image))
        self.button5.setIconSize(button5_image.size())
        self.button5.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button5.clicked.connect(self.circuit)

        self.error = QTextEdit()
        self.error.setReadOnly(True)
        self.error.setFixedSize(320, 320)
        
        button_layout.addWidget(self.button1)
        button_layout.addWidget(self.button2)
        button_layout.addWidget(self.button3)
        button_layout.addWidget(self.button4)
        button_layout.addWidget(self.button5)
        button_layout.addWidget(self.error)

        self.timer1 = QTimer(self)
        self.timer1.timeout.connect(self.display_widget)
        self.timer1.start(1000) 

        #Widgets & Layouts
        self.home_qwidget = QWidget()
        self.home_layout = QHBoxLayout()
        self.home_qwidget.setLayout(self.home_layout)
        self.solar_qwidget = QWidget()
        self.solar_layout = QVBoxLayout()
        self.solar_qwidget.setLayout(self.solar_layout)
        self.battery_qwidget = QWidget()
        self.battery_layout = QVBoxLayout()
        self.battery_qwidget.setLayout(self.battery_layout)
        self.mc_qwidget = QWidget()
        self.mc_layout = QVBoxLayout()
        self.mc_qwidget.setLayout(self.mc_layout)
        self.telemetry_qwidget = QWidget()
        self.telemetry_layout = QVBoxLayout()
        self.telemetry_qwidget.setLayout(self.telemetry_layout)
        self.drive_qwidget = QWidget()
        self.drive_layout = QHBoxLayout()
        self.drive_qwidget.setLayout(self.drive_layout)
        self.strategy_qwidget = QWidget()
        self.strategy_layout = QHBoxLayout()
        self.strategy_qwidget.setLayout(self.strategy_layout)
        self.voltamps_qwidget = QWidget()
        self.voltamps_layout = QVBoxLayout()
        self.voltamps_qwidget.setLayout(self.voltamps_layout)
        self.temps_qwidget = QWidget()
        self.temps_layout = QGridLayout()
        self.temps_qwidget.setLayout(self.temps_layout)
        self.cabin_qwidget = QWidget()
        self.cabin_layout = QVBoxLayout()
        self.cabin_qwidget.setLayout(self.cabin_layout)
        self.circuit_qwidget = QWidget()
        self.circuit_layout = QHBoxLayout()
        self.circuit_qwidget.setLayout(self.circuit_layout)        
        
        #HOME LAYOUT
        #Map and 3 widgets
        self.map_layout = QVBoxLayout()
        # h_spacer2 = QSpacerItem(70, 10) 
        # main_layout.addItem(h_spacer2)
        self.home_layout.addLayout(self.map_layout)
        url = "http://127.0.0.1:5500/GUI/Beaglebone/Dashboard/mapV3.html"  
        self.load_html_file(url, self.map_layout)
        
        Hwidgets_layout = QHBoxLayout()
        self.map_layout.addLayout(Hwidgets_layout)
        # h_spacer1 = QSpacerItem(70, 10) 
        # Hwidgets_layout.addItem(h_spacer1)

        self.telemetry_widget = QLabel()
        self.drive_widget = QLabel()
        self.strategy_widget = QLabel()

        Hwidgets_layout.addWidget(self.telemetry_widget)
        Hwidgets_layout.addWidget(self.drive_widget)
        Hwidgets_layout.addWidget(self.strategy_widget)

        pixmap_telemetry = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Telemetry_widget.png")
        self.telemetry_widget.setPixmap(pixmap_telemetry.scaled(320, 320))
        pixmap_drive = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Drive_widget.png")
        self.drive_widget.setPixmap(pixmap_drive.scaled(320, 320))
        pixmap_strategy = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Startegy_widget.png")
        self.strategy_widget.setPixmap(pixmap_strategy.scaled(320, 320))        

        # 3 Widgets
        Vwidgets_layout = QVBoxLayout()
        self.home_layout.addLayout(Vwidgets_layout)

        self.pw_out = 0
        self.temp = 0

        self.solar_widget = QLabel()
        self.battery_widget = QLabel()
        self.mc_widget = QLabel()

        Vwidgets_layout.addWidget(self.solar_widget)
        Vwidgets_layout.addWidget(self.battery_widget)
        Vwidgets_layout.addWidget(self.mc_widget)

        pixmap_solar = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Solar_widget.png")
        self.solar_widget.setPixmap(pixmap_solar.scaled(320, 320))
        pixmap_battery = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Battery_widget.png")
        self.battery_widget.setPixmap(pixmap_battery.scaled(320, 320))
        pixmap_MC = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/MC_widget.png")
        self.mc_widget.setPixmap(pixmap_MC.scaled(320, 320))

        #SOLAR LAYOUT
        mppt1graph_layout = QHBoxLayout()
        self.solar_layout.addLayout(mppt1graph_layout)
        self.mppt1_input_plot = pg.PlotWidget(
            title="MPPT-1 INPUT VOLTAGE",
            labels={'left': 'Input Voltage'},
            axisItems={'bottom': TimeAxisItem(orientation='bottom')}
        )
        self.mppt1_output_plot = pg.PlotWidget(
            title="MPPT-1 OUTPUT POWER",
            labels={'left': 'Output Power'},
            axisItems={'bottom': TimeAxisItem(orientation='bottom')}
        )
        mppt1graph_layout.addWidget(self.mppt1_input_plot)
        mppt1graph_layout.addWidget(self.mppt1_output_plot)
        self.mppt1_input_data = {'x': [], 'y': []}
        self.mppt1_output_data = {'x': [], 'y': []}

        mppt2graph_layout = QHBoxLayout()
        self.solar_layout.addLayout(mppt2graph_layout)
        self.mppt2_input_plot = pg.PlotWidget(
            title="MPPT-2 INPUT VOLTAGE",
            labels={'left': 'Input Voltage'},
            axisItems={'bottom': TimeAxisItem(orientation='bottom')}
        )
        self.mppt2_output_plot = pg.PlotWidget(
            title="MPPT-2 OUTPUT POWER",
            labels={'left': 'Output Power'},
            axisItems={'bottom': TimeAxisItem(orientation='bottom')}
        )
        mppt2graph_layout.addWidget(self.mppt2_input_plot)
        mppt2graph_layout.addWidget(self.mppt2_output_plot)
        self.mppt2_input_data = {'x': [], 'y': []}
        self.mppt2_output_data = {'x': [], 'y': []}

        mppt3graph_layout = QHBoxLayout()
        self.solar_layout.addLayout(mppt3graph_layout)
        self.mppt3_input_plot = pg.PlotWidget(
            title="MPPT-3 INPUT VOLTAGE",
            labels={'left': 'Input Voltage'},
            axisItems={'bottom': TimeAxisItem(orientation='bottom')}
        )
        self.mppt3_output_plot = pg.PlotWidget(
            title="MPPT-3 OUTPUT POWER",
            labels={'left': 'Output Power'},
            axisItems={'bottom': TimeAxisItem(orientation='bottom')}
        )
        mppt3graph_layout.addWidget(self.mppt3_input_plot)
        mppt3graph_layout.addWidget(self.mppt3_output_plot)
        self.mppt3_input_data = {'x': [], 'y': []}
        self.mppt3_output_data = {'x': [], 'y': []}

        mppt4graph_layout = QHBoxLayout()
        self.solar_layout.addLayout(mppt4graph_layout)
        self.mppt4_input_plot = pg.PlotWidget(
            title="MPPT-4 INPUT VOLTAGE",
            labels={'left': 'Input Voltage'},
            axisItems={'bottom': TimeAxisItem(orientation='bottom')}
        )
        self.mppt4_output_plot = pg.PlotWidget(
            title="MPPT-4 OUTPUT POWER",
            labels={'left': 'Output Power'},
            axisItems={'bottom': TimeAxisItem(orientation='bottom')}
        )
        mppt4graph_layout.addWidget(self.mppt4_input_plot)
        mppt4graph_layout.addWidget(self.mppt4_output_plot)
        self.mppt4_input_data = {'x': [], 'y': []}
        self.mppt4_output_data = {'x': [], 'y': []}

        mpptdata0_layout = QHBoxLayout()
        self.solar_layout.addLayout(mpptdata0_layout)
        self.mppt1_mode = QLabel()
        self.mppt1_mode.setText("Mode")
        self.mppt1_mode.setStyleSheet("color: white;")
        mpptdata0_layout.addWidget(self.mppt1_mode)
        self.mppt1_modeval = QLineEdit(self)
        self.mppt1_modeval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt1_modeval)
        self.mppt2_modeval = QLineEdit(self)
        self.mppt2_modeval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt2_modeval)
        self.mppt3_modeval = QLineEdit(self)
        self.mppt3_modeval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt3_modeval)
        self.mppt4_modeval = QLineEdit(self)
        self.mppt4_modeval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt4_modeval)
        self.mppt1_limits = QLabel()
        self.mppt1_limits.setText("Limits")
        self.mppt1_limits.setStyleSheet("color: white;")
        mpptdata0_layout.addWidget(self.mppt1_limits)
        self.mppt1_limitsval = QLineEdit(self)
        self.mppt1_limitsval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt1_limitsval)
        self.mppt2_limitsval = QLineEdit(self)
        self.mppt2_limitsval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt2_limitsval)
        self.mppt3_limitsval = QLineEdit(self)
        self.mppt3_limitsval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt3_limitsval)
        self.mppt4_limitsval = QLineEdit(self)
        self.mppt4_limitsval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt4_limitsval)
        self.mppt1_error = QLabel()
        self.mppt1_error.setText("Status / Error")
        self.mppt1_error.setStyleSheet("color: white;")
        mpptdata0_layout.addWidget(self.mppt1_error)
        self.mppt1_errorval = QLineEdit(self)
        self.mppt1_errorval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt1_errorval)
        self.mppt2_errorval = QLineEdit(self)
        self.mppt2_errorval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt2_errorval)
        self.mppt3_errorval = QLineEdit(self)
        self.mppt3_errorval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt3_errorval)
        self.mppt4_errorval = QLineEdit(self)
        self.mppt4_errorval.setReadOnly(True)
        mpptdata0_layout.addWidget(self.mppt4_errorval)
        
        mpptdata_layout =  QHBoxLayout()
        self.solar_layout.addLayout(mpptdata_layout)
        input_layout = QVBoxLayout()
        mpptdata_layout.addLayout(input_layout)
        self.mppt1_input = QLabel()
        self.mppt1_input.setText("Input")
        self.mppt1_input.setStyleSheet("color: white;")
        input_layout.addWidget(self.mppt1_input)
        self.mppt1_input_volt = QLabel()
        self.mppt1_input_volt.setText("Input Voltage")
        self.mppt1_input_volt.setStyleSheet("color: white;")
        input_layout.addWidget(self.mppt1_input_volt)
        self.mppt1_input_current = QLabel()
        self.mppt1_input_current.setText("Input Current")
        self.mppt1_input_current.setStyleSheet("color: white;")
        input_layout.addWidget(self.mppt1_input_current)
        self.mppt1_input_power = QLabel()
        self.mppt1_input_power.setText("Input Power")
        self.mppt1_input_power.setStyleSheet("color: white;")
        input_layout.addWidget(self.mppt1_input_power)

        inputval_layout = QVBoxLayout()
        mpptdata_layout.addLayout(inputval_layout)
        spacer = QSpacerItem(10, 30) 
        inputval_layout.addItem(spacer)
        self.mppt1_input_voltval = QLineEdit(self)
        self.mppt1_input_voltval.setReadOnly(True)
        inputval_layout.addWidget(self.mppt1_input_voltval)
        self.mppt1_input_currentval = QLineEdit(self)
        self.mppt1_input_currentval.setReadOnly(True)
        inputval_layout.addWidget(self.mppt1_input_currentval)
        self.mppt1_input_powerval = QLineEdit(self)
        self.mppt1_input_powerval.setReadOnly(True)
        inputval_layout.addWidget(self.mppt1_input_powerval)

        inputval_layout2 = QVBoxLayout()
        mpptdata_layout.addLayout(inputval_layout2)
        spacer = QSpacerItem(10, 30) 
        inputval_layout2.addItem(spacer)
        self.mppt2_input_voltval = QLineEdit(self)
        self.mppt2_input_voltval.setReadOnly(True)
        inputval_layout2.addWidget(self.mppt2_input_voltval)
        self.mppt2_input_currentval = QLineEdit(self)
        self.mppt2_input_currentval.setReadOnly(True)
        inputval_layout2.addWidget(self.mppt2_input_currentval)
        self.mppt2_input_powerval = QLineEdit(self)
        self.mppt2_input_powerval.setReadOnly(True)
        inputval_layout2.addWidget(self.mppt2_input_powerval)

        inputval_layout3 = QVBoxLayout()
        mpptdata_layout.addLayout(inputval_layout3)
        spacer = QSpacerItem(10, 30) 
        inputval_layout3.addItem(spacer)
        self.mppt3_input_voltval = QLineEdit(self)
        self.mppt3_input_voltval.setReadOnly(True)
        inputval_layout3.addWidget(self.mppt3_input_voltval)
        self.mppt3_input_currentval = QLineEdit(self)
        self.mppt3_input_currentval.setReadOnly(True)
        inputval_layout3.addWidget(self.mppt3_input_currentval)
        self.mppt3_input_powerval = QLineEdit(self)
        self.mppt3_input_powerval.setReadOnly(True)
        inputval_layout3.addWidget(self.mppt3_input_powerval)

        inputval_layout4 = QVBoxLayout()
        mpptdata_layout.addLayout(inputval_layout4)
        spacer = QSpacerItem(10, 30) 
        inputval_layout4.addItem(spacer)
        self.mppt4_input_voltval = QLineEdit(self)
        self.mppt4_input_voltval.setReadOnly(True)
        inputval_layout4.addWidget(self.mppt4_input_voltval)
        self.mppt4_input_currentval = QLineEdit(self)
        self.mppt4_input_currentval.setReadOnly(True)
        inputval_layout4.addWidget(self.mppt4_input_currentval)
        self.mppt4_input_powerval = QLineEdit(self)
        self.mppt4_input_powerval.setReadOnly(True)
        inputval_layout4.addWidget(self.mppt4_input_powerval)

        output_layout = QVBoxLayout()
        mpptdata_layout.addLayout(output_layout)
        self.mppt1_output = QLabel()
        self.mppt1_output.setText("Output")
        self.mppt1_output.setStyleSheet("color: white;")
        output_layout.addWidget(self.mppt1_output)
        self.mppt1_output_volt = QLabel()
        self.mppt1_output_volt.setText("Output Voltage")
        self.mppt1_output_volt.setStyleSheet("color: white;")
        output_layout.addWidget(self.mppt1_output_volt)
        self.mppt1_output_current = QLabel()
        self.mppt1_output_current.setText("Output Current")
        self.mppt1_output_current.setStyleSheet("color: white;")
        output_layout.addWidget(self.mppt1_output_current)
        self.mppt1_output_power = QLabel()
        self.mppt1_output_power.setText("Output Power")
        self.mppt1_output_power.setStyleSheet("color: white;")
        output_layout.addWidget(self.mppt1_output_power)

        outputval_layout = QVBoxLayout()
        mpptdata_layout.addLayout(outputval_layout)
        spacer = QSpacerItem(10, 30) 
        outputval_layout.addItem(spacer)
        self.mppt1_output_voltval = QLineEdit(self)
        self.mppt1_output_voltval.setReadOnly(True)
        outputval_layout.addWidget(self.mppt1_output_voltval)
        self.mppt1_output_currentval = QLineEdit(self)
        self.mppt1_output_currentval.setReadOnly(True)
        outputval_layout.addWidget(self.mppt1_output_currentval)
        self.mppt1_output_powerval = QLineEdit(self)
        self.mppt1_output_powerval.setReadOnly(True)
        outputval_layout.addWidget(self.mppt1_output_powerval)

        outputval_layout2 = QVBoxLayout()
        mpptdata_layout.addLayout(outputval_layout2)
        spacer = QSpacerItem(10, 30) 
        outputval_layout2.addItem(spacer)
        self.mppt2_output_voltval = QLineEdit(self)
        self.mppt2_output_voltval.setReadOnly(True)
        outputval_layout2.addWidget(self.mppt2_output_voltval)
        self.mppt2_output_currentval = QLineEdit(self)
        self.mppt2_output_currentval.setReadOnly(True)
        outputval_layout2.addWidget(self.mppt2_output_currentval)
        self.mppt2_output_powerval = QLineEdit(self)
        self.mppt2_output_powerval.setReadOnly(True)
        outputval_layout2.addWidget(self.mppt2_output_powerval)

        outputval_layout3 = QVBoxLayout()
        mpptdata_layout.addLayout(outputval_layout3)
        spacer = QSpacerItem(10, 30) 
        outputval_layout3.addItem(spacer)
        self.mppt3_output_voltval = QLineEdit(self)
        self.mppt3_output_voltval.setReadOnly(True)
        outputval_layout3.addWidget(self.mppt3_output_voltval)
        self.mppt3_output_currentval = QLineEdit(self)
        self.mppt3_output_currentval.setReadOnly(True)
        outputval_layout3.addWidget(self.mppt3_output_currentval)
        self.mppt3_output_powerval = QLineEdit(self)
        self.mppt3_output_powerval.setReadOnly(True)
        outputval_layout3.addWidget(self.mppt3_output_powerval)

        outputval_layout4 = QVBoxLayout()
        mpptdata_layout.addLayout(outputval_layout4)
        spacer = QSpacerItem(10, 30) 
        outputval_layout4.addItem(spacer)
        self.mppt4_output_voltval = QLineEdit(self)
        self.mppt4_output_voltval.setReadOnly(True)
        outputval_layout4.addWidget(self.mppt4_output_voltval)
        self.mppt4_output_currentval = QLineEdit(self)
        self.mppt4_output_currentval.setReadOnly(True)
        outputval_layout4.addWidget(self.mppt4_output_currentval)
        self.mppt4_output_powerval = QLineEdit(self)
        self.mppt4_output_powerval.setReadOnly(True)
        outputval_layout4.addWidget(self.mppt4_output_powerval)

        temps_layout = QVBoxLayout()
        mpptdata_layout.addLayout(temps_layout)
        self.mppt1_temps = QLabel()
        self.mppt1_temps.setText("Temps")
        self.mppt1_temps.setStyleSheet("color: white;")
        temps_layout.addWidget(self.mppt1_temps)
        self.mppt1_mosfet = QLabel()
        self.mppt1_mosfet.setText("Mosfet")
        self.mppt1_mosfet.setStyleSheet("color: white;")
        temps_layout.addWidget(self.mppt1_mosfet)
        self.mppt1_cntrl = QLabel()
        self.mppt1_cntrl.setText("Controller")
        self.mppt1_cntrl.setStyleSheet("color: white;")
        temps_layout.addWidget(self.mppt1_cntrl)
        self.mppt1_pwrcntrl = QLabel()
        self.mppt1_pwrcntrl.setText("Power Controller")
        self.mppt1_pwrcntrl.setStyleSheet("color: white;")
        temps_layout.addWidget(self.mppt1_pwrcntrl)
        
        tempsval_layout = QVBoxLayout()
        mpptdata_layout.addLayout(tempsval_layout)
        spacer = QSpacerItem(10, 30)  
        tempsval_layout.addItem(spacer)
        self.mppt1_mosfetval = QLineEdit(self)
        self.mppt1_mosfetval.setReadOnly(True)
        tempsval_layout.addWidget(self.mppt1_mosfetval)
        self.mppt1_cntrlval = QLineEdit(self)
        self.mppt1_cntrlval.setReadOnly(True)
        tempsval_layout.addWidget(self.mppt1_cntrlval)
        self.mppt1_pwrcntrlval = QLineEdit(self)
        self.mppt1_pwrcntrlval.setReadOnly(True)
        tempsval_layout.addWidget(self.mppt1_pwrcntrlval)
        
        tempsval_layout2 = QVBoxLayout()
        mpptdata_layout.addLayout(tempsval_layout2)
        spacer = QSpacerItem(10, 30)   
        tempsval_layout2.addItem(spacer)
        self.mppt2_mosfetval = QLineEdit(self)
        self.mppt2_mosfetval.setReadOnly(True)
        tempsval_layout2.addWidget(self.mppt2_mosfetval)
        self.mppt2_cntrlval = QLineEdit(self)
        self.mppt2_cntrlval.setReadOnly(True)
        tempsval_layout2.addWidget(self.mppt2_cntrlval)
        self.mppt2_pwrcntrlval = QLineEdit(self)
        self.mppt2_pwrcntrlval.setReadOnly(True)
        tempsval_layout2.addWidget(self.mppt2_pwrcntrlval)
        
        tempsval_layout3 = QVBoxLayout()
        mpptdata_layout.addLayout(tempsval_layout3)
        spacer = QSpacerItem(10, 30)   
        tempsval_layout3.addItem(spacer)
        self.mppt3_mosfetval = QLineEdit(self)
        self.mppt3_mosfetval.setReadOnly(True)
        tempsval_layout3.addWidget(self.mppt3_mosfetval)
        self.mppt3_cntrlval = QLineEdit(self)
        self.mppt3_cntrlval.setReadOnly(True)
        tempsval_layout3.addWidget(self.mppt3_cntrlval)
        self.mppt3_pwrcntrlval = QLineEdit(self)
        self.mppt3_pwrcntrlval.setReadOnly(True)
        tempsval_layout3.addWidget(self.mppt3_pwrcntrlval)
        
        tempsval_layout4 = QVBoxLayout()
        mpptdata_layout.addLayout(tempsval_layout4)
        spacer = QSpacerItem(10, 30)  
        tempsval_layout4.addItem(spacer)
        self.mppt4_mosfetval = QLineEdit(self)
        self.mppt4_mosfetval.setReadOnly(True)
        tempsval_layout4.addWidget(self.mppt4_mosfetval)
        self.mppt4_cntrlval = QLineEdit(self)
        self.mppt4_cntrlval.setReadOnly(True)
        tempsval_layout4.addWidget(self.mppt4_cntrlval)
        self.mppt4_pwrcntrlval = QLineEdit(self)
        self.mppt4_pwrcntrlval.setReadOnly(True)
        tempsval_layout4.addWidget(self.mppt4_pwrcntrlval)

        othervolt_layout = QVBoxLayout()
        mpptdata_layout.addLayout(othervolt_layout)
        self.mppt1_othervolt = QLabel()
        self.mppt1_othervolt.setText("Other Voltages")
        self.mppt1_othervolt.setStyleSheet("color: white;")
        othervolt_layout.addWidget(self.mppt1_othervolt)
        self.mppt1_aux12volt = QLabel()
        self.mppt1_aux12volt.setText("Aux.Twelve Volt")
        self.mppt1_aux12volt.setStyleSheet("color: white;")
        othervolt_layout.addWidget(self.mppt1_aux12volt)
        self.mppt1_aux3volt = QLabel()
        self.mppt1_aux3volt.setText("Aux.Three Volt")
        self.mppt1_aux3volt.setStyleSheet("color: white;")
        othervolt_layout.addWidget(self.mppt1_aux3volt)
        self.mppt1_output_battery = QLabel()
        self.mppt1_output_battery.setText("Output Battery Side")
        self.mppt1_output_battery.setStyleSheet("color: white;")
        othervolt_layout.addWidget(self.mppt1_output_battery)
        
        othervoltval_layout = QVBoxLayout()
        mpptdata_layout.addLayout(othervoltval_layout)
        spacer = QSpacerItem(10, 30) 
        othervoltval_layout.addItem(spacer)
        self.mppt1_aux12val = QLineEdit(self)
        self.mppt1_aux12val.setReadOnly(True)
        othervoltval_layout.addWidget(self.mppt1_aux12val)
        self.mppt1_aux3val = QLineEdit(self)
        self.mppt1_aux3val.setReadOnly(True)
        othervoltval_layout.addWidget(self.mppt1_aux3val)
        self.mppt1_output_batteryval = QLineEdit(self)
        self.mppt1_output_batteryval.setReadOnly(True)
        othervoltval_layout.addWidget(self.mppt1_output_batteryval)
        
        othervoltval_layout2 = QVBoxLayout()
        mpptdata_layout.addLayout(othervoltval_layout2)
        spacer = QSpacerItem(10, 30) 
        othervoltval_layout2.addItem(spacer)
        self.mppt2_aux12val = QLineEdit(self)
        self.mppt2_aux12val.setReadOnly(True)
        othervoltval_layout2.addWidget(self.mppt2_aux12val)
        self.mppt2_aux3val = QLineEdit(self)
        self.mppt2_aux3val.setReadOnly(True)
        othervoltval_layout2.addWidget(self.mppt2_aux3val)
        self.mppt2_output_batteryval = QLineEdit(self)
        self.mppt2_output_batteryval.setReadOnly(True)
        othervoltval_layout2.addWidget(self.mppt2_output_batteryval)
        
        othervoltval_layout3 = QVBoxLayout()
        mpptdata_layout.addLayout(othervoltval_layout3)
        spacer = QSpacerItem(10, 30) 
        othervoltval_layout3.addItem(spacer)
        self.mppt3_aux12val = QLineEdit(self)
        self.mppt3_aux12val.setReadOnly(True)
        othervoltval_layout3.addWidget(self.mppt3_aux12val)
        self.mppt3_aux3val = QLineEdit(self)
        self.mppt3_aux3val.setReadOnly(True)
        othervoltval_layout3.addWidget(self.mppt3_aux3val)
        self.mppt3_output_batteryval = QLineEdit(self)
        self.mppt3_output_batteryval.setReadOnly(True)
        othervoltval_layout3.addWidget(self.mppt3_output_batteryval)
        
        othervoltval_layout4 = QVBoxLayout()
        mpptdata_layout.addLayout(othervoltval_layout4)
        spacer = QSpacerItem(10, 30) 
        othervoltval_layout4.addItem(spacer)
        self.mppt4_aux12val = QLineEdit(self)
        self.mppt4_aux12val.setReadOnly(True)
        othervoltval_layout4.addWidget(self.mppt4_aux12val)
        self.mppt4_aux3val = QLineEdit(self)
        self.mppt4_aux3val.setReadOnly(True)
        othervoltval_layout4.addWidget(self.mppt4_aux3val)
        self.mppt4_output_batteryval = QLineEdit(self)
        self.mppt4_output_batteryval.setReadOnly(True)
        othervoltval_layout4.addWidget(self.mppt4_output_batteryval)


        #MC LAYOUT
        mcgraph_layout = QHBoxLayout()
        self.mc_layout.addLayout(mcgraph_layout)
        self.buspwr_plot = pg.PlotWidget(
            title="Bus Power (W)",
            labels={'left': 'W'},
            axisItems={'bottom': TimeAxisItem(orientation='bottom')}
        )
        self.velocity_plot = pg.PlotWidget(
            title="Velocity (m/s)",
            labels={'left': 'm/s'},
            axisItems={'bottom': TimeAxisItem(orientation='bottom')}
        )
        mcgraph_layout.addWidget(self.buspwr_plot)
        mcgraph_layout.addWidget(self.velocity_plot)
        self.buspwr_data = {'x': [], 'y': []}
        self.velocity_data = {'x': [], 'y': []}

        mcdata0_layout = QHBoxLayout()
        self.mc_layout.addLayout(mcdata0_layout)
        self.act_motor = QLabel()
        self.act_motor.setText("Active Motor")
        self.act_motor.setStyleSheet("font-family: 'Red Rose'; color: white; font-weight: bold; font-size: 25px;")
        mcdata0_layout.addWidget(self.act_motor,Qt.AlignCenter)
        self.act_motorval = QLineEdit(self)
        self.act_motorval.setFont(QFont('Red Rose',25))
        self.act_motorval.setReadOnly(True)
        mcdata0_layout.addWidget(self.act_motorval,Qt.AlignCenter)
        self.tri_id = QLabel()
        self.tri_id.setText("Tritium ID")
        self.tri_id.setStyleSheet("font-family: 'Red Rose'; color: white; font-weight: bold; font-size: 25px;")
        mcdata0_layout.addWidget(self.tri_id,Qt.AlignCenter)
        self.tri_idval = QLineEdit(self)
        self.tri_idval.setFont(QFont('Red Rose',25))
        self.tri_idval.setReadOnly(True)
        mcdata0_layout.addWidget(self.tri_idval,Qt.AlignCenter)
        self.serial_no = QLabel()
        self.serial_no.setText("Serial No.")
        self.serial_no.setStyleSheet("font-family: 'Red Rose'; color: white; font-weight: bold; font-size: 25px;")
        mcdata0_layout.addWidget(self.serial_no,Qt.AlignCenter)
        self.serial_noval = QLineEdit(self)
        self.serial_noval.setFont(QFont('Red Rose',25))
        self.serial_noval.setReadOnly(True)
        mcdata0_layout.addWidget(self.serial_noval,Qt.AlignCenter)

        mcdata_layout =  QHBoxLayout()
        self.mc_layout.addLayout(mcdata_layout,2)
        col1_layout = QVBoxLayout()
        mcdata_layout.addLayout(col1_layout)        
        # spacer = QSpacerItem(10, 30) 
        # col1_layout.addItem(spacer)
        self.power = QLabel()
        self.power.setText("Power")
        self.power.setStyleSheet("font-family: 'Red Rose';color: white; font-weight: bold; font-size: 30px;")
        col1_layout.addWidget(self.power,Qt.AlignCenter)
        self.bus_volt = QLabel()
        self.bus_volt.setText("Bus Voltage")
        self.bus_volt.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col1_layout.addWidget(self.bus_volt,Qt.AlignCenter)
        self.bus_curr = QLabel()
        self.bus_curr.setText("Bus Current")
        self.bus_curr.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col1_layout.addWidget(self.bus_curr,Qt.AlignCenter)
        self.bus_pwr = QLabel()
        self.bus_pwr.setText("Bus Power")
        self.bus_pwr.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col1_layout.addWidget(self.bus_pwr,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 30) 
        # col1_layout.addItem(spacer)
        self.phase_curr = QLabel()
        self.phase_curr.setText("Phase Current")
        self.phase_curr.setStyleSheet("color: white; font-weight: bold;font-family: 'Red Rose';font-size: 30px;")
        col1_layout.addWidget(self.phase_curr,Qt.AlignCenter)
        self.rms_c = QLabel()
        self.rms_c.setText("RMS Current C")
        self.rms_c.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col1_layout.addWidget(self.rms_c,Qt.AlignCenter)
        self.rms_b = QLabel()
        self.rms_b.setText("RMS Current B")
        self.rms_b.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col1_layout.addWidget(self.rms_b,Qt.AlignCenter)

        col2_layout = QVBoxLayout()
        mcdata_layout.addLayout(col2_layout)
        spacer = QSpacerItem(10, 60)  
        col2_layout.addItem(spacer)
        self.bus_voltval = QLineEdit(self)
        self.bus_voltval.setFont(QFont('Red Rose',25))
        self.bus_voltval.setReadOnly(True)
        col2_layout.addWidget(self.bus_voltval,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 1)  
        # col2_layout.addItem(spacer)
        self.bus_currval = QLineEdit(self)
        self.bus_currval.setFont(QFont('Red Rose',25))
        self.bus_currval.setReadOnly(True)
        col2_layout.addWidget(self.bus_currval,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 1)  
        # col2_layout.addItem(spacer)
        self.bus_pwrval = QLineEdit(self)
        self.bus_pwrval.setFont(QFont('Red Rose',25))
        self.bus_pwrval.setReadOnly(True)
        col2_layout.addWidget(self.bus_pwrval,Qt.AlignCenter)
        spacer = QSpacerItem(10, 60)  
        col2_layout.addItem(spacer)
        self.rms_cval = QLineEdit(self)
        self.rms_cval.setFont(QFont('Red Rose',25))
        self.rms_cval.setReadOnly(True)
        col2_layout.addWidget(self.rms_cval,Qt.AlignCenter)
        self.rms_bval = QLineEdit(self)
        self.rms_bval.setFont(QFont('Red Rose',25))
        self.rms_bval.setReadOnly(True)
        col2_layout.addWidget(self.rms_bval,Qt.AlignCenter)

        col3_layout = QVBoxLayout()
        mcdata_layout.addLayout(col3_layout)
        # spacer = QSpacerItem(10, 30) 
        # col3_layout.addItem(spacer)
        self.velocity = QLabel()
        self.velocity.setText("Velocity")
        self.velocity.setStyleSheet("color: white; font-weight: bold;font-family: 'Red Rose';font-size: 30px;")
        col3_layout.addWidget(self.velocity,Qt.AlignCenter)
        # spacer = QSpacerItem(10, -18) 
        # col3_layout.addItem(spacer)
        self.rpm = QLabel()
        self.rpm.setText("rpm")
        self.rpm.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col3_layout.addWidget(self.rpm,Qt.AlignCenter)
        # spacer = QSpacerItem(10, -11)  
        # col3_layout.addItem(spacer)
        self.kph = QLabel()
        self.kph.setText("kph")
        self.kph.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col3_layout.addWidget(self.kph,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 70) 
        # col3_layout.addItem(spacer)
        self.bemf = QLabel()
        self.bemf.setText("Motor BEMF")
        self.bemf.setStyleSheet("color: white; font-weight: bold;font-family: 'Red Rose';font-size: 30px;")
        col3_layout.addWidget(self.bemf,Qt.AlignCenter)
        self.vd = QLabel()
        self.vd.setText("Vd")
        self.vd.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col3_layout.addWidget(self.vd,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 30) 
        # col3_layout.addItem(spacer)
        self.vq = QLabel()
        self.vq.setText("Vq")
        self.vq.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col3_layout.addWidget(self.vq,Qt.AlignCenter)

        col4_layout = QVBoxLayout()
        mcdata_layout.addLayout(col4_layout)
        spacer = QSpacerItem(10, 80)  
        col4_layout.addItem(spacer)
        self.rpmval = QLineEdit(self)
        self.rpmval.setFont(QFont('Red Rose',25))
        self.rpmval.setReadOnly(True)
        col4_layout.addWidget(self.rpmval,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 70)  
        # col4_layout.addItem(spacer)
        self.kphval = QLineEdit(self)
        self.kphval.setFont(QFont('Red Rose',25))
        self.kphval.setReadOnly(True)
        col4_layout.addWidget(self.kphval,Qt.AlignCenter)
        spacer = QSpacerItem(10, 100)  
        col4_layout.addItem(spacer)
        self.vdval = QLineEdit(self)
        self.vdval.setFont(QFont('Red Rose',25))
        self.vdval.setReadOnly(True)
        col4_layout.addWidget(self.vdval,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 1)  
        # col4_layout.addItem(spacer)
        self.vqval = QLineEdit(self)
        self.vqval.setFont(QFont('Red Rose',25))
        self.vqval.setReadOnly(True)
        col4_layout.addWidget(self.vqval,Qt.AlignCenter)

        col5_layout = QVBoxLayout()
        mcdata_layout.addLayout(col5_layout)        
        # spacer = QSpacerItem(10, 30) 
        # col5_layout.addItem(spacer)
        self.mc_temps = QLabel()
        self.mc_temps.setText("Temperatures")
        self.mc_temps.setStyleSheet("color: white; font-weight: bold;font-family: 'Red Rose';font-size: 30px;")
        col5_layout.addWidget(self.mc_temps,Qt.AlignCenter)
        self.motor_temp = QLabel()
        self.motor_temp.setText("Motor Temp")
        self.motor_temp.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col5_layout.addWidget(self.motor_temp,Qt.AlignCenter)
        self.hs_temp = QLabel()
        self.hs_temp.setText("Heatsink Temp")
        self.hs_temp.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col5_layout.addWidget(self.hs_temp,Qt.AlignCenter)
        self.dsp_temp = QLabel()
        self.dsp_temp.setText("DSP Temp")
        self.dsp_temp.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col5_layout.addWidget(self.dsp_temp,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 15) 
        # col5_layout.addItem(spacer)
        self.motor_volt = QLabel()
        self.motor_volt.setText("Motor Voltage")
        self.motor_volt.setStyleSheet("color: white; font-weight: bold;font-family: 'Red Rose';font-size: 30px;")
        col5_layout.addWidget(self.motor_volt,Qt.AlignCenter)
        self.vd1 = QLabel()
        self.vd1.setText("Vd")
        self.vd1.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col5_layout.addWidget(self.vd1,Qt.AlignCenter)
        self.vq1 = QLabel()
        self.vq1.setText("Vq")
        self.vq1.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col5_layout.addWidget(self.vq1,Qt.AlignCenter)

        col6_layout = QVBoxLayout()
        mcdata_layout.addLayout(col6_layout)
        spacer = QSpacerItem(10, 60)  
        col6_layout.addItem(spacer)
        self.motor_tempval = QLineEdit(self)
        self.motor_tempval.setFont(QFont('Red Rose',25))
        self.motor_tempval.setReadOnly(True)
        col6_layout.addWidget(self.motor_tempval,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 1)  
        # col6_layout.addItem(spacer)
        self.hs_tempval = QLineEdit(self)
        self.hs_tempval.setFont(QFont('Red Rose',25))
        self.hs_tempval.setReadOnly(True)
        col6_layout.addWidget(self.hs_tempval,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 1)  
        # col6_layout.addItem(spacer)
        self.dsp_tempval = QLineEdit(self)
        self.dsp_tempval.setFont(QFont('Red Rose',25))
        self.dsp_tempval.setReadOnly(True)
        col6_layout.addWidget(self.dsp_tempval,Qt.AlignCenter)
        spacer = QSpacerItem(10, 60)  
        col6_layout.addItem(spacer)
        self.vd1val = QLineEdit(self)
        self.vd1val.setFont(QFont('Red Rose',25))
        self.vd1val.setReadOnly(True)
        col6_layout.addWidget(self.vd1val,Qt.AlignCenter)
        self.vq1val = QLineEdit(self)
        self.vq1val.setFont(QFont('Red Rose',25))
        self.vq1val.setReadOnly(True)
        col6_layout.addWidget(self.vq1val,Qt.AlignCenter)
        
        col7_layout = QVBoxLayout()
        mcdata_layout.addLayout(col7_layout)        
        # spacer = QSpacerItem(10, 30) 
        # col7_layout.addItem(spacer)
        self.other_volt = QLabel()
        self.other_volt.setText("Other Voltages")
        self.other_volt.setStyleSheet("color: white; font-weight: bold;font-family: 'Red Rose';font-size: 30px;")
        col7_layout.addWidget(self.other_volt,Qt.AlignCenter)
        self.rail1 = QLabel()
        self.rail1.setText("15V Rail")
        self.rail1.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col7_layout.addWidget(self.rail1,Qt.AlignCenter)
        self.rail2 = QLabel()
        self.rail2.setText("1.9V Rail")
        self.rail2.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col7_layout.addWidget(self.rail2,Qt.AlignCenter)
        self.rail3 = QLabel()
        self.rail3.setText("3.3V Rail")
        self.rail3.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col7_layout.addWidget(self.rail3,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 30) 
        # col7_layout.addItem(spacer)
        self.motor_curr = QLabel()
        self.motor_curr.setText("Motor Current")
        self.motor_curr.setStyleSheet("color: white; font-weight: bold;font-family: 'Red Rose';font-size: 30px;")
        col7_layout.addWidget(self.motor_curr,Qt.AlignCenter)
        self.id = QLabel()
        self.id.setText("Id")
        self.id.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col7_layout.addWidget(self.id,Qt.AlignCenter)
        self.iq = QLabel()
        self.iq.setText("Iq")
        self.iq.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        col7_layout.addWidget(self.iq,Qt.AlignCenter)

        col8_layout = QVBoxLayout()
        mcdata_layout.addLayout(col8_layout)
        spacer = QSpacerItem(10, 60)  
        col8_layout.addItem(spacer)
        self.rail1val = QLineEdit(self)
        self.rail1val.setFont(QFont('Red Rose',25))
        self.rail1val.setReadOnly(True)
        col8_layout.addWidget(self.rail1val,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 1)  
        # col8_layout.addItem(spacer)
        self.rail2val = QLineEdit(self)
        self.rail2val.setFont(QFont('Red Rose',25))
        self.rail2val.setReadOnly(True)
        col8_layout.addWidget(self.rail2val,Qt.AlignCenter)
        # spacer = QSpacerItem(10, 1)  
        # col8_layout.addItem(spacer)
        self.rail3val = QLineEdit(self)
        self.rail3val.setFont(QFont('Red Rose',25))
        self.rail3val.setReadOnly(True)
        col8_layout.addWidget(self.rail3val,Qt.AlignCenter)
        spacer = QSpacerItem(10, 60)  
        col8_layout.addItem(spacer)
        self.idval = QLineEdit(self)
        self.idval.setFont(QFont('Red Rose',25))
        self.idval.setReadOnly(True)
        col8_layout.addWidget(self.idval,Qt.AlignCenter)
        self.iqval = QLineEdit(self)
        self.iqval.setFont(QFont('Red Rose',25))
        self.iqval.setReadOnly(True)
        col8_layout.addWidget(self.iqval,Qt.AlignCenter)

        #BATTERY Layout
        
        self.nocan_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/nocan.png']

        self.vehiclecomms_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/vehicomms.png']

        self.cmucomms_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/cmucomms.png']

        self.prechargeerror_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/prechargerror.png']

        self.prechargeidle_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/prechargeidle.png']

        self.onandr_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/onandr.png']

        self.cellov_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/cellvoltage.png']

        self.celluv_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/cellunderv.png']

        self.cellot_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/cellovert.png']

        self.trusterror_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/trusterror.png']

        self.packisofail_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/packisofail.png']

        self.contactorstuck_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/contactorstuck.png']

        self.contactorclosed_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/contactorclosed.png']

        self.can12vlow_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/can12vlow.png']

        self.cmupwrstat_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/cmupwrstatus.png']

        self.extracell_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/extracell.png']

        self.socinvalid_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/socinvalid.png']

        self.bmusetup_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/bmu.png']

        self.grid1 = QGridLayout()
        self.battery_layout.addLayout(self.grid1) 

        self.nocan = QLabel()
        self.grid1.addWidget(self.nocan, 0,0, Qt.AlignCenter)

        self.vehiclecomms = QLabel()
        self.grid1.addWidget(self.vehiclecomms, 0,1,Qt.AlignCenter)

        self.cmucomms = QLabel()
        self.grid1.addWidget(self.cmucomms, 0,2,Qt.AlignCenter)

        self.prechargeerror = QLabel()
        self.grid1.addWidget(self.prechargeerror, 0,3,Qt.AlignCenter)

        self.prechargeidle = QLabel()
        self.grid1.addWidget(self.prechargeidle, 1,0,Qt.AlignCenter)

        self.onandr = QLabel()
        self.grid1.addWidget(self.onandr, 1,1,Qt.AlignCenter)

        self.cellov = QLabel()
        self.grid1.addWidget(self.cellov, 1,2,Qt.AlignCenter)

        self.celluv = QLabel()
        self.grid1.addWidget(self.celluv, 1,3,Qt.AlignCenter)

        self.cellot = QLabel()
        self.grid1.addWidget(self.cellot, 2,0,Qt.AlignCenter)

        self.trusterror = QLabel()
        self.grid1.addWidget(self.trusterror, 2,1,Qt.AlignCenter)

        self.packisofail = QLabel()
        self.grid1.addWidget(self.packisofail, 2,2,Qt.AlignCenter)

        self.contactorstuck = QLabel()
        self.grid1.addWidget(self.contactorstuck, 2,3,Qt.AlignCenter)

        self.contactorclosed = QLabel()
        self.grid1.addWidget(self.contactorclosed, 3,0,Qt.AlignCenter)

        self.can12vlow = QLabel()
        self.grid1.addWidget(self.can12vlow, 3,1,Qt.AlignCenter)

        self.cmupwrstat = QLabel()
        self.grid1.addWidget(self.cmupwrstat, 3,2,Qt.AlignCenter)

        self.extracell = QLabel()
        self.grid1.addWidget(self.extracell, 3,3,Qt.AlignCenter)

        self.socinvalid = QLabel()
        self.grid1.addWidget(self.socinvalid, 4,0,Qt.AlignCenter)

        self.bmusetup = QLabel()
        self.grid1.addWidget(self.bmusetup, 4,1,Qt.AlignCenter)

        self.bmu_label = QLabel()
        self.bmu_label.setText("BMU Telemetry")
        self.bmu_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 30px;")
        self.battery_layout.addWidget(self.bmu_label,Qt.AlignCenter)

        self.grid2 = QGridLayout()
        self.battery_layout.addLayout(self.grid2)

        self.minmv_label = QLabel()
        self.minmv_label.setText("Min mV")
        self.minmv_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.minmv_label, 0,1, Qt.AlignCenter)

        self.maxmv_label = QLabel()
        self.maxmv_label.setText("Max mV")
        self.maxmv_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.maxmv_label, 0,2, Qt.AlignCenter)

        self.minc_label = QLabel()
        self.minc_label.setText("Min °C")
        self.minc_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.minc_label, 0,3, Qt.AlignCenter)

        self.maxc_label = QLabel()
        self.maxc_label.setText("Max °C")
        self.maxc_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.maxc_label, 0,4, Qt.AlignCenter)

        self.packmv_label = QLabel()
        self.packmv_label.setText("Pack mV")
        self.packmv_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.packmv_label, 0,5, Qt.AlignCenter)

        self.packma_label = QLabel()
        self.packma_label.setText("Pack mA")
        self.packma_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.packma_label, 0,6, Qt.AlignCenter)

        self.balp_label = QLabel()
        self.balp_label.setText("Balance +")
        self.balp_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.balp_label, 0,7, Qt.AlignCenter)

        self.balm_label = QLabel()
        self.balm_label.setText("Balance -")
        self.balm_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.balm_label, 0,8, Qt.AlignCenter)

        self.cmucount_label = QLabel()
        self.cmucount_label.setText("CMU Count")
        self.cmucount_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.cmucount_label, 0,9, Qt.AlignCenter)

        self.sysstat_label = QLabel()
        self.sysstat_label.setText("Sys Status")
        self.sysstat_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.sysstat_label, 1,0, Qt.AlignCenter)

        self.minmv_val = QLineEdit(self)
        self.minmv_val.setFont(QFont('Red Rose',25))
        self.minmv_val.setReadOnly(True)
        self.grid2.addWidget(self.minmv_val,1,1,Qt.AlignCenter)

        self.maxmv_val = QLineEdit(self)
        self.maxmv_val.setFont(QFont('Red Rose',25))
        self.maxmv_val.setReadOnly(True)
        self.grid2.addWidget(self.maxmv_val,1,2,Qt.AlignCenter)

        self.minc_val = QLineEdit(self)
        self.minc_val.setFont(QFont('Red Rose',25))
        self.minc_val.setReadOnly(True)
        self.grid2.addWidget(self.minc_val,1,3,Qt.AlignCenter)

        self.maxc_val = QLineEdit(self)
        self.maxc_val.setFont(QFont('Red Rose',25))
        self.maxc_val.setReadOnly(True)
        self.grid2.addWidget(self.maxc_val,1,4,Qt.AlignCenter)

        self.packmv_val = QLineEdit(self)
        self.packmv_val.setFont(QFont('Red Rose',25))
        self.packmv_val.setReadOnly(True)
        self.grid2.addWidget(self.packmv_val,1,5,Qt.AlignCenter)

        self.packma_val = QLineEdit(self)
        self.packma_val.setFont(QFont('Red Rose',25))
        self.packma_val.setReadOnly(True)
        self.grid2.addWidget(self.packma_val,1,6,Qt.AlignCenter)

        self.balp_val = QLineEdit(self)
        self.balp_val.setFont(QFont('Red Rose',25))
        self.balp_val.setReadOnly(True)
        self.grid2.addWidget(self.balp_val,1,7,Qt.AlignCenter)

        self.balm_val = QLineEdit(self)
        self.balm_val.setFont(QFont('Red Rose',25))
        self.balm_val.setReadOnly(True)
        self.grid2.addWidget(self.balm_val,1,8,Qt.AlignCenter)

        self.cmucount_val = QLineEdit(self)
        self.cmucount_val.setFont(QFont('Red Rose',25))
        self.cmucount_val.setReadOnly(True)
        self.grid2.addWidget(self.cmucount_val,1,9,Qt.AlignCenter)

        self.fan_label = QLabel()
        self.fan_label.setText("Fan Speed\n     (rpm)")
        self.fan_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.fan_label, 2,7, Qt.AlignCenter)

        self.soc_label = QLabel()
        self.soc_label.setText("SOC/BAL\n      (Ah)")
        self.soc_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.soc_label, 2,8, Qt.AlignCenter)

        self.socp_label = QLabel()
        self.socp_label.setText("SOC/BAL %")
        self.socp_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.socp_label, 2,9, Qt.AlignCenter)     
        
        self.prestat_label = QLabel()
        self.prestat_label.setText("Precharge\n    Status")
        self.prestat_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid2.addWidget(self.prestat_label, 3,0, Qt.AlignCenter)  

        self.prestat_val = QLineEdit(self)
        self.prestat_val.setFont(QFont('Red Rose',25))
        self.prestat_val.setReadOnly(True)
        self.grid2.addWidget(self.prestat_val,3,1,Qt.AlignCenter) 

        self.fan1_val = QLineEdit(self)
        self.fan1_val.setFont(QFont('Red Rose',25))
        self.fan1_val.setReadOnly(True)
        self.grid2.addWidget(self.fan1_val,3,7,Qt.AlignCenter) 

        self.soc1_val = QLineEdit(self)
        self.soc1_val.setFont(QFont('Red Rose',25))
        self.soc1_val.setReadOnly(True)
        self.grid2.addWidget(self.soc1_val,3,8,Qt.AlignCenter) 

        self.socp1_val = QLineEdit(self)
        self.socp1_val.setFont(QFont('Red Rose',25))
        self.socp1_val.setReadOnly(True)
        self.grid2.addWidget(self.socp1_val,3,9,Qt.AlignCenter) 

        self.fan2_val = QLineEdit(self)
        self.fan2_val.setFont(QFont('Red Rose',25))
        self.fan2_val.setReadOnly(True)
        self.grid2.addWidget(self.fan2_val,4,7,Qt.AlignCenter) 

        self.soc2_val = QLineEdit(self)
        self.soc2_val.setFont(QFont('Red Rose',25))
        self.soc2_val.setReadOnly(True)
        self.grid2.addWidget(self.soc2_val,4,8,Qt.AlignCenter) 

        self.socp2_val = QLineEdit(self)
        self.socp2_val.setFont(QFont('Red Rose',25))
        self.socp2_val.setReadOnly(True)
        self.grid2.addWidget(self.socp2_val,4,9,Qt.AlignCenter) 

        self.cmu_label = QLabel()
        self.cmu_label.setText("CMU Telemetry")
        self.cmu_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 30px;")
        self.battery_layout.addWidget(self.cmu_label)

        self.grid3 = QGridLayout()
        self.battery_layout.addLayout(self.grid3)

        self.ser_label = QLabel()
        self.ser_label.setText("Serial")
        self.ser_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.ser_label, 0,1, Qt.AlignCenter)

        self.pcbc_label = QLabel()
        self.pcbc_label.setText("PCB °C")
        self.pcbc_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.pcbc_label, 0,2, Qt.AlignCenter)

        self.cellc_label = QLabel()
        self.cellc_label.setText("Cell °C")
        self.cellc_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cellc_label, 0,3, Qt.AlignCenter)

        self.cell0_label = QLabel()
        self.cell0_label.setText("Cell 0\n mV")
        self.cell0_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell0_label, 0,4, Qt.AlignCenter)

        self.cell1_label = QLabel()
        self.cell1_label.setText("Cell 1\n mV")
        self.cell1_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell1_label, 0,5, Qt.AlignCenter)

        self.cell2_label = QLabel()
        self.cell2_label.setText("Cell 2\n mV")
        self.cell2_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell2_label, 0,6, Qt.AlignCenter)

        self.cell3_label = QLabel()
        self.cell3_label.setText("Cell 3\n mV")
        self.cell3_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell3_label, 0,7, Qt.AlignCenter)

        self.cell4_label = QLabel()
        self.cell4_label.setText("Cell 4\n mV")
        self.cell4_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell4_label, 0,8, Qt.AlignCenter)

        self.cell5_label = QLabel()
        self.cell5_label.setText("Cell 5\n mV")
        self.cell5_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell5_label, 0,9, Qt.AlignCenter)

        self.cell6_label = QLabel()
        self.cell6_label.setText("Cell 6\n mV")
        self.cell6_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell6_label, 0,10, Qt.AlignCenter)

        self.cell7_label = QLabel()
        self.cell7_label.setText("Cell 7\n mV")
        self.cell7_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell7_label, 0,11, Qt.AlignCenter)

        self.cmu1_label = QLabel()
        self.cmu1_label.setText("CMU 1")
        self.cmu1_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cmu1_label, 1,0, Qt.AlignCenter)

        self.ser1_val = QLabel()
        self.ser1_val.setText("0")
        self.ser1_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.ser1_val, 1,1, Qt.AlignCenter)

        self.pcb1_val = QLabel()
        self.pcb1_val.setText("0")
        self.pcb1_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.pcb1_val, 1,2, Qt.AlignCenter)

        self.cell_val = QLabel()
        self.cell_val.setText("0")
        self.cell_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell_val, 1,3, Qt.AlignCenter)

        self.cell01_val = QLabel()
        self.cell01_val.setText("0")
        self.cell01_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell01_val, 1,4, Qt.AlignCenter)

        self.cell11_val = QLabel()
        self.cell11_val.setText("0")
        self.cell11_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell11_val, 1,5, Qt.AlignCenter)

        self.cell21_val = QLabel()
        self.cell21_val.setText("0")
        self.cell21_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell21_val, 1,6, Qt.AlignCenter)

        self.cell31_val = QLabel()
        self.cell31_val.setText("0")
        self.cell31_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell31_val, 1,7, Qt.AlignCenter)

        self.cell41_val = QLabel()
        self.cell41_val.setText("0")
        self.cell41_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell41_val, 1,8, Qt.AlignCenter)

        self.cell51_val = QLabel()
        self.cell51_val.setText("0")
        self.cell51_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell51_val, 1,9, Qt.AlignCenter)

        self.cell61_val = QLabel()
        self.cell61_val.setText("0")
        self.cell61_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell61_val, 1,10, Qt.AlignCenter)

        self.cell71_val = QLabel()
        self.cell71_val.setText("0")
        self.cell71_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell71_val, 1,11, Qt.AlignCenter)

        self.cmu2_label = QLabel()
        self.cmu2_label.setText("CMU 2")
        self.cmu2_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cmu2_label, 2,0, Qt.AlignCenter)

        self.ser2_val = QLabel()
        self.ser2_val.setText("0")
        self.ser2_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.ser2_val, 2,1, Qt.AlignCenter)

        self.pcb2_val = QLabel()
        self.pcb2_val.setText("0")
        self.pcb2_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.pcb2_val, 2,2, Qt.AlignCenter)

        self.cell1_val = QLabel()
        self.cell1_val.setText("0")
        self.cell1_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell1_val, 2,3, Qt.AlignCenter)

        self.cell02_val = QLabel()
        self.cell02_val.setText("0")
        self.cell02_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell02_val, 2,4, Qt.AlignCenter)

        self.cell12_val = QLabel()
        self.cell12_val.setText("0")
        self.cell12_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell12_val, 2,5, Qt.AlignCenter)

        self.cell22_val = QLabel()
        self.cell22_val.setText("0")
        self.cell22_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell22_val, 2,6, Qt.AlignCenter)

        self.cell32_val = QLabel()
        self.cell32_val.setText("0")
        self.cell32_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell32_val, 2,7, Qt.AlignCenter)

        self.cell42_val = QLabel()
        self.cell42_val.setText("0")
        self.cell42_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell42_val, 2,8, Qt.AlignCenter)

        self.cell52_val = QLabel()
        self.cell52_val.setText("0")
        self.cell52_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell52_val, 2,9, Qt.AlignCenter)

        self.cell62_val = QLabel()
        self.cell62_val.setText("0")
        self.cell62_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell62_val, 2,10, Qt.AlignCenter)

        self.cell72_val = QLabel()
        self.cell72_val.setText("0")
        self.cell72_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell72_val, 2,11, Qt.AlignCenter)

        self.cmu3_label = QLabel()
        self.cmu3_label.setText("CMU 3")
        self.cmu3_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cmu3_label, 3,0, Qt.AlignCenter)

        self.ser3_val = QLabel()
        self.ser3_val.setText("0")
        self.ser3_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.ser3_val, 3,1, Qt.AlignCenter)

        self.pcb3_val = QLabel()
        self.pcb3_val.setText("0")
        self.pcb3_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.pcb3_val, 3,2, Qt.AlignCenter)

        self.cell2_val = QLabel()
        self.cell2_val.setText("0")
        self.cell2_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell2_val, 3,3, Qt.AlignCenter)

        self.cell03_val = QLabel()
        self.cell03_val.setText("0")
        self.cell03_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell03_val, 3,4, Qt.AlignCenter)

        self.cell13_val = QLabel()
        self.cell13_val.setText("0")
        self.cell13_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell13_val, 3,5, Qt.AlignCenter)

        self.cell23_val = QLabel()
        self.cell23_val.setText("0")
        self.cell23_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell23_val, 3,6, Qt.AlignCenter)

        self.cell33_val = QLabel()
        self.cell33_val.setText("0")
        self.cell33_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell33_val, 3,7, Qt.AlignCenter)

        self.cell43_val = QLabel()
        self.cell43_val.setText("0")
        self.cell43_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell43_val, 3,8, Qt.AlignCenter)

        self.cell53_val = QLabel()
        self.cell53_val.setText("0")
        self.cell53_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell53_val, 3,9, Qt.AlignCenter)

        self.cell63_val = QLabel()
        self.cell63_val.setText("0")
        self.cell63_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell63_val, 3,10, Qt.AlignCenter)

        self.cell73_val = QLabel()
        self.cell73_val.setText("0")
        self.cell73_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell73_val, 3,11, Qt.AlignCenter)

        self.cmu4_label = QLabel()
        self.cmu4_label.setText("CMU 4")
        self.cmu4_label.setStyleSheet("color: white;font-weight: bold;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cmu4_label, 4,0, Qt.AlignCenter)

        self.ser4_val = QLabel()
        self.ser4_val.setText("0")
        self.ser4_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.ser4_val, 4,1, Qt.AlignCenter)

        self.pcb4_val = QLabel()
        self.pcb4_val.setText("0")
        self.pcb4_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.pcb4_val, 4,2, Qt.AlignCenter)

        self.cell3_val = QLabel()
        self.cell3_val.setText("0")
        self.cell3_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell3_val, 4,3, Qt.AlignCenter)

        self.cell04_val = QLabel()
        self.cell04_val.setText("0")
        self.cell04_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell04_val, 4,4, Qt.AlignCenter)

        self.cell14_val = QLabel()
        self.cell14_val.setText("0")
        self.cell14_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell14_val, 4,5, Qt.AlignCenter)

        self.cell24_val = QLabel()
        self.cell24_val.setText("0")
        self.cell24_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell24_val, 4,6, Qt.AlignCenter)

        self.cell34_val = QLabel()
        self.cell34_val.setText("0")
        self.cell34_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell34_val, 4,7, Qt.AlignCenter)

        self.cell44_val = QLabel()
        self.cell44_val.setText("0")
        self.cell44_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell44_val, 4,8, Qt.AlignCenter)

        self.cell54_val = QLabel()
        self.cell54_val.setText("0")
        self.cell54_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell54_val, 4,9, Qt.AlignCenter)

        self.cell64_val = QLabel()
        self.cell64_val.setText("0")
        self.cell64_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell64_val, 4,10, Qt.AlignCenter)

        self.cell74_val = QLabel()
        self.cell74_val.setText("0")
        self.cell74_val.setStyleSheet("color: white;font-family: 'Red Rose';font-size: 25px;")
        self.grid3.addWidget(self.cell74_val, 4,11, Qt.AlignCenter)

        

        #VOLT/AMPS
        self.curr_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/currgr_fr.png',
                            '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/curr_fr.png',
                            '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/currred_fr.png']

        self.solar_label = QLabel(self)
        self.solar_label.setAlignment(Qt.AlignCenter)
        self.solar_label.setStyleSheet("font-family: 'Good Times'; font-size: 50px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; ")
        self.solar_label.setText("SOLAR")
        self.voltamps_layout.addWidget(self.solar_label)

        self.solar_grid = QGridLayout()
        self.voltamps_layout.addLayout(self.solar_grid)

        self.solar_voltage_fr = QLabel(self)
        self.solar_grid.addWidget(self.solar_voltage_fr, 0,0, Qt.AlignCenter)

        self.solar_voltage_fr2 = QLabel(self)
        self.solar_grid.addWidget(self.solar_voltage_fr2, 0,1, Qt.AlignCenter)

        self.solar_voltage_fr3 = QLabel(self)
        self.solar_grid.addWidget(self.solar_voltage_fr3, 0,2, Qt.AlignCenter)

        self.solar_voltage_fr4 = QLabel(self)
        self.solar_grid.addWidget(self.solar_voltage_fr4, 0,3, Qt.AlignCenter)

        self.solar_voltage_fr5 = QLabel(self)
        self.solar_grid.addWidget(self.solar_voltage_fr5, 0,4, Qt.AlignCenter)

        self.solar_current_fr = QLabel(self)
        self.solar_grid.addWidget(self.solar_current_fr, 1,0, Qt.AlignCenter)
        
        self.solar_current_fr2 = QLabel(self)
        self.solar_grid.addWidget(self.solar_current_fr2, 1,1, Qt.AlignCenter)

        self.solar_current_fr3 = QLabel(self)
        self.solar_grid.addWidget(self.solar_current_fr3, 1,2, Qt.AlignCenter)

        self.solar_current_fr4 = QLabel(self)
        self.solar_grid.addWidget(self.solar_current_fr4, 1,3, Qt.AlignCenter)

        self.solar_current_fr5 = QLabel(self)
        self.solar_grid.addWidget(self.solar_current_fr5, 1,4, Qt.AlignCenter)

        self.solar_power_fr = QLabel(self)
        self.solar_grid.addWidget(self.solar_power_fr, 2,0, Qt.AlignCenter)
        
        self.solar_power_fr2 = QLabel(self)
        self.solar_grid.addWidget(self.solar_power_fr2, 2,1, Qt.AlignCenter)
        
        self.solar_power_fr3 = QLabel(self)
        self.solar_grid.addWidget(self.solar_power_fr3, 2,2, Qt.AlignCenter)

        self.solar_power_fr4 = QLabel(self)
        self.solar_grid.addWidget(self.solar_power_fr4, 2,3, Qt.AlignCenter)

        self.solar_power_fr5 = QLabel(self)
        self.solar_grid.addWidget(self.solar_power_fr5, 2,4, Qt.AlignCenter)

        self.battery_label = QLabel(self)
        self.battery_label.setAlignment(Qt.AlignCenter)
        self.battery_label.setStyleSheet("font-family: 'Good Times'; font-size: 50px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        )
        self.battery_label.setText("BATTERY")
        self.voltamps_layout.addWidget(self.battery_label)

        self.battery_grid = QGridLayout()
        self.voltamps_layout.addLayout(self.battery_grid)

        self.battery_soc_fr = QLabel(self)
        self.battery_grid.addWidget(self.battery_soc_fr, 0,0, Qt.AlignCenter)

        self.battery_voltage_fr = QLabel(self)
        self.battery_grid.addWidget(self.battery_voltage_fr, 0,1, Qt.AlignCenter)

        self.battery_current_fr = QLabel(self)
        self.battery_grid.addWidget(self.battery_current_fr, 0,2, Qt.AlignCenter)

        self.battery_power_fr = QLabel(self)
        self.battery_grid.addWidget(self.battery_power_fr, 0,3, Qt.AlignCenter)

        self.motorctrl_label = QLabel(self)
        self.motorctrl_label.setAlignment(Qt.AlignCenter)
        self.motorctrl_label.setStyleSheet("font-family: 'Good Times'; font-size: 50px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        )
        self.motorctrl_label.setText("MOTOR CONTROLLER")
        self.voltamps_layout.addWidget(self.motorctrl_label)

        self.motorctrl_grid = QGridLayout()
        self.voltamps_layout.addLayout(self.motorctrl_grid)

        self.motorctrl_voltage_fr = QLabel(self)
        self.motorctrl_grid.addWidget(self.motorctrl_voltage_fr, 0,0, Qt.AlignCenter)

        self.motorctrl_current_fr = QLabel(self)
        self.motorctrl_grid.addWidget(self.motorctrl_current_fr, 0,1, Qt.AlignCenter)

        self.motorctrl_power_fr = QLabel(self)
        self.motorctrl_grid.addWidget(self.motorctrl_power_fr, 0,2, Qt.AlignCenter)

        #TEMPS
        self.temp_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Tempcool.png',
                            '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/tempok.png',
                            '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/temphot.png',
                            ]

        self.solar1 = QLabel(self)
        self.solar1.setAlignment(Qt.AlignCenter)
        self.solar1.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar1.setText("SOLAR-1")
        self.temps_layout.addWidget(self.solar1, 0,0, Qt.AlignCenter)

        self.solar2 = QLabel(self)
        self.solar2.setAlignment(Qt.AlignCenter)
        self.solar2.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar2.setText("SOLAR-2")
        self.temps_layout.addWidget(self.solar2, 0,1, Qt.AlignCenter)

        self.solar3 = QLabel(self)
        self.solar3.setAlignment(Qt.AlignCenter)
        self.solar3.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar3.setText("SOLAR-3")
        self.temps_layout.addWidget(self.solar3, 0,2, Qt.AlignCenter)

        self.solar4 = QLabel(self)
        self.solar4.setAlignment(Qt.AlignCenter)
        self.solar4.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar4.setText("SOLAR-4")
        self.temps_layout.addWidget(self.solar4, 0,3, Qt.AlignCenter)

        self.solar5 = QLabel(self)
        self.solar5.setAlignment(Qt.AlignCenter)
        self.solar5.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar5.setText("SOLAR-5")
        self.temps_layout.addWidget(self.solar5, 0,4, Qt.AlignCenter)

        self.solar6 = QLabel(self)
        self.solar6.setAlignment(Qt.AlignCenter)
        self.solar6.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar6.setText("SOLAR-6")
        self.temps_layout.addWidget(self.solar6, 0,5, Qt.AlignCenter)

        self.temp1_label = QLabel(self)
        self.temps_layout.addWidget(self.temp1_label, 1,0, Qt.AlignCenter)

        self.temp2_label = QLabel(self)
        self.temps_layout.addWidget(self.temp2_label, 1,1, Qt.AlignCenter)

        self.temp3_label = QLabel(self)
        self.temps_layout.addWidget(self.temp3_label, 1,2, Qt.AlignCenter)
               
        self.temp4_label = QLabel(self)
        self.temps_layout.addWidget(self.temp4_label, 1,3, Qt.AlignCenter)
        
        self.temp5_label = QLabel(self)
        self.temps_layout.addWidget(self.temp5_label, 1,4, Qt.AlignCenter)
        
        self.temp6_label = QLabel(self)
        self.temps_layout.addWidget(self.temp6_label, 1,5, Qt.AlignCenter)


        self.solar7 = QLabel(self)
        self.solar7.setAlignment(Qt.AlignCenter)
        self.solar7.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar7.setText("SOLAR-7")
        self.temps_layout.addWidget(self.solar7, 2,0, Qt.AlignCenter)

        self.solar8 = QLabel(self)
        self.solar8.setAlignment(Qt.AlignCenter)
        self.solar8.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar8.setText("SOLAR-8")
        self.temps_layout.addWidget(self.solar8, 2,1, Qt.AlignCenter)

        self.solar9 = QLabel(self)
        self.solar9.setAlignment(Qt.AlignCenter)
        self.solar9.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar9.setText("SOLAR-9")
        self.temps_layout.addWidget(self.solar9, 2,2, Qt.AlignCenter)

        self.solar10 = QLabel(self)
        self.solar10.setAlignment(Qt.AlignCenter)
        self.solar10.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar10.setText("SOLAR-10")
        self.temps_layout.addWidget(self.solar10, 2,3, Qt.AlignCenter)

        self.solar11 = QLabel(self)
        self.solar11.setAlignment(Qt.AlignCenter)
        self.solar11.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar11.setText("SOLAR-11")
        self.temps_layout.addWidget(self.solar11, 2,4, Qt.AlignCenter)

        self.solar12 = QLabel(self)
        self.solar12.setAlignment(Qt.AlignCenter)
        self.solar12.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.solar12.setText("SOLAR-12")
        self.temps_layout.addWidget(self.solar12, 2,5, Qt.AlignCenter)
        
        self.temp7_label = QLabel(self)
        self.temps_layout.addWidget(self.temp7_label, 3,0, Qt.AlignCenter)

        self.temp8_label = QLabel(self)
        self.temps_layout.addWidget(self.temp8_label, 3,1, Qt.AlignCenter)

        self.temp9_label = QLabel(self)
        self.temps_layout.addWidget(self.temp9_label, 3,2, Qt.AlignCenter)
               
        self.temp10_label = QLabel(self)
        self.temps_layout.addWidget(self.temp10_label, 3,3, Qt.AlignCenter)
        
        self.temp11_label = QLabel(self)
        self.temps_layout.addWidget(self.temp11_label, 3,4, Qt.AlignCenter)
        
        self.temp12_label = QLabel(self)
        self.temps_layout.addWidget(self.temp12_label, 3,5, Qt.AlignCenter)

        self.mppt1 = QLabel(self)
        self.mppt1.setAlignment(Qt.AlignCenter)
        self.mppt1.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.mppt1.setText("MPPT-1")
        self.temps_layout.addWidget(self.mppt1, 4,0, Qt.AlignCenter)

        self.mppt2 = QLabel(self)
        self.mppt2.setAlignment(Qt.AlignCenter)
        self.mppt2.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.mppt2.setText("MPPT-2")
        self.temps_layout.addWidget(self.mppt2, 4,1, Qt.AlignCenter)

        self.mppt3 = QLabel(self)
        self.mppt3.setAlignment(Qt.AlignCenter)
        self.mppt3.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.mppt3.setText("MPPT-3")
        self.temps_layout.addWidget(self.mppt3, 4,2, Qt.AlignCenter)

        self.mppt4 = QLabel(self)
        self.mppt4.setAlignment(Qt.AlignCenter)
        self.mppt4.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.mppt4.setText("MPPT-4")
        self.temps_layout.addWidget(self.mppt4, 4,3, Qt.AlignCenter)

        self.motorctrl_temp = QLabel(self)
        self.motorctrl_temp.setAlignment(Qt.AlignCenter)
        self.motorctrl_temp.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.motorctrl_temp.setText("MOTOR CNTRL")
        self.temps_layout.addWidget(self.motorctrl_temp, 4,4, Qt.AlignCenter)

        self.motor_temp = QLabel(self)
        self.motor_temp.setAlignment(Qt.AlignCenter)
        self.motor_temp.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.motor_temp.setText("MOTOR")
        self.temps_layout.addWidget(self.motor_temp, 4,5, Qt.AlignCenter)

        self.temp13_label = QLabel(self)
        self.temps_layout.addWidget(self.temp13_label, 5,0, Qt.AlignCenter)

        self.temp14_label = QLabel(self)
        self.temps_layout.addWidget(self.temp14_label, 5,1, Qt.AlignCenter)

        self.temp15_label = QLabel(self)
        self.temps_layout.addWidget(self.temp15_label, 5,2, Qt.AlignCenter)
               
        self.temp16_label = QLabel(self)
        self.temps_layout.addWidget(self.temp16_label, 5,3, Qt.AlignCenter)
        
        self.temp17_label = QLabel(self)
        self.temps_layout.addWidget(self.temp17_label, 5,4, Qt.AlignCenter)
        
        self.temp18_label = QLabel(self)
        self.temps_layout.addWidget(self.temp18_label, 5,5, Qt.AlignCenter)

        self.bat1 = QLabel(self)
        self.bat1.setAlignment(Qt.AlignCenter)
        self.bat1.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.bat1.setText("BATTERY-1")
        self.temps_layout.addWidget(self.bat1, 6,0, Qt.AlignCenter)

        self.bat2 = QLabel(self)
        self.bat2.setAlignment(Qt.AlignCenter)
        self.bat2.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.bat2.setText("BATTERY-2")
        self.temps_layout.addWidget(self.bat2, 6,1, Qt.AlignCenter)

        self.bat3 = QLabel(self)
        self.bat3.setAlignment(Qt.AlignCenter)
        self.bat3.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.bat3.setText("BATTERY-3")
        self.temps_layout.addWidget(self.bat3, 6,2, Qt.AlignCenter)

        self.bat4 = QLabel(self)
        self.bat4.setAlignment(Qt.AlignCenter)
        self.bat4.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.bat4.setText("BATTERY-4")
        self.temps_layout.addWidget(self.bat4, 6,3, Qt.AlignCenter)

        self.cabin_temp = QLabel(self)
        self.cabin_temp.setAlignment(Qt.AlignCenter)
        self.cabin_temp.setStyleSheet("font-family: 'Good Times'; font-size: 30px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                )
        self.cabin_temp.setText("CABIN")
        self.temps_layout.addWidget(self.cabin_temp, 6,4, Qt.AlignCenter)

        self.temp19_label = QLabel(self)
        self.temps_layout.addWidget(self.temp19_label, 7,0, Qt.AlignCenter)

        self.temp20_label = QLabel(self)
        self.temps_layout.addWidget(self.temp20_label, 7,1, Qt.AlignCenter)

        self.temp21_label = QLabel(self)
        self.temps_layout.addWidget(self.temp21_label, 7,2, Qt.AlignCenter)
               
        self.temp22_label = QLabel(self)
        self.temps_layout.addWidget(self.temp22_label, 7,3, Qt.AlignCenter)
        
        self.temp23_label = QLabel(self)
        self.temps_layout.addWidget(self.temp23_label, 7,4, Qt.AlignCenter)

        #TELEMETRY
        self.radio_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/greenfr.png',
                             "/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/redfr.png"]

        self.telemetry_label = QLabel(self)
        self.telemetry_label.setAlignment(Qt.AlignCenter)
        self.telemetry_label.setStyleSheet("font-family: 'Good Times'; font-size: 60px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                              )
        self.telemetry_label.setText("TELEMETRY")
        self.telemetry_layout.addWidget(self.telemetry_label)

        teledata_layout = QHBoxLayout()
        self.telemetry_layout.addLayout(teledata_layout)

        self.telecol1 = QVBoxLayout()
        teledata_layout.addLayout(self.telecol1)

        self.radio_label = QLabel(self)
        self.radio_label.setAlignment(Qt.AlignCenter)
        self.radio_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.radio_label.setText("Radio State")
        self.telecol1.addWidget(self.radio_label,Qt.AlignCenter)

        self.transmit_label = QLabel(self)
        self.transmit_label.setAlignment(Qt.AlignCenter)
        self.transmit_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.transmit_label.setText("Transmitting")
        self.telecol1.addWidget(self.transmit_label,Qt.AlignCenter)

        self.receive_label = QLabel(self)
        self.receive_label.setAlignment(Qt.AlignCenter)
        self.receive_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.receive_label.setText("Receiving")
        self.telecol1.addWidget(self.receive_label,Qt.AlignCenter)

        self.rfrssi_label = QLabel(self)
        self.rfrssi_label.setAlignment(Qt.AlignCenter)
        self.rfrssi_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.rfrssi_label.setText("RF RSSI")
        self.telecol1.addWidget(self.rfrssi_label,Qt.AlignCenter)

        self.snr_label = QLabel(self)
        self.snr_label.setAlignment(Qt.AlignCenter)
        self.snr_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.snr_label.setText("SNR")
        self.telecol1.addWidget(self.snr_label,Qt.AlignCenter)

        self.msgrec_label = QLabel(self)
        self.msgrec_label.setAlignment(Qt.AlignCenter)
        self.msgrec_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.msgrec_label.setText("Message\nRecived ")
        self.telecol1.addWidget(self.msgrec_label,Qt.AlignCenter)

        self.msgsent_label = QLabel(self)
        self.msgsent_label.setAlignment(Qt.AlignCenter)
        self.msgsent_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.msgsent_label.setText("Message\nSent")
        self.telecol1.addWidget(self.msgsent_label,Qt.AlignCenter)

        telecol2 = QVBoxLayout()
        teledata_layout.addLayout(telecol2)

        self.radiost_label = QLabel(self)
        telecol2.addWidget(self.radiost_label,Qt.AlignCenter)

        self.transt_label = QLabel(self)
        telecol2.addWidget(self.transt_label,Qt.AlignCenter)

        self.recst_label = QLabel(self)
        telecol2.addWidget(self.recst_label,Qt.AlignCenter)

        self.rssi_fr = QLabel(self)
        telecol2.addWidget(self.rssi_fr,Qt.AlignCenter)

        self.snr_fr = QLabel(self)
        telecol2.addWidget(self.snr_fr,Qt.AlignCenter)

        self.msgrec_fr = QLabel(self)
        telecol2.addWidget(self.msgrec_fr,Qt.AlignCenter)

        self.msgsent_fr = QLabel(self)
        telecol2.addWidget(self.msgsent_fr,Qt.AlignCenter)

        #CABIN
        self.cabin_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/cabingr_fr.png',
                             '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/radiofr.png',
                             "/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/cabinred_fr.png"]

        self.cabin_label = QLabel(self)
        self.cabin_label.setAlignment(Qt.AlignCenter)
        self.cabin_label.setStyleSheet("font-family: 'Good Times'; font-size: 60px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                              )
        self.cabin_label.setText("CABIN")
        self.cabin_layout.addWidget(self.cabin_label)

        cabindata_layout = QHBoxLayout()
        self.cabin_layout.addLayout(cabindata_layout)

        self.cabincol1 = QVBoxLayout()
        cabindata_layout.addLayout(self.cabincol1)

        self.temp_label = QLabel(self)
        self.temp_label.setAlignment(Qt.AlignCenter)
        self.temp_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.temp_label.setText("Temperarture")
        self.cabincol1.addWidget(self.temp_label,Qt.AlignCenter)

        self.humd_label = QLabel(self)
        self.humd_label.setAlignment(Qt.AlignCenter)
        self.humd_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.humd_label.setText("Humidity Level")
        self.cabincol1.addWidget(self.humd_label,Qt.AlignCenter)

        self.pressure_label = QLabel(self)
        self.pressure_label.setAlignment(Qt.AlignCenter)
        self.pressure_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.pressure_label.setText("Pressure")
        self.cabincol1.addWidget(self.pressure_label,Qt.AlignCenter)

        self.uv_label = QLabel(self)
        self.uv_label.setAlignment(Qt.AlignCenter)
        self.uv_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.uv_label.setText("UV Intensity")
        self.cabincol1.addWidget(self.uv_label,Qt.AlignCenter)

        self.lumin_label = QLabel(self)
        self.lumin_label.setAlignment(Qt.AlignCenter)
        self.lumin_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.lumin_label.setText("Luminous Intensity")
        self.cabincol1.addWidget(self.lumin_label,Qt.AlignCenter)

        self.o2_label = QLabel(self)
        self.o2_label.setAlignment(Qt.AlignCenter)
        self.o2_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.o2_label.setText("Oxygen Level")
        self.cabincol1.addWidget(self.o2_label,Qt.AlignCenter)

        self.co2_label = QLabel(self)
        self.co2_label.setAlignment(Qt.AlignCenter)
        self.co2_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.co2_label.setText("Carbon-Dioxide Level")
        self.cabincol1.addWidget(self.co2_label,Qt.AlignCenter)

        self.air_label = QLabel(self)
        self.air_label.setAlignment(Qt.AlignCenter)
        self.air_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.air_label.setText("Air Quality")
        self.cabincol1.addWidget(self.air_label,Qt.AlignCenter)

        self.tvoc_label = QLabel(self)
        self.tvoc_label.setAlignment(Qt.AlignCenter)
        self.tvoc_label.setStyleSheet("font-family: 'Red Rose'; font-size: 45px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                              )
        self.tvoc_label.setText("TVOC Levels")
        self.cabincol1.addWidget(self.tvoc_label,Qt.AlignCenter)

        cabincol2 = QVBoxLayout()
        cabindata_layout.addLayout(cabincol2)

        self.tempfr_label = QLabel(self)
        cabincol2.addWidget(self.tempfr_label,Qt.AlignCenter)

        self.humdfr_label = QLabel(self)
        cabincol2.addWidget(self.humdfr_label,Qt.AlignCenter)

        self.pressurefr_label = QLabel(self)
        cabincol2.addWidget(self.pressurefr_label,Qt.AlignCenter)

        self.uv_fr = QLabel(self)
        cabincol2.addWidget(self.uv_fr,Qt.AlignCenter)

        self.lumin_fr = QLabel(self)
        cabincol2.addWidget(self.lumin_fr,Qt.AlignCenter)

        self.o2_fr = QLabel(self)
        cabincol2.addWidget(self.o2_fr,Qt.AlignCenter)

        self.co2_fr = QLabel(self)
        cabincol2.addWidget(self.co2_fr,Qt.AlignCenter)

        self.air_fr = QLabel(self)
        cabincol2.addWidget(self.air_fr,Qt.AlignCenter)

        self.tvoc_fr = QLabel(self)
        cabincol2.addWidget(self.tvoc_fr,Qt.AlignCenter)



        #Stacked Widgets
        self.stacked_widget = QStackedWidget()
        self.stacked_widget.addWidget(self.home_qwidget)
        self.stacked_widget.addWidget(self.solar_qwidget)
        self.stacked_widget.addWidget(self.battery_qwidget)
        self.stacked_widget.addWidget(self.mc_qwidget)
        self.stacked_widget.addWidget(self.telemetry_qwidget)
        self.stacked_widget.addWidget(self.drive_qwidget)
        self.stacked_widget.addWidget(self.strategy_qwidget)
        self.stacked_widget.addWidget(self.voltamps_qwidget)
        self.stacked_widget.addWidget(self.temps_qwidget)
        self.stacked_widget.addWidget(self.circuit_qwidget)
        self.stacked_widget.addWidget(self.cabin_qwidget)
        main_layout.addWidget(self.stacked_widget)
        
        #Mouse Press event
        self.solar_widget.mousePressEvent = self.solar
        self.battery_widget.mousePressEvent = self.battery
        self.mc_widget.mousePressEvent = self.mc
        self.telemetry_widget.mousePressEvent = self.telemetry
        self.drive_widget.mousePressEvent = self.drive
        self.strategy_widget.mousePressEvent = self.strategy

    def load_html_file(self, url, layout):
        self.web_view = QWebEngineView(self)
        self.web_view.setGeometry(QRect(190, 32, 1323, 660))
        self.web_view.setUrl(QUrl(url))
        layout.addWidget(self.web_view)

    def connect_ros(self):
        try:
            # ROS2 init
            rclpy.init(args=None)
            self.node = Node('daq_node')
            self.sub = self.node.create_subscription(
                rosarray,
                self.topic_name,
                self.sub_rosarray_callback,
                10,
            )        
            
            # spin once, timeout_sec 5[s]
            timeout_sec_rclpy = 5
            timeout_init = time.time()
            rclpy.spin_once(self.node, timeout_sec=timeout_sec_rclpy)
            timeout_end = time.time()
            ros_connect_time = timeout_end - timeout_init

            # Error Handle for rclpy timeout
            if ros_connect_time >= timeout_sec_rclpy:
                print("Couldn't Connect")
            else:
                print("Connected")
                self.ros_thread = RosThread(self.node)   # Create ros thread 
                self.ros_thread.start()
        except:
            pass

    def sub_rosarray_callback(self, msg):
        self.data = msg.data
        self.data = [round(x,3) for x in self.data]
        print(self.data)

    def sensorData(self): 
        if self.ser.in_waiting > 0:
            self.data = self.ser.readline().decode().rstrip().split(",")

    def display_widget(self):
        # print(self.data)
        graph_limit = 30
        #MPPTs Graph
        self.mppt1_input_data['y'].append(self.data[93])
        self.mppt1_output_data['y'].append(self.data[93]*self.data[94])
        self.mppt1_input_data['x'].append(timestamp())
        self.mppt1_output_data['x'].append(timestamp())
        if len(self.mppt1_input_data['y']) > graph_limit:
            self.mppt1_input_data['y']= self.mppt1_input_data['y'][-30:]
            self.mppt1_input_data['x']= self.mppt1_input_data['x'][-30:]
            self.mppt1_output_data['y']= self.mppt1_output_data['y'][-30:]
            self.mppt1_output_data['x']= self.mppt1_output_data['x'][-30:]
        self.mppt1_input_plot.plot(self.mppt1_input_data['x'],self.mppt1_input_data['y'], pen='y')
        self.mppt1_output_plot.plot(self.mppt1_output_data['x'],self.mppt1_output_data['y'], pen='y')

        self.mppt2_input_data['y'].append(self.data[125])
        self.mppt2_output_data['y'].append(self.data[125]*self.data[126])
        self.mppt2_input_data['x'].append(timestamp())
        self.mppt2_output_data['x'].append(timestamp())
        if len(self.mppt2_input_data['y']) > graph_limit:
            self.mppt2_input_data['y']= self.mppt2_input_data['y'][-30:]
            self.mppt2_input_data['x']= self.mppt2_input_data['x'][-30:]
            self.mppt2_output_data['y']= self.mppt2_output_data['y'][-30:]
            self.mppt2_output_data['x']= self.mppt2_output_data['x'][-30:]
        self.mppt2_input_plot.plot(self.mppt2_input_data['x'],self.mppt2_input_data['y'], pen='y')
        self.mppt2_output_plot.plot(self.mppt2_output_data['x'],self.mppt2_output_data['y'], pen='y')

        self.mppt3_input_data['y'].append(self.data[157])
        self.mppt3_output_data['y'].append(self.data[157]*self.data[158])
        self.mppt3_input_data['x'].append(timestamp())
        self.mppt3_output_data['x'].append(timestamp())
        if len(self.mppt3_input_data['y']) > graph_limit:
            self.mppt3_input_data['y']= self.mppt3_input_data['y'][-30:]
            self.mppt3_input_data['x']= self.mppt3_input_data['x'][-30:]
            self.mppt3_output_data['y']= self.mppt3_output_data['y'][-30:]
            self.mppt3_output_data['x']= self.mppt3_output_data['x'][-30:]
        self.mppt3_input_plot.plot(self.mppt3_input_data['x'],self.mppt3_input_data['y'], pen='y')
        self.mppt3_output_plot.plot(self.mppt3_output_data['x'],self.mppt3_output_data['y'], pen='y')

        self.mppt4_input_data['y'].append(self.data[189])
        self.mppt4_output_data['y'].append(self.data[189]*self.data[190])
        self.mppt4_input_data['x'].append(timestamp())
        self.mppt4_output_data['x'].append(timestamp())
        if len(self.mppt4_input_data['y']) > graph_limit:
            self.mppt4_input_data['y']= self.mppt4_input_data['y'][-30:]
            self.mppt4_input_data['x']= self.mppt4_input_data['x'][-30:]
            self.mppt4_output_data['y']= self.mppt4_output_data['y'][-30:]
            self.mppt4_output_data['x']= self.mppt4_output_data['x'][-30:]
        self.mppt4_input_plot.plot(self.mppt4_input_data['x'],self.mppt4_input_data['y'], pen='y')
        self.mppt4_output_plot.plot(self.mppt4_output_data['x'],self.mppt4_output_data['y'], pen='y')
        
        #MC graphs
        self.buspwr_data['y'].append(self.data[242]*self.data[243])
        self.velocity_data['y'].append(self.data[245])
        self.buspwr_data['x'].append(timestamp())
        self.velocity_data['x'].append(timestamp())
        if len(self.buspwr_data['y']) > graph_limit:
            self.buspwr_data['y']= self.buspwr_data['y'][-30:]
            self.buspwr_data['x']= self.buspwr_data['x'][-30:]
            self.velocity_data['y']= self.velocity_data['y'][-30:]
            self.velocity_data['x']= self.velocity_data['x'][-30:]
        self.buspwr_plot.plot(self.velocity_data['x'],self.buspwr_data['y'], pen='y')
        self.velocity_plot.plot(self.velocity_data['x'],self.buspwr_data['y'], pen='y')
        
        self.pw_out +=1
        self.temp += 1
        if self.pw_out > 100:
            self.pw_out = 0
        if self.temp > 100:
            self.temp = 0
        
        #BATTERY BMU Telemetry
        self.minmv_val.setText(str(self.data[257]))
        self.maxmv_val.setText(str(self.data[257]))
        self.minc_val.setText(str(self.data[257]))
        self.maxc_val.setText(str(self.data[257]))
        self.packmv_val.setText(str(self.data[257]))
        self.packma_val.setText(str(self.data[257]))
        self.balp_val.setText(str(self.data[257]))
        self.balm_val.setText(str(self.data[257]))
        self.cmucount_val.setText(str(self.data[257]))
        self.prestat_val.setText(str(self.data[257]))
        self.fan1_val.setText(str(self.data[257]))
        self.fan2_val.setText(str(self.data[257]))
        self.soc1_val.setText(str(self.data[257]))
        self.soc2_val.setText(str(self.data[257]))
        self.socp1_val.setText(str(self.data[257]))
        self.socp2_val.setText(str(self.data[257]))

        #CMU Telemetry
        self.ser1_val.setText(str(self.data[257]))
        self.ser2_val.setText(str(self.data[257]))
        self.ser3_val.setText(str(self.data[257]))
        self.ser4_val.setText(str(self.data[257]))
        self.pcb1_val.setText(str(self.data[257]))
        self.pcb2_val.setText(str(self.data[257]))
        self.pcb3_val.setText(str(self.data[257]))
        self.pcb4_val.setText(str(self.data[257]))
        self.cell_val.setText(str(self.data[257]))
        self.cell1_val.setText(str(self.data[257]))
        self.cell2_val.setText(str(self.data[257]))
        self.cell3_val.setText(str(self.data[257]))
        self.cell01_val.setText(str(self.data[257]))
        self.cell02_val.setText(str(self.data[257]))
        self.cell03_val.setText(str(self.data[257]))
        self.cell04_val.setText(str(self.data[257]))
        self.cell11_val.setText(str(self.data[257]))
        self.cell12_val.setText(str(self.data[257]))
        self.cell13_val.setText(str(self.data[257]))
        self.cell14_val.setText(str(self.data[257]))
        self.cell21_val.setText(str(self.data[257]))
        self.cell22_val.setText(str(self.data[257]))
        self.cell23_val.setText(str(self.data[257]))
        self.cell24_val.setText(str(self.data[257]))
        self.cell31_val.setText(str(self.data[257]))
        self.cell32_val.setText(str(self.data[257]))
        self.cell33_val.setText(str(self.data[257]))
        self.cell34_val.setText(str(self.data[257]))
        self.cell41_val.setText(str(self.data[257]))
        self.cell42_val.setText(str(self.data[257]))
        self.cell43_val.setText(str(self.data[257]))
        self.cell44_val.setText(str(self.data[257]))
        self.cell51_val.setText(str(self.data[257]))
        self.cell52_val.setText(str(self.data[257]))
        self.cell53_val.setText(str(self.data[257]))
        self.cell54_val.setText(str(self.data[257]))
        self.cell61_val.setText(str(self.data[257]))
        self.cell62_val.setText(str(self.data[257]))
        self.cell63_val.setText(str(self.data[257]))
        self.cell64_val.setText(str(self.data[257]))
        self.cell71_val.setText(str(self.data[257]))
        self.cell72_val.setText(str(self.data[257]))
        self.cell73_val.setText(str(self.data[257]))
        self.cell74_val.setText(str(self.data[257]))    

        #MPPT 
        self.mppt1_modeval.setText(str(self.data[121]))
        self.mppt1_input_voltval.setText(str(self.data[93]))
        self.mppt1_input_currentval.setText(str(self.data[94]))
        self.mppt1_input_powerval.setText(str(self.data[93]*self.data[94]))
        self.mppt1_limitsval.setText(str("NA"))
        self.mppt1_output_voltval.setText(str(self.data[95]))
        self.mppt1_output_currentval.setText(str(self.data[96]))
        self.mppt1_output_powerval.setText(str(self.data[95]*self.data[96]))
        self.mppt1_errorval.setText(str("NA"))
        self.mppt1_mosfetval.setText(str(self.data[97]))
        self.mppt1_cntrlval.setText(str(self.data[98]))
        self.mppt1_pwrcntrlval.setText(str(self.data[124]))
        self.mppt1_aux12val.setText(str(self.data[99]))
        self.mppt1_aux3val.setText(str(self.data[100]))
        self.mppt1_output_batteryval.setText(str(self.data[123]))

        self.mppt2_modeval.setText(str(self.data[153]))
        self.mppt2_input_voltval.setText(str(self.data[125]))
        self.mppt2_input_currentval.setText(str(self.data[126]))
        self.mppt2_input_powerval.setText(str(self.data[125]*self.data[126]))
        self.mppt2_limitsval.setText(str("NA"))
        self.mppt2_output_voltval.setText(str(self.data[127]))
        self.mppt2_output_currentval.setText(str(self.data[128]))
        self.mppt2_output_powerval.setText(str(self.data[127]*self.data[128]))
        self.mppt2_errorval.setText(str("NA"))
        self.mppt2_mosfetval.setText(str(self.data[129]))
        self.mppt2_cntrlval.setText(str(self.data[130]))
        self.mppt2_pwrcntrlval.setText(str(self.data[156]))
        self.mppt2_aux12val.setText(str(self.data[131]))
        self.mppt2_aux3val.setText(str(self.data[132]))
        self.mppt2_output_batteryval.setText(str(self.data[155]))

        self.mppt3_modeval.setText(str(self.data[185]))
        self.mppt3_input_voltval.setText(str(self.data[157]))
        self.mppt3_input_currentval.setText(str(self.data[158]))
        self.mppt3_input_powerval.setText(str(self.data[157]*self.data[158]))
        self.mppt3_limitsval.setText(str("NA"))
        self.mppt3_output_voltval.setText(str(self.data[159]))
        self.mppt3_output_currentval.setText(str(self.data[160]))
        self.mppt3_output_powerval.setText(str(self.data[159]*self.data[160]))
        self.mppt3_errorval.setText(str("NA"))
        self.mppt3_mosfetval.setText(str(self.data[161]))
        self.mppt3_cntrlval.setText(str(self.data[162]))
        self.mppt3_pwrcntrlval.setText(str(self.data[188]))
        self.mppt3_aux12val.setText(str(self.data[163]))
        self.mppt3_aux3val.setText(str(self.data[164]))
        self.mppt3_output_batteryval.setText(str(self.data[187]))

        self.mppt4_modeval.setText(str(self.data[217]))
        self.mppt4_input_voltval.setText(str(self.data[189]))
        self.mppt4_input_currentval.setText(str(self.data[190]))
        self.mppt4_input_powerval.setText(str(self.data[189]*self.data[190]))
        self.mppt4_limitsval.setText(str("NA"))
        self.mppt4_output_voltval.setText(str(self.data[191]))
        self.mppt4_output_currentval.setText(str(self.data[192]))
        self.mppt4_output_powerval.setText(str(self.data[191]*self.data[192]))
        self.mppt4_errorval.setText(str("NA"))
        self.mppt4_mosfetval.setText(str(self.data[193]))
        self.mppt4_cntrlval.setText(str(self.data[194]))
        self.mppt4_pwrcntrlval.setText(str(self.data[220]))
        self.mppt4_aux12val.setText(str(self.data[195]))
        self.mppt4_aux3val.setText(str(self.data[196]))
        self.mppt4_output_batteryval.setText(str(self.data[219]))

        self.act_motorval.setText(str(self.data[239]))
        self.tri_idval.setText(str(self.data[221]))
        self.serial_noval.setText(str(self.data[222]))

        self.bus_voltval.setText(str(self.data[242]))
        self.bus_currval.setText(str(self.data[243]))
        self.bus_pwrval.setText(str(self.data[242]*self.data[243]))
        self.rms_cval.setText(str(self.data[246]))
        self.rms_bval.setText(str(self.data[247]))

        self.rpmval.setText(str(self.data[244]))
        self.kphval.setText(str(self.data[245]))
        self.vdval.setText(str(self.data[253]))
        self.vqval.setText(str(self.data[252]))

        self.motor_tempval.setText(str(self.data[257]))
        self.hs_tempval.setText(str(self.data[258]))
        self.dsp_tempval.setText(str(self.data[259]))
        self.vd1val.setText(str(self.data[249]))
        self.vq1val.setText(str(self.data[248]))

        self.rail1val.setText(str(self.data[254]))
        self.rail2val.setText(str(self.data[255]))
        self.rail3val.setText(str(self.data[256]))
        self.idval.setText(str(self.data[251]))
        self.iqval.setText(str(self.data[250]))

        #Battery Layout
        x = random.randint(0, 1)
        soc_index = 0
        if self.temp > 20:
            soc_index = 1
        else:
            soc_index = 0

        sob_index = 0
        if self.temp > 20:
            sob_index = 1
        else:
            sob_index = 0

        loadvolt_index = 0
        if self.temp > 20:
            loadvolt_index = 1
        else:
            loadvolt_index = 0

        battvolt_index = 0
        if self.temp > 20:
            battvolt_index = 1
        else:
            battvolt_index = 0

        totcurr_index = 0
        if self.temp > 20:
            totcurr_index = 1
        else:
            totcurr_index = 0

        node_index = 0
        if self.temp > 20:
            node_index = 1
        else:
             node_index = 0

        maxcell_index = 0
        if self.temp > 20:
            maxcell_index = 1
        else:
            maxcell_index = 0

        mincell_index = 0
        if self.temp > 20:
            mincell_index = 1
        else:
            mincell_index = 0

        maxtemp_index = 0
        if self.temp > 20:
            maxtemp_index = 1
        else:
            maxtemp_index = 0
        
        mintemp_index = 0
        if self.temp > 20:
            mintemp_index = 1
        else:
            mintemp_index = 0
        
        init_index = 0
        if x == 1:
            init_index = 1
        else:
            init_index = 0
        
        precharge_index = 0
        if x == 1:
            precharge_index = 1
        else:
            precharge_index = 0
        
        discharge_index = 0
        if x == 1:
            discharge_index = 1
        else:
            discharge_index = 0
        
        charge_index = 0
        if x == 1:
            charge_index = 1
        else:
            charge_index = 0
        
        all_index = 0
        if x == 1:
            all_index = 1
        else:
            all_index = 0
        
        dischargepre_index = 0
        if x == 1:
            dischargepre_index = 1
        else:
            dischargepre_index = 0

        chargepre_index = 0
        if x == 1:
            chargepre_index = 1
        else:
            charegpre_index = 0

        allpre_index = 0
        if x == 1:
            allpre_index = 1
        else:
            allpre_index = 0

        error_index = 0
        if x == 1:
            error_index = 1
        else:
            error_index = 0
        
        pixmap_nocan = QPixmap(self.nocan_images[0])
        self.nocan.setPixmap(pixmap_nocan)

        pixmap_vehiclecomms = QPixmap(self.vehiclecomms_images[0])
        self.vehiclecomms.setPixmap(pixmap_vehiclecomms)

        pixmap_cmucomms = QPixmap(self.cmucomms_images[0])
        self.cmucomms.setPixmap(pixmap_cmucomms)

        pixmap_prechargeerror = QPixmap(self.prechargeerror_images[0])
        self.prechargeerror.setPixmap(pixmap_prechargeerror)
         
        pixmap_prechargeidle = QPixmap(self.prechargeidle_images[0])
        self.prechargeidle.setPixmap(pixmap_prechargeidle)

        pixmap_onandr = QPixmap(self.onandr_images[0])
        self.onandr.setPixmap(pixmap_onandr)

        pixmap_cellov = QPixmap(self.cellov_images[0])
        self.cellov.setPixmap(pixmap_cellov)

        pixmap_celluv = QPixmap(self.celluv_images[0])
        self.celluv.setPixmap(pixmap_celluv)

        pixmap_cellot = QPixmap(self.cellot_images[0])
        self.cellot.setPixmap(pixmap_cellot)
        
        pixmap_trusterror = QPixmap(self.trusterror_images[0])
        self.trusterror.setPixmap(pixmap_trusterror)

        pixmap_packisofail = QPixmap(self.packisofail_images[0])
        self.packisofail.setPixmap(pixmap_packisofail)

        pixmap_contactorstuck = QPixmap(self.contactorstuck_images[0])
        self.contactorstuck.setPixmap(pixmap_contactorstuck)

        pixmap_contactorclosed = QPixmap(self.contactorclosed_images[0])
        self.contactorclosed.setPixmap(pixmap_contactorclosed)

        pixmap_can12vlow = QPixmap(self.can12vlow_images[0])
        self.can12vlow.setPixmap(pixmap_can12vlow)

        pixmap_cmupwrstatus = QPixmap(self.cmupwrstat_images[0])
        self.cmupwrstat.setPixmap(pixmap_cmupwrstatus)

        pixmap_extracell = QPixmap(self.extracell_images[0])
        self.extracell.setPixmap(pixmap_extracell)
        
        pixmap_socinvalid = QPixmap(self.socinvalid_images[0])
        self.socinvalid.setPixmap(pixmap_socinvalid)

        pixmap_bmu = QPixmap(self.bmusetup_images[0])
        self.bmusetup.setPixmap(pixmap_bmu)

        mppt1c_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt1c_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt1c_index = 2
        else:
            mppt1c_index = 0 

        mppt2c_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt2c_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt2c_index = 2
        else:
            mppt2c_index = 0 

        mppt3c_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt3c_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt3c_index = 2
        else:
            mppt3c_index = 0 

        mppt4c_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt4c_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt4c_index = 2
        else:
            mppt4c_index = 0 

        mppt5c_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt5c_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt5c_index = 2
        else:
            mppt5c_index = 0 

        mppt1v_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt1v_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt1v_index = 2
        else:
            mppt1v_index = 0 

        mppt2v_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt2v_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt2v_index = 2
        else:
            mppt2v_index = 0 

        mppt3v_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt3v_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt3v_index = 2
        else:
            mppt3v_index = 0 

        mppt4v_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt4v_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt4v_index = 2
        else:
            mppt4v_index = 0 

        mppt5v_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt5v_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt5v_index = 2
        else:
            mppt5v_index = 0 

        
        mppt1p_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt1p_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt1p_index = 2
        else:
            mppt1p_index = 0 

        mppt2p_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt2p_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt2p_index = 2
        else:
            mppt2p_index = 0 

        mppt3p_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt3p_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt3p_index = 2
        else:
            mppt3p_index = 0 

        mppt4p_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt4p_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt4p_index = 2
        else:
            mppt4p_index = 0 

        mppt5p_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mppt5p_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mppt5p_index = 2
        else:
            mppt5p_index = 0 

        mcamp_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mcamp_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mcamp_index = 2
        else:
            mcamp_index = 0 

        mcv_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mcv_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mcv_index = 2
        else:
            mcv_index = 0 

        mcp_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            mcp_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            mcp_index = 2
        else:
            mcp_index = 0

        batc_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            batc_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            batc_index = 2
        else:
            batc_index = 0

        batv_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            batv_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            batv_index = 2
        else:
            batv_index = 0

        batp_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            batp_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            batp_index = 2
        else:
            batp_index = 0

        batsoc_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            batsoc_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            batsoc_index = 2
        else:
            batsoc_index = 0        
        
        #Current of MPPT1 
        pixmap_motor = QPixmap(self.curr_images[mppt1c_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[7])+ " A")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[7])+ " A")
        painter_motor.end()
        self.solar_current_fr.setPixmap(pixmap_motor.scaled(225, 75))

        #Current of MPPT2 
        pixmap_motor = QPixmap(self.curr_images[mppt2c_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[7])+ " A")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[7])+ " A")
        painter_motor.end()
        self.solar_current_fr2.setPixmap(pixmap_motor.scaled(225, 75))

        #Current of MPPT3
        pixmap_motor = QPixmap(self.curr_images[mppt3c_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[7])+ " A")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[7])+ " A")
        painter_motor.end()
        self.solar_current_fr3.setPixmap(pixmap_motor.scaled(225, 75))

        #Current of MPPT4
        pixmap_motor = QPixmap(self.curr_images[mppt4c_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[7])+ " A")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[7])+ " A")
        painter_motor.end()
        self.solar_current_fr4.setPixmap(pixmap_motor.scaled(225, 75))

        #Current of MPPT5
        pixmap_motor = QPixmap(self.curr_images[mppt5c_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[7])+ " A")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[7])+ " A")
        painter_motor.end()
        self.solar_current_fr5.setPixmap(pixmap_motor.scaled(225, 75))

        #Voltage of MPPT1 
        pixmap_motor = QPixmap(self.curr_images[mppt1v_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[8])+ " V")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[8])+ " V")
        painter_motor.end()
        self.solar_voltage_fr.setPixmap(pixmap_motor.scaled(225, 75))

        #Voltage of MPPT2
        pixmap_motor = QPixmap(self.curr_images[mppt2v_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[8])+ " V")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[8])+ " V")
        painter_motor.end()
        self.solar_voltage_fr2.setPixmap(pixmap_motor.scaled(225, 75))

        #Voltage of MPPT3
        pixmap_motor = QPixmap(self.curr_images[mppt3v_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[8])+ " V")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[8])+ " V")
        painter_motor.end()
        self.solar_voltage_fr3.setPixmap(pixmap_motor.scaled(225, 75))

        #Voltage of MPPT4
        pixmap_motor = QPixmap(self.curr_images[mppt4v_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[8])+ " V")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[8])+ " V")
        painter_motor.end()
        self.solar_voltage_fr4.setPixmap(pixmap_motor.scaled(225, 75))

        #Voltage of MPPT4
        pixmap_motor = QPixmap(self.curr_images[mppt5v_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[8])+ " V")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[8])+ " V")
        painter_motor.end()
        self.solar_voltage_fr5.setPixmap(pixmap_motor.scaled(225, 75))

        #Power of MPPT1 
        pixmap_motor = QPixmap(self.curr_images[mppt1p_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[8])+ " W")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[8])+ " W")
        painter_motor.end()
        self.solar_power_fr.setPixmap(pixmap_motor.scaled(225, 75))

        #Power of MPPT2
        pixmap_motor = QPixmap(self.curr_images[mppt2p_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[8])+ " W")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[8])+ " W")
        painter_motor.end()
        self.solar_power_fr2.setPixmap(pixmap_motor.scaled(225, 75))

        #Power of MPPT3
        pixmap_motor = QPixmap(self.curr_images[mppt3p_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[8])+ " W")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[8])+ " W")
        painter_motor.end()
        self.solar_power_fr3.setPixmap(pixmap_motor.scaled(225, 75))

        #Power of MPPT4
        pixmap_motor = QPixmap(self.curr_images[mppt4p_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[8])+ " W")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[8])+ " W")
        painter_motor.end()
        self.solar_power_fr4.setPixmap(pixmap_motor.scaled(225, 75))

        #Power of MPPT5
        pixmap_motor = QPixmap(self.curr_images[mppt5p_index])
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 17))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[8])+ " W")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 35, str(self.data[8])+ " W")
        painter_motor.end()
        self.solar_power_fr5.setPixmap(pixmap_motor.scaled(225, 75))

        #Current of motor controller
        pixmap_motorctrl = QPixmap(self.curr_images[mcamp_index])
        painter_motorctrl = QPainter(pixmap_motorctrl)
        painter_motorctrl.setFont(QFont('Good Times', 17))  # Set the font and size of the variable text
        painter_motorctrl.setPen(Qt.white)  # Set the color of the variable text
        text_width_motorctrl = painter_motorctrl.fontMetrics().width(str(self.data[9]) + " A")
        x_temp = (pixmap_motorctrl.width() - text_width_motorctrl) // 2
        painter_motorctrl.drawText(x_temp, 35, str(self.data[9]) + " A")
        painter_motorctrl.end()
        self.motorctrl_current_fr.setPixmap(pixmap_motorctrl.scaled(225, 75))

        #Voltage of motor controller
        pixmap_motorctrl = QPixmap(self.curr_images[mcv_index])
        painter_motorctrl = QPainter(pixmap_motorctrl)
        painter_motorctrl.setFont(QFont('Good Times', 17))  # Set the font and size of the variable text
        painter_motorctrl.setPen(Qt.white)  # Set the color of the variable text
        text_width_motorctrl = painter_motorctrl.fontMetrics().width(str(self.data[10]) + " V")
        x_temp = (pixmap_motorctrl.width() - text_width_motorctrl) // 2
        painter_motorctrl.drawText(x_temp, 35, str(self.data[10]) + " V")
        painter_motorctrl.end()
        self.motorctrl_voltage_fr.setPixmap(pixmap_motorctrl.scaled(225, 75))

        #Power of motor controller
        pixmap_motorctrl = QPixmap(self.curr_images[mcp_index])
        painter_motorctrl = QPainter(pixmap_motorctrl)
        painter_motorctrl.setFont(QFont('Good Times', 17))  # Set the font and size of the variable text
        painter_motorctrl.setPen(Qt.white)  # Set the color of the variable text
        text_width_motorctrl = painter_motorctrl.fontMetrics().width(str(self.data[10]) + " W")
        x_temp = (pixmap_motorctrl.width() - text_width_motorctrl) // 2
        painter_motorctrl.drawText(x_temp, 35, str(self.data[10]) + " W")
        painter_motorctrl.end()
        self.motorctrl_power_fr.setPixmap(pixmap_motorctrl.scaled(225, 75))

        #Current of battery 
        pixmap_battery = QPixmap(self.curr_images[batc_index])
        painter_battery = QPainter(pixmap_battery)
        painter_battery.setFont(QFont('Good Times', 17))  # Set the font and size of the variable text
        painter_battery.setPen(Qt.white)  # Set the color of the variable text
        text_width_battery = painter_battery.fontMetrics().width(str(self.data[11])+ " A")
        x_temp = (pixmap_battery.width() - text_width_battery) // 2
        painter_battery.drawText(x_temp, 35, str(self.data[11])+ " A")
        painter_battery.end()
        self.battery_current_fr.setPixmap(pixmap_battery.scaled(225, 75))

        #Voltage of battery 
        pixmap_battery = QPixmap(self.curr_images[batv_index])
        painter_battery = QPainter(pixmap_battery)
        painter_battery.setFont(QFont('Good Times', 17))  # Set the font and size of the variable text
        painter_battery.setPen(Qt.white)  # Set the color of the variable text
        text_width_battery = painter_battery.fontMetrics().width(str(self.data[12])+ " V")
        x_temp = (pixmap_battery.width() - text_width_battery) // 2
        painter_battery.drawText(x_temp, 35, str(self.data[12])+ " V")
        painter_battery.end()
        self.battery_voltage_fr.setPixmap(pixmap_battery.scaled(225, 75))

        #SOC of battery 
        pixmap_battery = QPixmap(self.curr_images[batsoc_index])
        painter_battery = QPainter(pixmap_battery)
        painter_battery.setFont(QFont('Good Times', 17))  # Set the font and size of the variable text
        painter_battery.setPen(Qt.white)  # Set the color of the variable text
        text_width_battery = painter_battery.fontMetrics().width(str(self.data[12])+ " %")
        x_temp = (pixmap_battery.width() - text_width_battery) // 2
        painter_battery.drawText(x_temp, 35, str(self.data[12])+ " %")
        painter_battery.end()
        self.battery_soc_fr.setPixmap(pixmap_battery.scaled(225, 75))

        #Power of battery 
        pixmap_battery = QPixmap(self.curr_images[batp_index])
        painter_battery = QPainter(pixmap_battery)
        painter_battery.setFont(QFont('Good Times', 17))  # Set the font and size of the variable text
        painter_battery.setPen(Qt.white)  # Set the color of the variable text
        text_width_battery = painter_battery.fontMetrics().width(str(self.data[12])+ " W")
        x_temp = (pixmap_battery.width() - text_width_battery) // 2
        painter_battery.drawText(x_temp, 35, str(self.data[12])+ " W")
        painter_battery.end()
        self.battery_power_fr.setPixmap(pixmap_battery.scaled(225, 75))

        temp_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            temp_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            temp_index = 2

        #Solar-1 Temperature
        pixmap_temp = QPixmap(self.temp_images[temp_index])
        painter_temp = QPainter(pixmap_temp)
        painter_temp.setFont(QFont('Good Times', 20, QFont.Bold))  
        painter_temp.setPen(Qt.black)  
        text_width_temp = painter_temp.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_temp.width() - text_width_temp) // 2
        painter_temp.drawText(x_temp, 145, str(self.data[255])[:4])
        painter_temp.end()
        self.temp1_label.setPixmap(pixmap_temp.scaled(75, 150))

        #Solar-2 Temperature
        pixmap_temp2 = QPixmap(self.temp_images[temp_index])
        painter_temp2 = QPainter(pixmap_temp2)
        painter_temp2.setFont(QFont('Good Times', 20,QFont.Bold))
        painter_temp2.setPen(Qt.black)  
        text_width_temp2 = painter_temp2.fontMetrics().width(str(self.data[256])[:4])
        x_temp = (pixmap_temp2.width() - text_width_temp2) // 2
        painter_temp2.drawText(x_temp, 145, str(self.data[256])[:4])
        painter_temp2.end()
        self.temp2_label.setPixmap(pixmap_temp2.scaled(75, 150))

        #Solar-3 Controller Temperature
        pixmap_temp3 = QPixmap(self.temp_images[temp_index])
        painter_temp3 = QPainter(pixmap_temp3)
        painter_temp3.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp3.setPen(Qt.black)  
        text_width_temp3 = painter_temp3.fontMetrics().width(str(self.data[257])[:4])
        x_temp = (pixmap_temp3.width() - text_width_temp3) // 2
        painter_temp3.drawText(x_temp, 145, str(self.data[257])[:4])
        painter_temp3.end()
        self.temp3_label.setPixmap(pixmap_temp3.scaled(75, 150))

        #Solar-4 Temperature
        pixmap_temp4 = QPixmap(self.temp_images[temp_index])
        painter_temp4 = QPainter(pixmap_temp4)
        painter_temp4.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp4.setPen(Qt.black)  
        text_width_temp4 = painter_temp4.fontMetrics().width(str(self.data[258])[:4])
        x_temp = (pixmap_temp4.width() - text_width_temp4) // 2
        painter_temp4.drawText(x_temp, 145, str(self.data[258])[:4])
        painter_temp4.end()
        self.temp4_label.setPixmap(pixmap_temp4.scaled(75, 150))

        #Solar-5 Temperature
        pixmap_temp5 = QPixmap(self.temp_images[temp_index])
        painter_temp5 = QPainter(pixmap_temp5)
        painter_temp5.setFont(QFont('Good Times', 20, QFont.Bold))  
        painter_temp5.setPen(Qt.black)  
        text_width_temp5 = painter_temp5.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_temp5.width() - text_width_temp5) // 2
        painter_temp5.drawText(x_temp, 145, str(self.data[255])[:4])
        painter_temp5.end()
        self.temp5_label.setPixmap(pixmap_temp5.scaled(75, 150))

        #Solar-6 Temperature
        pixmap_temp6 = QPixmap(self.temp_images[temp_index])
        painter_temp6 = QPainter(pixmap_temp6)
        painter_temp6.setFont(QFont('Good Times', 20,QFont.Bold))
        painter_temp6.setPen(Qt.black)  
        text_width_temp6 = painter_temp6.fontMetrics().width(str(self.data[256])[:4])
        x_temp = (pixmap_temp6.width() - text_width_temp6) // 2
        painter_temp6.drawText(x_temp, 145, str(self.data[256])[:4])
        painter_temp6.end()
        self.temp6_label.setPixmap(pixmap_temp6.scaled(75, 150))

        #Solar-7 Temperature
        pixmap_temp7 = QPixmap(self.temp_images[temp_index])
        painter_temp7 = QPainter(pixmap_temp7)
        painter_temp7.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp7.setPen(Qt.black)  
        text_width_temp7 = painter_temp7.fontMetrics().width(str(self.data[257])[:4])
        x_temp = (pixmap_temp7.width() - text_width_temp7) // 2
        painter_temp7.drawText(x_temp, 145, str(self.data[257])[:4])
        painter_temp7.end()
        self.temp7_label.setPixmap(pixmap_temp3.scaled(75, 150))

        #Solar-8 Temperature
        pixmap_temp8 = QPixmap(self.temp_images[temp_index])
        painter_temp8 = QPainter(pixmap_temp8)
        painter_temp8.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp8.setPen(Qt.black)  
        text_width_temp8 = painter_temp8.fontMetrics().width(str(self.data[258])[:4])
        x_temp = (pixmap_temp8.width() - text_width_temp8) // 2
        painter_temp8.drawText(x_temp, 145, str(self.data[258])[:4])
        painter_temp8.end()
        self.temp8_label.setPixmap(pixmap_temp8.scaled(75, 150))

        #Solar-9 Temperature
        pixmap_temp9 = QPixmap(self.temp_images[temp_index])
        painter_temp9 = QPainter(pixmap_temp9)
        painter_temp9.setFont(QFont('Good Times', 20, QFont.Bold))  
        painter_temp9.setPen(Qt.black)  
        text_width_temp9 = painter_temp9.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_temp9.width() - text_width_temp9) // 2
        painter_temp9.drawText(x_temp, 145, str(self.data[255])[:4])
        painter_temp9.end()
        self.temp9_label.setPixmap(pixmap_temp9.scaled(75, 150))

        #Solar-10 Temperature
        pixmap_temp10 = QPixmap(self.temp_images[temp_index])
        painter_temp10 = QPainter(pixmap_temp10)
        painter_temp10.setFont(QFont('Good Times', 20,QFont.Bold))
        painter_temp10.setPen(Qt.black)  
        text_width_temp10 = painter_temp10.fontMetrics().width(str(self.data[256])[:4])
        x_temp = (pixmap_temp10.width() - text_width_temp10) // 2
        painter_temp10.drawText(x_temp, 145, str(self.data[256])[:4])
        painter_temp10.end()
        self.temp10_label.setPixmap(pixmap_temp10.scaled(75, 150))

        #Solar-11 Controller Temperature
        pixmap_temp11 = QPixmap(self.temp_images[temp_index])
        painter_temp11 = QPainter(pixmap_temp11)
        painter_temp11.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp11.setPen(Qt.black)  
        text_width_temp11 = painter_temp11.fontMetrics().width(str(self.data[257])[:4])
        x_temp = (pixmap_temp11.width() - text_width_temp11) // 2
        painter_temp11.drawText(x_temp, 145, str(self.data[257])[:4])
        painter_temp11.end()
        self.temp11_label.setPixmap(pixmap_temp11.scaled(75, 150))

        #Solar-12 Temperature
        pixmap_temp12 = QPixmap(self.temp_images[temp_index])
        painter_temp12 = QPainter(pixmap_temp12)
        painter_temp12.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp12.setPen(Qt.black)  
        text_width_temp12 = painter_temp12.fontMetrics().width(str(self.data[258])[:4])
        x_temp = (pixmap_temp4.width() - text_width_temp12) // 2
        painter_temp12.drawText(x_temp, 145, str(self.data[258])[:4])
        painter_temp12.end()
        self.temp12_label.setPixmap(pixmap_temp12.scaled(75, 150))

        #MPPT-1 Temperature
        pixmap_temp13 = QPixmap(self.temp_images[temp_index])
        painter_temp13 = QPainter(pixmap_temp13)
        painter_temp13.setFont(QFont('Good Times', 20, QFont.Bold))  
        painter_temp13.setPen(Qt.black)  
        text_width_temp13 = painter_temp13.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_temp13.width() - text_width_temp13) // 2
        painter_temp13.drawText(x_temp, 145, str(self.data[255])[:4])
        painter_temp13.end()
        self.temp13_label.setPixmap(pixmap_temp13.scaled(75, 150))

        #MPPT-2 Temperature
        pixmap_temp14 = QPixmap(self.temp_images[temp_index])
        painter_temp14 = QPainter(pixmap_temp14)
        painter_temp14.setFont(QFont('Good Times', 20,QFont.Bold))
        painter_temp14.setPen(Qt.black)  
        text_width_temp14 = painter_temp14.fontMetrics().width(str(self.data[256])[:4])
        x_temp = (pixmap_temp14.width() - text_width_temp14) // 2
        painter_temp14.drawText(x_temp, 145, str(self.data[256])[:4])
        painter_temp14.end()
        self.temp14_label.setPixmap(pixmap_temp14.scaled(75, 150))

        #MPPT-3 Temperature
        pixmap_temp15 = QPixmap(self.temp_images[temp_index])
        painter_temp15 = QPainter(pixmap_temp15)
        painter_temp15.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp15.setPen(Qt.black)  
        text_width_temp15 = painter_temp15.fontMetrics().width(str(self.data[257])[:4])
        x_temp = (pixmap_temp15.width() - text_width_temp15) // 2
        painter_temp15.drawText(x_temp, 145, str(self.data[257])[:4])
        painter_temp15.end()
        self.temp15_label.setPixmap(pixmap_temp15.scaled(75, 150))

        #MPPT-4 Temperature
        pixmap_temp16 = QPixmap(self.temp_images[temp_index])
        painter_temp16 = QPainter(pixmap_temp16)
        painter_temp16.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp16.setPen(Qt.black)  
        text_width_temp16 = painter_temp16.fontMetrics().width(str(self.data[258])[:4])
        x_temp = (pixmap_temp16.width() - text_width_temp16) // 2
        painter_temp16.drawText(x_temp, 145, str(self.data[258])[:4])
        painter_temp16.end()
        self.temp16_label.setPixmap(pixmap_temp16.scaled(75, 150))

        #Motor cntrl Temperatur
        pixmap_temp17 = QPixmap(self.temp_images[temp_index])
        painter_temp17 = QPainter(pixmap_temp17)
        painter_temp17.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp17.setPen(Qt.black)  
        text_width_temp17 = painter_temp17.fontMetrics().width(str(self.data[257])[:4])
        x_temp = (pixmap_temp17.width() - text_width_temp17) // 2
        painter_temp17.drawText(x_temp, 145, str(self.data[257])[:4])
        painter_temp17.end()
        self.temp17_label.setPixmap(pixmap_temp17.scaled(75, 150))

        #Motor Temperature
        pixmap_temp18 = QPixmap(self.temp_images[temp_index])
        painter_temp18 = QPainter(pixmap_temp18)
        painter_temp18.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp18.setPen(Qt.black)  
        text_width_temp18 = painter_temp18.fontMetrics().width(str(self.data[258])[:4])
        x_temp = (pixmap_temp18.width() - text_width_temp18) // 2
        painter_temp18.drawText(x_temp, 145, str(self.data[258])[:4])
        painter_temp18.end()
        self.temp18_label.setPixmap(pixmap_temp18.scaled(75, 150))

        #Batery-1 Temp
        pixmap_temp19 = QPixmap(self.temp_images[temp_index])
        painter_temp19 = QPainter(pixmap_temp19)
        painter_temp19.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp19.setPen(Qt.black)  
        text_width_temp19 = painter_temp19.fontMetrics().width(str(self.data[258])[:4])
        x_temp = (pixmap_temp19.width() - text_width_temp19) // 2
        painter_temp19.drawText(x_temp, 145, str(self.data[258])[:4])
        painter_temp19.end()
        self.temp19_label.setPixmap(pixmap_temp19.scaled(75, 150))

        #Battery-2 Temperature
        pixmap_temp20 = QPixmap(self.temp_images[temp_index])
        painter_temp20 = QPainter(pixmap_temp20)
        painter_temp20.setFont(QFont('Good Times', 20,QFont.Bold))
        painter_temp20.setPen(Qt.black)  
        text_width_temp20 = painter_temp20.fontMetrics().width(str(self.data[256])[:4])
        x_temp = (pixmap_temp20.width() - text_width_temp20) // 2
        painter_temp20.drawText(x_temp, 145, str(self.data[256])[:4])
        painter_temp20.end()
        self.temp20_label.setPixmap(pixmap_temp20.scaled(75, 150))

        #Battery-3 Temperature
        pixmap_temp21 = QPixmap(self.temp_images[temp_index])
        painter_temp21 = QPainter(pixmap_temp21)
        painter_temp21.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp21.setPen(Qt.black)  
        text_width_temp21 = painter_temp21.fontMetrics().width(str(self.data[257])[:4])
        x_temp = (pixmap_temp21.width() - text_width_temp21) // 2
        painter_temp21.drawText(x_temp, 145, str(self.data[257])[:4])
        painter_temp21.end()
        self.temp21_label.setPixmap(pixmap_temp21.scaled(75, 150))

        #Battery-4 Temperature
        pixmap_temp22 = QPixmap(self.temp_images[temp_index])
        painter_temp22 = QPainter(pixmap_temp22)
        painter_temp22.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp22.setPen(Qt.black)  
        text_width_temp22 = painter_temp22.fontMetrics().width(str(self.data[258])[:4])
        x_temp = (pixmap_temp22.width() - text_width_temp22) // 2
        painter_temp22.drawText(x_temp, 145, str(self.data[258])[:4])
        painter_temp22.end()
        self.temp22_label.setPixmap(pixmap_temp22.scaled(75, 150))

        #Cabin Temperature
        pixmap_temp23 = QPixmap(self.temp_images[temp_index])
        painter_temp23 = QPainter(pixmap_temp23)
        painter_temp23.setFont(QFont('Good Times', 20,QFont.Bold)) 
        painter_temp23.setPen(Qt.black)  
        text_width_temp23 = painter_temp23.fontMetrics().width(str(self.data[257])[:4])
        x_temp = (pixmap_temp23.width() - text_width_temp23) // 2
        painter_temp23.drawText(x_temp, 145, str(self.data[257])[:4])
        painter_temp23.end()
        self.temp23_label.setPixmap(pixmap_temp23.scaled(75, 150))

        #Telemetry
        radio_index = 0 
        if self.data[5] == 1:
            radio_index = 1
        else:
            radio_index

        recst_index = 0 
        if self.data[5] == 1:
            recst_index = 1
        else:
            recst_index

        transt_index = 0 
        if self.data[5] == 1:
            transt_index = 1
        else:
            transt_index

        #Radio state
        pixmap_radiost = QPixmap(self.radio_images[radio_index])
        self.radiost_label.setPixmap(pixmap_radiost.scaled(221, 52))

        #Trasnmitting
        pixmap_transt = QPixmap(self.radio_images[transt_index])
        self.transt_label.setPixmap(pixmap_transt.scaled(221, 52))

        #Receiving
        pixmap_recst = QPixmap(self.radio_images[recst_index])
        self.recst_label.setPixmap(pixmap_recst.scaled(221, 52))

        #RF RSSI
        pixmap_rssi = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/radiofr.png")
        painter_rssi = QPainter(pixmap_rssi)
        painter_rssi.setFont(QFont('Red Rose', 25))  # Set the font and size of the variable text
        painter_rssi.setPen(Qt.white)  # Set the color of the variable text
        text_width_rssi = painter_rssi.fontMetrics().width(str(self.data[258])[:4])
        x_temp = (pixmap_rssi.width() - text_width_rssi) // 2
        painter_rssi.drawText(x_temp, 35, str(self.data[258])[:4])
        painter_rssi.end()
        self.rssi_fr.setPixmap(pixmap_rssi.scaled(221, 52))

        #SNR
        pixmap_snr = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/radiofr.png")
        painter_snr = QPainter(pixmap_snr)
        painter_snr.setFont(QFont('Red Rose', 25))  # Set the font and size of the variable text
        painter_snr.setPen(Qt.white)  # Set the color of the variable text
        text_width_snr = painter_snr.fontMetrics().width(str(self.data[258])[:4])
        x_temp = (pixmap_snr.width() - text_width_snr) // 2
        painter_snr.drawText(x_temp, 35, str(self.data[258])[:4])
        painter_snr.end()
        self.snr_fr.setPixmap(pixmap_snr.scaled(221, 52))

        #Message Received
        pixmap_msgrec = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/msgfr.png")
        painter_msgrec = QPainter(pixmap_msgrec)
        painter_msgrec.setFont(QFont('Red Rose', 25))  # Set the font and size of the variable text
        painter_msgrec.setPen(Qt.white)  # Set the color of the variable text
        text_width_msgrec = painter_msgrec.fontMetrics().width(str(self.data[258])[:4])
        x_temp = (pixmap_msgrec.width() - text_width_msgrec) // 2
        painter_msgrec.drawText(x_temp, 35, str(self.data[258])[:4])
        painter_msgrec.end()
        self.msgrec_fr.setPixmap(pixmap_msgrec.scaled(900, 52))

        #Message Received
        pixmap_msgsent = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/msgfr.png")
        painter_msgsent = QPainter(pixmap_msgsent)
        painter_msgsent.setFont(QFont('Red Rose', 25))  # Set the font and size of the variable text
        painter_msgsent.setPen(Qt.white)  # Set the color of the variable text
        text_width_msgsent = painter_msgsent.fontMetrics().width(str(self.data[258])[:4])
        x_temp = (pixmap_msgsent.width() - text_width_msgsent) // 2
        painter_msgsent.drawText(x_temp, 35, str(self.data[258])[:4])
        painter_msgsent.end()
        self.msgsent_fr.setPixmap(pixmap_msgsent.scaled(900, 52))    

        #CABIN
        cbtemp_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            cbtemp_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            cbtemp_index = 2
        else:
            cbtemp_index =0 

        humd_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            humd_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            humd_index = 2
        else:
            humd_index =0 

        
        pressure_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            pressure_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            pressure_index = 2
        else:
            pressure_index =0 

        uv_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            uv_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            uv_index = 2
        else:
            uv_index =0 

        lumin_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            lumin_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            lumin_index = 2
        else:
            lumin_index =0 

        o2_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            o2_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            o2_index = 2
        else:
            o2_index =0 

        co2_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            co2_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            co2_index = 2
        else:
            co2_index =0 

        air_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            air_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            air_index = 2
        else:
            air_index =0 

        tvoc_index = 0
        if self.data[258]> 45 and self.data[258] < 60:
            tvoc_index = 1
        elif self.data[258] >= 60 and self.data[258] < 80:
            tvoc_index = 2
        else:
            tvoc_index =0 

        #Cabin Temperature
        pixmap_cabin1 = QPixmap(self.cabin_images[cbtemp_index])
        painter_cabin1 = QPainter(pixmap_cabin1)
        painter_cabin1.setFont(QFont('Good Times', 25, QFont.Bold))  
        painter_cabin1.setPen(Qt.white)  
        text_width_cabin1 = painter_cabin1.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_cabin1.width() - text_width_cabin1) // 2
        painter_cabin1.drawText(x_temp, 39, str(self.data[255])[:4])
        painter_cabin1.end()
        self.tempfr_label.setPixmap(pixmap_cabin1.scaled(221, 52))

        pixmap_cabin2 = QPixmap(self.cabin_images[humd_index])
        painter_cabin2 = QPainter(pixmap_cabin2)
        painter_cabin2.setFont(QFont('Good Times', 25, QFont.Bold))  
        painter_cabin2.setPen(Qt.white)  
        text_width_cabin2 = painter_cabin2.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_cabin2.width() - text_width_cabin2) // 2
        painter_cabin2.drawText(x_temp, 39, str(self.data[255])[:4])
        painter_cabin2.end()
        self.humdfr_label.setPixmap(pixmap_cabin2.scaled(221, 52))

        pixmap_cabin3 = QPixmap(self.cabin_images[pressure_index])
        painter_cabin3 = QPainter(pixmap_cabin3)
        painter_cabin3.setFont(QFont('Good Times', 25, QFont.Bold))  
        painter_cabin3.setPen(Qt.white)  
        text_width_cabin3 = painter_cabin3.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_cabin3.width() - text_width_cabin3) // 2
        painter_cabin3.drawText(x_temp, 39, str(self.data[255])[:4])
        painter_cabin3.end()
        self.pressurefr_label.setPixmap(pixmap_cabin3.scaled(221, 52))

        pixmap_cabin4 = QPixmap(self.cabin_images[uv_index])
        painter_cabin4 = QPainter(pixmap_cabin4)
        painter_cabin4.setFont(QFont('Good Times', 25, QFont.Bold))  
        painter_cabin4.setPen(Qt.white)  
        text_width_cabin4 = painter_cabin4.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_cabin4.width() - text_width_cabin4) // 2
        painter_cabin4.drawText(x_temp, 39, str(self.data[255])[:4])
        painter_cabin4.end()
        self.uv_fr.setPixmap(pixmap_cabin4.scaled(221, 52))

        pixmap_cabin5 = QPixmap(self.cabin_images[lumin_index])
        painter_cabin5 = QPainter(pixmap_cabin5)
        painter_cabin5.setFont(QFont('Good Times', 25, QFont.Bold))  
        painter_cabin5.setPen(Qt.white)  
        text_width_cabin5 = painter_cabin5.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_cabin5.width() - text_width_cabin5) // 2
        painter_cabin5.drawText(x_temp, 39, str(self.data[255])[:4])
        painter_cabin5.end()
        self.lumin_fr.setPixmap(pixmap_cabin5.scaled(221, 52))

        pixmap_cabin6 = QPixmap(self.cabin_images[o2_index])
        painter_cabin6 = QPainter(pixmap_cabin6)
        painter_cabin6.setFont(QFont('Good Times', 25, QFont.Bold))  
        painter_cabin6.setPen(Qt.white)  
        text_width_cabin6 = painter_cabin6.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_cabin6.width() - text_width_cabin6) // 2
        painter_cabin6.drawText(x_temp, 39, str(self.data[255])[:4])
        painter_cabin6.end()
        self.o2_fr.setPixmap(pixmap_cabin6.scaled(221, 52))

        pixmap_cabin7 = QPixmap(self.cabin_images[co2_index])
        painter_cabin7 = QPainter(pixmap_cabin7)
        painter_cabin7.setFont(QFont('Good Times', 25, QFont.Bold))  
        painter_cabin7.setPen(Qt.white)  
        text_width_cabin7 = painter_cabin7.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_cabin7.width() - text_width_cabin7) // 2
        painter_cabin7.drawText(x_temp, 39, str(self.data[255])[:4])
        painter_cabin7.end()
        self.co2_fr.setPixmap(pixmap_cabin7.scaled(221, 52))

        pixmap_cabin8 = QPixmap(self.cabin_images[air_index])
        painter_cabin8 = QPainter(pixmap_cabin8)
        painter_cabin8.setFont(QFont('Good Times', 25, QFont.Bold))  
        painter_cabin8.setPen(Qt.white)  
        text_width_cabin8 = painter_cabin8.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_cabin8.width() - text_width_cabin8) // 2
        painter_cabin8.drawText(x_temp, 39, str(self.data[255])[:4])
        painter_cabin8.end()
        self.air_fr.setPixmap(pixmap_cabin8.scaled(221, 52))

        pixmap_cabin9 = QPixmap(self.cabin_images[tvoc_index])
        painter_cabin9 = QPainter(pixmap_cabin9)
        painter_cabin9.setFont(QFont('Good Times', 25, QFont.Bold))  
        painter_cabin9.setPen(Qt.white)  
        text_width_cabin9 = painter_cabin9.fontMetrics().width(str(self.data[255])[:4])
        x_temp = (pixmap_cabin9.width() - text_width_cabin9) // 2
        painter_cabin9.drawText(x_temp, 39, str(self.data[255])[:4])
        painter_cabin9.end()
        self.tvoc_fr.setPixmap(pixmap_cabin9.scaled(221, 52))

                
        #Solar Widget
        pixmap_solar = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Solar_widget.png")
        painter_solar = QPainter(pixmap_solar)
        painter_solar.setFont(QFont('Bookman Uralic', 25))  
        painter_solar.setPen(Qt.white)  
        painter_solar.drawText(35, 150, "PW OUT:" + str(self.pw_out)+ "V")
        painter_solar.drawText(35, 265, "TEMP:" + str(self.temp)+ "°C")
        painter_solar.end()
        self.solar_widget.setPixmap(pixmap_solar.scaled(320, 320))
        #Battery Widget
        pixmap_battery = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Battery_widget.png")
        painter_battery = QPainter(pixmap_battery)
        painter_battery.setFont(QFont('Bookman Uralic', 25))  
        painter_battery.setPen(Qt.white)  
        painter_battery.drawText(35, 150, "PW OUT:" + str(self.pw_out)+ "V")
        painter_battery.drawText(35, 265, "TEMP:" + str(self.temp)+ "°C")
        painter_battery.end()
        self.battery_widget.setPixmap(pixmap_battery.scaled(320, 320))
        #MC Widget
        pixmap_MC = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/MC_widget.png")
        painter_MC = QPainter(pixmap_MC)
        painter_MC.setFont(QFont('Bookman Uralic', 25))  
        painter_MC.setPen(Qt.white)  
        painter_MC.drawText(35, 150, "PW OUT:" + str(self.pw_out)+ "V")
        painter_MC.drawText(35, 265, "TEMP:" + str(self.temp)+ "°C")
        painter_MC.end()
        self.mc_widget.setPixmap(pixmap_MC.scaled(320, 320))
        #Telemetry Widget
        pixmap_telemetry = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Telemetry_widget.png")
        painter_telemetry = QPainter(pixmap_telemetry)
        painter_telemetry.setFont(QFont('Bookman Uralic', 25))  
        painter_telemetry.setPen(Qt.white)  
        painter_telemetry.drawText(35, 150, "PW OUT:" + str(self.pw_out)+ "V")
        painter_telemetry.drawText(35, 265, "TEMP:" + str(self.temp)+ "°C")
        painter_telemetry.end()
        self.telemetry_widget.setPixmap(pixmap_telemetry.scaled(400, 320))
        #Drive Widget
        pixmap_drive = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Drive_widget.png")
        painter_drive = QPainter(pixmap_drive)
        painter_drive.setFont(QFont('Bookman Uralic', 25))  
        painter_drive.setPen(Qt.white)  
        painter_drive.drawText(35, 150, "PW OUT:" + str(self.pw_out)+ "V")
        painter_drive.drawText(35, 265, "TEMP:" + str(self.temp)+ "°C")
        painter_drive.end()
        self.drive_widget.setPixmap(pixmap_drive.scaled(400, 320))
        #Strategy Widget
        pixmap_strategy = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Startegy_widget.png")
        painter_strategy = QPainter(pixmap_strategy)
        painter_strategy.setFont(QFont('Bookman Uralic', 25)) 
        painter_strategy.setPen(Qt.white)  
        painter_strategy.drawText(55, 150, "PW OUT:" + str(self.pw_out)+ "V")
        painter_strategy.drawText(55, 265, "TEMP:" + str(self.temp)+ "°C")
        painter_strategy.end()
        self.strategy_widget.setPixmap(pixmap_strategy.scaled(400, 320))
    
    def ros(self, event):
        self.start_stack.setCurrentIndex(1)
        self.connect_ros()
    
    def serial(self, event):
        self.start_stack.setCurrentIndex(1)
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        timer = QTimer(self)
        timer.timeout.connect(self.sensorData)
        timer.start(100)

    def home(self, event):
        self.stacked_widget.setCurrentIndex(0)

    def solar(self, event):
        self.stacked_widget.setCurrentIndex(1)

    def battery(self, event):
        self.stacked_widget.setCurrentIndex(2)

    def mc(self, event):
        self.stacked_widget.setCurrentIndex(3)

    def telemetry(self, event):
        self.stacked_widget.setCurrentIndex(4)

    def drive(self, event):
        self.stacked_widget.setCurrentIndex(5)

    def strategy(self, event):
        self.stacked_widget.setCurrentIndex(6)

    def voltamps(self, event):
        self.stacked_widget.setCurrentIndex(7)

    def temps(self, event):
        self.stacked_widget.setCurrentIndex(8)

    def circuit(self, event):
        self.stacked_widget.setCurrentIndex(9)

    def cabin(self, event):
        self.stacked_widget.setCurrentIndex(10)

def main():
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())

        
if __name__ == "__main__":
    main()
    

