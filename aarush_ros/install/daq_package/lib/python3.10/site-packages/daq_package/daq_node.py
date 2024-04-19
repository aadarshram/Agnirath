import sys
import numpy as np
import random
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QWidget, QTextEdit, QSpacerItem, QStackedWidget, QLineEdit, QSizePolicy, QTableWidget, QTableWidgetItem, QGridLayout
from PyQt5.QtGui import QColor, QPalette, QPixmap, QIcon, QPainter, QFont
from PyQt5.QtCore import Qt, QRect, QUrl, QTimer, QThread
from PyQt5.QtWebEngineWidgets import QWebEngineView
import pyqtgraph as pg
import datetime
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray

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
        self.topic_name = '/final_data'
        self.connect_ros()

        # Set background color
        background_color = QColor(0x1C, 0x1B, 0x1A)
        palette = QPalette()
        palette.setColor(QPalette.Window, background_color)
        self.setPalette(palette)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        main_layout = QHBoxLayout()
        self.central_widget.setLayout(main_layout)

        #BUTTONS VBoxlayout
        button_layout = QVBoxLayout()
        main_layout.addLayout(button_layout)
        button1_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/home.png")
        button2_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/volt_amps.png")
        button3_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Temps.png")
        button4_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/circuit.png")
        button5_image = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/TBD.png")

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
        self.button4.clicked.connect(self.circuit)

        self.button5 = QPushButton(self)
        self.button5.setIcon(QIcon(button5_image))
        self.button5.setIconSize(button5_image.size())
        self.button5.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        # self.button5.clicked.connect(self.CPanel)

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
        self.battery_layout = QHBoxLayout()
        self.battery_qwidget.setLayout(self.battery_layout)
        self.mc_qwidget = QWidget()
        self.mc_layout = QVBoxLayout()
        self.mc_qwidget.setLayout(self.mc_layout)
        self.telemetry_qwidget = QWidget()
        self.telemetry_layout = QHBoxLayout()
        self.telemetry_qwidget.setLayout(self.telemetry_layout)
        self.drive_qwidget = QWidget()
        self.drive_layout = QHBoxLayout()
        self.drive_qwidget.setLayout(self.drive_layout)
        self.strategy_qwidget = QWidget()
        self.strategy_layout = QHBoxLayout()
        self.strategy_qwidget.setLayout(self.strategy_layout)
        self.voltamps_qwidget = QWidget()
        self.voltamps_layout = QHBoxLayout()
        self.voltamps_qwidget.setLayout(self.voltamps_layout)
        self.temps_qwidget = QWidget()
        self.temps_layout = QHBoxLayout()
        self.temps_qwidget.setLayout(self.temps_layout)
        self.circuit_qwidget = QWidget()
        self.circuit_layout = QHBoxLayout()
        self.circuit_qwidget.setLayout(self.circuit_layout)
        
        #HOME LAYOUT
        #Map and 3 widgets
        self.map_layout = QVBoxLayout()
        h_spacer2 = QSpacerItem(70, 10) 
        # main_layout.addItem(h_spacer2)
        self.home_layout.addLayout(self.map_layout)
        url = "http://127.0.0.1:5500/GUI/Beaglebone/Dashboard/mapV2.html"  
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
        self.act_motor.setStyleSheet("color: white; font-weight: bold;")
        mcdata0_layout.addWidget(self.act_motor)
        self.act_motorval = QLineEdit(self)
        self.act_motorval.setReadOnly(True)
        mcdata0_layout.addWidget(self.act_motorval)
        self.tri_id = QLabel()
        self.tri_id.setText("Tritium ID")
        self.tri_id.setStyleSheet("color: white; font-weight: bold;")
        mcdata0_layout.addWidget(self.tri_id)
        self.tri_idval = QLineEdit(self)
        self.tri_idval.setReadOnly(True)
        mcdata0_layout.addWidget(self.tri_idval)
        self.serial_no = QLabel()
        self.serial_no.setText("Serial No.")
        self.serial_no.setStyleSheet("color: white; font-weight: bold;")
        mcdata0_layout.addWidget(self.serial_no)
        self.serial_noval = QLineEdit(self)
        self.serial_noval.setReadOnly(True)
        mcdata0_layout.addWidget(self.serial_noval)

        mcdata_layout =  QHBoxLayout()
        self.mc_layout.addLayout(mcdata_layout)
        col1_layout = QVBoxLayout()
        mcdata_layout.addLayout(col1_layout)        
        spacer = QSpacerItem(10, 30) 
        col1_layout.addItem(spacer)
        self.power = QLabel()
        self.power.setText("Power")
        self.power.setStyleSheet("color: white; font-weight: bold;")
        col1_layout.addWidget(self.power)
        self.bus_volt = QLabel()
        self.bus_volt.setText("Bus Voltage")
        self.bus_volt.setStyleSheet("color: white;")
        col1_layout.addWidget(self.bus_volt)
        self.bus_curr = QLabel()
        self.bus_curr.setText("Bus Current")
        self.bus_curr.setStyleSheet("color: white;")
        col1_layout.addWidget(self.bus_curr)
        self.bus_pwr = QLabel()
        self.bus_pwr.setText("Bus Power")
        self.bus_pwr.setStyleSheet("color: white;")
        col1_layout.addWidget(self.bus_pwr)
        spacer = QSpacerItem(10, 30) 
        col1_layout.addItem(spacer)
        self.phase_curr = QLabel()
        self.phase_curr.setText("Phase Current")
        self.phase_curr.setStyleSheet("color: white; font-weight: bold;")
        col1_layout.addWidget(self.phase_curr)
        self.rms_c = QLabel()
        self.rms_c.setText("RMS Current C")
        self.rms_c.setStyleSheet("color: white;")
        col1_layout.addWidget(self.rms_c)
        self.rms_b = QLabel()
        self.rms_b.setText("RMS Current B")
        self.rms_b.setStyleSheet("color: white;")
        col1_layout.addWidget(self.rms_b)

        col2_layout = QVBoxLayout()
        mcdata_layout.addLayout(col2_layout)
        spacer = QSpacerItem(10, 60)  
        col2_layout.addItem(spacer)
        self.bus_voltval = QLineEdit(self)
        self.bus_voltval.setReadOnly(True)
        col2_layout.addWidget(self.bus_voltval)
        spacer = QSpacerItem(10, 1)  
        col2_layout.addItem(spacer)
        self.bus_currval = QLineEdit(self)
        self.bus_currval.setReadOnly(True)
        col2_layout.addWidget(self.bus_currval)
        spacer = QSpacerItem(10, 1)  
        col2_layout.addItem(spacer)
        self.bus_pwrval = QLineEdit(self)
        self.bus_pwrval.setReadOnly(True)
        col2_layout.addWidget(self.bus_pwrval)
        spacer = QSpacerItem(10, 60)  
        col2_layout.addItem(spacer)
        self.rms_cval = QLineEdit(self)
        self.rms_cval.setReadOnly(True)
        col2_layout.addWidget(self.rms_cval)
        self.rms_bval = QLineEdit(self)
        self.rms_bval.setReadOnly(True)
        col2_layout.addWidget(self.rms_bval)

        col3_layout = QVBoxLayout()
        mcdata_layout.addLayout(col3_layout)
        spacer = QSpacerItem(10, 30) 
        col3_layout.addItem(spacer)
        self.velocity = QLabel()
        self.velocity.setText("Velocity")
        self.velocity.setStyleSheet("color: white; font-weight: bold;")
        col3_layout.addWidget(self.velocity)
        # spacer = QSpacerItem(10, -18) 
        # col3_layout.addItem(spacer)
        self.rpm = QLabel()
        self.rpm.setText("rpm")
        self.rpm.setStyleSheet("color: white;")
        col3_layout.addWidget(self.rpm)
        # spacer = QSpacerItem(10, -11)  
        # col3_layout.addItem(spacer)
        self.kph = QLabel()
        self.kph.setText("kph")
        self.kph.setStyleSheet("color: white;")
        col3_layout.addWidget(self.kph)
        spacer = QSpacerItem(10, 70) 
        col3_layout.addItem(spacer)
        self.bemf = QLabel()
        self.bemf.setText("Motor BEMF")
        self.bemf.setStyleSheet("color: white; font-weight: bold;")
        col3_layout.addWidget(self.bemf)
        self.vd = QLabel()
        self.vd.setText("Vd")
        self.vd.setStyleSheet("color: white;")
        col3_layout.addWidget(self.vd)
        # spacer = QSpacerItem(10, 30) 
        # col3_layout.addItem(spacer)
        self.vq = QLabel()
        self.vq.setText("Vq")
        self.vq.setStyleSheet("color: white;")
        col3_layout.addWidget(self.vq)

        col4_layout = QVBoxLayout()
        mcdata_layout.addLayout(col4_layout)
        spacer = QSpacerItem(10, 60)  
        col4_layout.addItem(spacer)
        self.rpmval = QLineEdit(self)
        self.rpmval.setReadOnly(True)
        col4_layout.addWidget(self.rpmval)
        # spacer = QSpacerItem(10, 70)  
        # col4_layout.addItem(spacer)
        self.kphval = QLineEdit(self)
        self.kphval.setReadOnly(True)
        col4_layout.addWidget(self.kphval)
        spacer = QSpacerItem(10, 100)  
        col4_layout.addItem(spacer)
        self.vdval = QLineEdit(self)
        self.vdval.setReadOnly(True)
        col4_layout.addWidget(self.vdval)
        # spacer = QSpacerItem(10, 1)  
        # col4_layout.addItem(spacer)
        self.vqval = QLineEdit(self)
        self.vqval.setReadOnly(True)
        col4_layout.addWidget(self.vqval)

        col5_layout = QVBoxLayout()
        mcdata_layout.addLayout(col5_layout)        
        spacer = QSpacerItem(10, 30) 
        col5_layout.addItem(spacer)
        self.mc_temps = QLabel()
        self.mc_temps.setText("Temperatures")
        self.mc_temps.setStyleSheet("color: white; font-weight: bold;")
        col5_layout.addWidget(self.mc_temps)
        self.motor_temp = QLabel()
        self.motor_temp.setText("Motor Temp")
        self.motor_temp.setStyleSheet("color: white;")
        col5_layout.addWidget(self.motor_temp)
        self.hs_temp = QLabel()
        self.hs_temp.setText("Heatsink Temp")
        self.hs_temp.setStyleSheet("color: white;")
        col5_layout.addWidget(self.hs_temp)
        self.dsp_temp = QLabel()
        self.dsp_temp.setText("DSP Temp")
        self.dsp_temp.setStyleSheet("color: white;")
        col5_layout.addWidget(self.dsp_temp)
        spacer = QSpacerItem(10, 30) 
        col5_layout.addItem(spacer)
        self.motor_volt = QLabel()
        self.motor_volt.setText("Motor Voltage")
        self.motor_volt.setStyleSheet("color: white; font-weight: bold;")
        col5_layout.addWidget(self.motor_volt)
        self.vd1 = QLabel()
        self.vd1.setText("Vd")
        self.vd1.setStyleSheet("color: white;")
        col5_layout.addWidget(self.vd1)
        self.vq1 = QLabel()
        self.vq1.setText("Vq")
        self.vq1.setStyleSheet("color: white;")
        col5_layout.addWidget(self.vq1)

        col6_layout = QVBoxLayout()
        mcdata_layout.addLayout(col6_layout)
        spacer = QSpacerItem(10, 60)  
        col6_layout.addItem(spacer)
        self.motor_tempval = QLineEdit(self)
        self.motor_tempval.setReadOnly(True)
        col6_layout.addWidget(self.motor_tempval)
        spacer = QSpacerItem(10, 1)  
        col6_layout.addItem(spacer)
        self.hs_tempval = QLineEdit(self)
        self.hs_tempval.setReadOnly(True)
        col6_layout.addWidget(self.hs_tempval)
        spacer = QSpacerItem(10, 1)  
        col6_layout.addItem(spacer)
        self.dsp_tempval = QLineEdit(self)
        self.dsp_tempval.setReadOnly(True)
        col6_layout.addWidget(self.dsp_tempval)
        spacer = QSpacerItem(10, 60)  
        col6_layout.addItem(spacer)
        self.vd1val = QLineEdit(self)
        self.vd1val.setReadOnly(True)
        col6_layout.addWidget(self.vd1val)
        self.vq1val = QLineEdit(self)
        self.vq1val.setReadOnly(True)
        col6_layout.addWidget(self.vq1val)
        
        col7_layout = QVBoxLayout()
        mcdata_layout.addLayout(col7_layout)        
        spacer = QSpacerItem(10, 30) 
        col7_layout.addItem(spacer)
        self.other_volt = QLabel()
        self.other_volt.setText("Other Voltages")
        self.other_volt.setStyleSheet("color: white; font-weight: bold;")
        col7_layout.addWidget(self.other_volt)
        self.rail1 = QLabel()
        self.rail1.setText("15V Rail")
        self.rail1.setStyleSheet("color: white;")
        col7_layout.addWidget(self.rail1)
        self.rail2 = QLabel()
        self.rail2.setText("1.9V Rail")
        self.rail2.setStyleSheet("color: white;")
        col7_layout.addWidget(self.rail2)
        self.rail3 = QLabel()
        self.rail3.setText("3.3V Rail")
        self.rail3.setStyleSheet("color: white;")
        col7_layout.addWidget(self.rail3)
        spacer = QSpacerItem(10, 30) 
        col7_layout.addItem(spacer)
        self.motor_curr = QLabel()
        self.motor_curr.setText("Motor Current")
        self.motor_curr.setStyleSheet("color: white; font-weight: bold;")
        col7_layout.addWidget(self.motor_curr)
        self.id = QLabel()
        self.id.setText("Id")
        self.id.setStyleSheet("color: white;")
        col7_layout.addWidget(self.id)
        self.iq = QLabel()
        self.iq.setText("Iq")
        self.iq.setStyleSheet("color: white;")
        col7_layout.addWidget(self.iq)

        col8_layout = QVBoxLayout()
        mcdata_layout.addLayout(col8_layout)
        spacer = QSpacerItem(10, 60)  
        col8_layout.addItem(spacer)
        self.rail1val = QLineEdit(self)
        self.rail1val.setReadOnly(True)
        col8_layout.addWidget(self.rail1val)
        spacer = QSpacerItem(10, 1)  
        col8_layout.addItem(spacer)
        self.rail2val = QLineEdit(self)
        self.rail2val.setReadOnly(True)
        col8_layout.addWidget(self.rail2val)
        spacer = QSpacerItem(10, 1)  
        col8_layout.addItem(spacer)
        self.rail3val = QLineEdit(self)
        self.rail3val.setReadOnly(True)
        col8_layout.addWidget(self.rail3val)
        spacer = QSpacerItem(10, 60)  
        col8_layout.addItem(spacer)
        self.idval = QLineEdit(self)
        self.idval.setReadOnly(True)
        col8_layout.addWidget(self.idval)
        self.iqval = QLineEdit(self)
        self.iqval.setReadOnly(True)
        col8_layout.addWidget(self.iqval)

        #BATTERY Layout
        self.batcol0_layout = QVBoxLayout()
        self.battery_layout.addLayout(self.batcol0_layout)
        self.grid = QGridLayout()
        self.batcol0_layout.addLayout(self.grid) 

        self.soc_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/SOC_def.png',
                           '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/SOC_red.png']
        
        self.sob_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/SOB_def.png',
                           '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/SOB_red.png']
        
        self.loadvolt_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Loadvolt_def.png',
                                '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Loadvolt_red.png']
        
        self.battvolt_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Battvolt_def.png',
                                '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Battvolt_red.png']
        
        self.totcurr_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Totcurr_def.png',
                               '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Totcurr_red.png']
        
        self.nodecount_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/nodecount_def.png',
                                 '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/nodecount_red.png']
        
        self.maxcellvolt_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/maxcellvolt_def.png',
                                   '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/maxcellvolt_red.png']
        
        self.mincellvolt_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/mincellvolt_def.png',
                                   '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/mincellvolt_red.png']
        
        self.maxtemp_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/maxtemp_def.png',
                               '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/maxtemp_red.png']
        
        self.mintemp_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/mintemp_def.png',
                               '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/mintemp_red.png']
        
        self.init_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/init.png',
                            '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/init_gr.png']

        self.precharge_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/precharge.png',
                                 '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/precharge_gr.png']
        
        self.discharge_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/discharge.png',
                                 '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/discharge_gr.png']
        
        self.charge_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/charge.png',
                              '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/charge_gr.png']
        
        self.all_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/all.png',
                           '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/all_gr.png']
        
        self.dischargepre_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/dischargepre.png',
                                    '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/dischargepre_gr.png']
        
        self.chargepre_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/chargepre.png',
                                 '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/chargepre_gr.png']
        
        self.allpre_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/allpre.png',
                              '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/allpre_gr.png']
        
        self.error_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/error.png',
                             '/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/error_gr.png']
        
        self.calibrated_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/calibrated.png']

        self.connected_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/connected.png']

        self.standalone_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/standalone.png']

        self.sense_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/sense.png']

        self.overvolt_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/overvolt.png']

        self.undervolt_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/undervolt.png']

        self.overtemp_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/overtemp.png']

        self.undertemp_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/undertemp.png']

        self.criticaloc_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/criticalOC.png']

        self.criticalov_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/criticalOV.png']

        self.criticaluv_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/criticalUV.png']

        self.balancing_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/balancing.png']

        self.fail_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/fail.png']

        self.invalid_images = ['/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/invalid.png']

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

        self.batinfo = QLabel()
        self.batinfo.setText("Battery Info")
        self.batinfo.setStyleSheet("color: white;")
        self.grid.addWidget(self.batinfo, 0, 0)

        self.soc = QLabel()
        self.grid.addWidget(self.soc, 1,0)
        
        self.sob = QLabel()
        self.grid.addWidget(self.sob, 1,1)
        
        self.loadvolt = QLabel()
        self.grid.addWidget(self.loadvolt, 2,0)
        
        self.battvolt = QLabel()
        self.grid.addWidget(self.battvolt, 2,1)
        
        self.totcurr = QLabel()
        self.grid.addWidget(self.totcurr, 3,0)

        self.nodecount = QLabel()
        self.grid.addWidget(self.nodecount, 3,1)

        self.maxcellvolt = QLabel()
        self.grid.addWidget(self.maxcellvolt, 4,0)

        self.mincellvolt = QLabel()
        self.grid.addWidget(self.mincellvolt, 4,1)

        self.maxtemp = QLabel()
        self.grid.addWidget(self.maxtemp, 5,0)

        self.mintemp = QLabel()
        self.grid.addWidget(self.mintemp, 5,1)

        self.batstate = QLabel()
        self.batstate.setText("Battery State")
        self.batstate.setStyleSheet("color: white;")
        self.grid.addWidget(self.batstate, 6,0)

        self.init = QLabel()
        self.grid.addWidget(self.init, 7,0)

        self.precharge = QLabel()
        self.grid.addWidget(self.precharge, 7,1)

        self.discharge = QLabel()
        self.grid.addWidget(self.discharge, 8,0)
        
        self.charge = QLabel()
        self.grid.addWidget(self.charge, 8,1)
        
        self.all = QLabel()
        self.grid.addWidget(self.all, 9,0)
        
        self.dischargepre = QLabel()
        self.grid.addWidget(self.dischargepre, 9,1)
        
        self.chargepre = QLabel()
        self.grid.addWidget(self.chargepre, 10,0)
        
        self.allpre = QLabel()
        self.grid.addWidget(self.allpre, 10,1)
        
        self.error = QLabel()
        self.grid.addWidget(self.error, 11,0)

        self.batevents = QLabel()
        self.batevents.setText("Battery Events")
        self.batevents.setStyleSheet("color: white;")
        self.grid.addWidget(self.batevents, 12,0)

        self.calibrated = QLabel()
        self.grid.addWidget(self.calibrated, 13,0)

        self.connected = QLabel()
        self.grid.addWidget(self.connected, 13,1)

        self.standalone = QLabel()
        self.grid.addWidget(self.standalone, 14,0)

        self.sense = QLabel()
        self.grid.addWidget(self.sense, 14,1)

        self.overvolt = QLabel()
        self.grid.addWidget(self.overvolt, 15,0)

        self.undervolt = QLabel()
        self.grid.addWidget(self.undervolt, 15,1)

        self.overtemp = QLabel()
        self.grid.addWidget(self.overtemp, 16,0)
        
        self.undertemp = QLabel()
        self.grid.addWidget(self.undertemp, 16,1)

        self.criticaloc = QLabel()
        self.grid.addWidget(self.criticaloc, 17,0)

        self.criticalov = QLabel()
        self.grid.addWidget(self.criticalov, 17,1)

        self.criticaluv = QLabel()
        self.grid.addWidget(self.criticaluv, 18,0)

        self.balancing = QLabel()
        self.grid.addWidget(self.balancing, 18,1)

        self.fail = QLabel()
        self.grid.addWidget(self.fail, 19,0)

        self.invalid = QLabel()
        self.grid.addWidget(self.invalid, 19,1)
       
        
        self.batcol1_layout = QVBoxLayout()
        self.battery_layout.addLayout(self.batcol1_layout)
        self.bmu_label = QLabel()
        self.bmu_label.setText("BMU Telemetry")
        self.bmu_label.setStyleSheet("color: white;")
        self.batcol1_layout.addWidget(self.bmu_label)
        self.bmu_table_widget = QTableWidget()
        self.batcol1_layout.addWidget(self.bmu_table_widget)
        self.bmu_table_widget.setRowCount(4)
        self.bmu_table_widget.setColumnCount(9)
        row_label = ['Min mV','Max mV','Min °C','Max °C','Pack mV','Pack mA','Balance +','Balance -','CMU Count']
        column_label = ['Sys Status','','Prechg Status','']
        self.bmu_table_widget.setVerticalHeaderLabels(column_label) 
        self.bmu_table_widget.setHorizontalHeaderLabels(row_label) 
        self.bmu_table_widget.setItem(1, 6, QTableWidgetItem('Fan Speed (rpm)'))
        self.bmu_table_widget.setItem(1, 7, QTableWidgetItem('SOC/BAL (Ah)'))
        self.bmu_table_widget.setItem(1, 8, QTableWidgetItem('SOC/BAL (%)'))
        


        spacer = QSpacerItem(1, 130)  
        self.batcol1_layout.addItem(spacer)

        self.grid1 = QGridLayout()
        self.batcol1_layout.addLayout(self.grid1) 

        self.nocan = QLabel()
        self.grid1.addWidget(self.nocan, 0,0)

        self.vehiclecomms = QLabel()
        self.grid1.addWidget(self.vehiclecomms, 0,1)

        self.cmucomms = QLabel()
        self.grid1.addWidget(self.cmucomms, 0,2)

        self.prechargeerror = QLabel()
        self.grid1.addWidget(self.prechargeerror, 1,0)

        self.prechargeidle = QLabel()
        self.grid1.addWidget(self.prechargeidle, 1,1)

        self.onandr = QLabel()
        self.grid1.addWidget(self.onandr, 1,2)

        self.cellov = QLabel()
        self.grid1.addWidget(self.cellov, 2,0)

        self.celluv = QLabel()
        self.grid1.addWidget(self.celluv, 2,1)

        self.cellot = QLabel()
        self.grid1.addWidget(self.cellot, 2,2)

        self.trusterror = QLabel()
        self.grid1.addWidget(self.trusterror, 3,0)

        self.packisofail = QLabel()
        self.grid1.addWidget(self.packisofail, 3,1)

        self.contactorstuck = QLabel()
        self.grid1.addWidget(self.contactorstuck, 3,2)

        self.contactorclosed = QLabel()
        self.grid1.addWidget(self.contactorclosed, 4,0)

        self.can12vlow = QLabel()
        self.grid1.addWidget(self.can12vlow, 4,1)

        self.cmupwrstat = QLabel()
        self.grid1.addWidget(self.cmupwrstat, 4,2)

        self.extracell = QLabel()
        self.grid1.addWidget(self.extracell, 5,0)

        self.socinvalid = QLabel()
        self.grid1.addWidget(self.socinvalid, 5,1)

        self.bmusetup = QLabel()
        self.grid1.addWidget(self.bmusetup, 5,2)

        spacer = QSpacerItem(1, 120)  
        self.batcol1_layout.addItem(spacer)

        self.cmu_label = QLabel()
        self.cmu_label.setText("CMU Telemetry")
        self.cmu_label.setStyleSheet("color: white;")
        self.batcol1_layout.addWidget(self.cmu_label)

        self.bmu_table_widget1 = QTableWidget()
        self.batcol1_layout.addWidget(self.bmu_table_widget1)
        row_label1 = ['Serial','PCB °C','Cell °C','Cell 0 mV','Cell 1 mV','Cell 2 mV','Cell 3 mV','Cell 4 mV','Cell 5 mV','Cell 6 mV','Cell 7 mV']
        column_label1 = ['CMU 1', 'CMU 2', 'CMU 3', 'CMU 4']
        self.bmu_table_widget1.setRowCount(4)
        self.bmu_table_widget1.setColumnCount(11)
        self.bmu_table_widget1.setHorizontalHeaderLabels(row_label1)
        self.bmu_table_widget1.setVerticalHeaderLabels(column_label1) 


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

        self.mppt4_input_data['y'].append(self.data[389])
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
        self.bmu_table_widget.setItem(0, 0, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(0, 1, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(0, 2, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(0, 3, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(0, 4, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(0, 5, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(0, 6, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(0, 7, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(0, 8, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(2, 0, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(2, 6, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(2, 7, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(2, 8, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(3, 6, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(3, 7, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget.setItem(3, 8, QTableWidgetItem(str(self.pw_out)))
        
        #BATTERY CMU Telemetry
        self.bmu_table_widget1.setItem(0, 0, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(0, 1, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(0, 2, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(0, 3, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(0, 4, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(0, 5, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(0, 6, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(0, 7, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(0, 8, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(0, 9, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(0, 10, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(1, 0, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(1, 1, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(1, 2, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(1, 3, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(1, 4, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(1, 5, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(1, 6, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(1, 7, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(1, 8, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(1, 9, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(1, 10, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(2, 0, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(2, 1, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(2, 2, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(2, 3, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(2, 4, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(2, 5, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(2, 6, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(2, 7, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(2, 8, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(2, 9, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(2, 10, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(3, 0, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(3, 1, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(3, 2, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(3, 3, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(3, 4, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(3, 5, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(3, 6, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(3, 7, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(3, 8, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(3, 9, QTableWidgetItem(str(self.pw_out)))
        self.bmu_table_widget1.setItem(3, 10, QTableWidgetItem(str(self.pw_out)))

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
        
        pixmap_soc = QPixmap(self.soc_images[soc_index])
        painter_soc = QPainter(pixmap_soc)
        painter_soc.setFont(QFont('Bookman Uralic', 20))  
        painter_soc.setPen(Qt.white)  
        text_width_temp = painter_soc.fontMetrics().width(str(self.temp))
        x_temp = (pixmap_soc.width() - text_width_temp) // 2
        painter_soc.drawText(x_temp, 45, str(self.temp))
        painter_soc.end()
        self.soc.setPixmap(pixmap_soc.scaled(265, 40)) #.scaled(71, 157)

        pixmap_sob = QPixmap(self.sob_images[sob_index])
        painter_sob = QPainter(pixmap_sob)
        painter_sob.setFont(QFont('Bookman Uralic', 20))  
        painter_sob.setPen(Qt.white)  
        text_width_temp = painter_sob.fontMetrics().width(str(self.temp))
        x_temp = (pixmap_sob.width() - text_width_temp) // 2
        painter_sob.drawText(x_temp, 45, str(self.temp))
        painter_sob.end()
        self.sob.setPixmap(pixmap_sob.scaled(265, 40)) 

        pixmap_loadvolt = QPixmap(self.loadvolt_images[loadvolt_index])
        painter_loadvolt = QPainter(pixmap_loadvolt)
        painter_loadvolt.setFont(QFont('Bookman Uralic', 20))  
        painter_loadvolt.setPen(Qt.white)  
        text_width_temp = painter_loadvolt.fontMetrics().width(str(self.temp))
        x_temp = (pixmap_loadvolt.width() - text_width_temp) // 2
        painter_loadvolt.drawText(x_temp, 45, str(self.temp))
        painter_loadvolt.end()
        self.loadvolt.setPixmap(pixmap_loadvolt.scaled(265, 40))

        pixmap_battvolt = QPixmap(self.battvolt_images[battvolt_index])
        painter_battvolt = QPainter(pixmap_battvolt)
        painter_battvolt.setFont(QFont('Bookman Uralic', 20))  
        painter_battvolt.setPen(Qt.white)  
        text_width_temp = painter_battvolt.fontMetrics().width(str(self.temp))
        x_temp = (pixmap_battvolt.width() - text_width_temp) // 2
        painter_battvolt.drawText(x_temp, 45, str(self.temp))
        painter_battvolt.end()
        self.battvolt.setPixmap(pixmap_battvolt.scaled(265, 40)) 

        pixmap_totcurr = QPixmap(self.totcurr_images[totcurr_index])
        painter_totcurr = QPainter(pixmap_totcurr)
        painter_totcurr.setFont(QFont('Bookman Uralic', 20))  
        painter_totcurr.setPen(Qt.white)  
        text_width_temp = painter_totcurr.fontMetrics().width(str(self.temp))
        x_temp = (pixmap_totcurr.width() - text_width_temp) // 2
        painter_totcurr.drawText(x_temp, 45, str(self.temp))
        painter_totcurr.end()
        self.totcurr.setPixmap(pixmap_totcurr.scaled(265, 40)) 

        pixmap_node = QPixmap(self.nodecount_images[node_index])
        painter_node = QPainter(pixmap_node)
        painter_node.setFont(QFont('Bookman Uralic', 20))  
        painter_node.setPen(Qt.white)  
        text_width_temp = painter_node.fontMetrics().width(str(self.temp))
        x_temp = (pixmap_node.width() - text_width_temp) // 2
        painter_node.drawText(x_temp, 45, str(self.temp))
        painter_node.end()
        self.nodecount.setPixmap(pixmap_node.scaled(265, 40)) 

        pixmap_maxcell = QPixmap(self.maxcellvolt_images[maxcell_index])
        painter_maxcell = QPainter(pixmap_maxcell)
        painter_maxcell.setFont(QFont('Bookman Uralic', 20))  
        painter_maxcell.setPen(Qt.white)  
        text_width_temp = painter_maxcell.fontMetrics().width(str(self.temp))
        x_temp = (pixmap_maxcell.width() - text_width_temp) // 2
        painter_maxcell.drawText(x_temp, 45, str(self.temp))
        painter_maxcell.end()
        self.maxcellvolt.setPixmap(pixmap_maxcell.scaled(265, 40)) 
                
        pixmap_mincell = QPixmap(self.mincellvolt_images[mincell_index])
        painter_mincell = QPainter(pixmap_mincell)
        painter_mincell.setFont(QFont('Bookman Uralic', 20))  
        painter_mincell.setPen(Qt.white)  
        text_width_temp = painter_mincell.fontMetrics().width(str(self.temp))
        x_temp = (pixmap_mincell.width() - text_width_temp) // 2
        painter_mincell.drawText(x_temp, 45, str(self.temp))
        painter_mincell.end()
        self.mincellvolt.setPixmap(pixmap_mincell.scaled(265, 40)) 

        pixmap_maxtemp = QPixmap(self.maxtemp_images[maxtemp_index])
        painter_maxtemp = QPainter(pixmap_maxtemp)
        painter_maxtemp.setFont(QFont('Bookman Uralic', 20))  
        painter_maxtemp.setPen(Qt.white)  
        text_width_temp = painter_maxtemp.fontMetrics().width(str(self.temp))
        x_temp = (pixmap_maxtemp.width() - text_width_temp) // 2
        painter_maxtemp.drawText(x_temp, 45, str(self.temp))
        painter_maxtemp.end()
        self.maxtemp.setPixmap(pixmap_maxtemp.scaled(265, 40)) 

        pixmap_mintemp = QPixmap(self.mintemp_images[mintemp_index])
        painter_mintemp = QPainter(pixmap_mintemp)
        painter_mintemp.setFont(QFont('Bookman Uralic', 20))  
        painter_mintemp.setPen(Qt.white)  
        text_width_temp = painter_mintemp.fontMetrics().width(str(self.temp))
        x_temp = (pixmap_mintemp.width() - text_width_temp) // 2
        painter_mintemp.drawText(x_temp, 45, str(self.temp))
        painter_mintemp.end()
        self.mintemp.setPixmap(pixmap_mintemp.scaled(265, 40)) 

        pixmap_init = QPixmap(self.init_images[init_index])
        self.init.setPixmap(pixmap_init.scaled(265, 40))

        pixmap_precharge = QPixmap(self.precharge_images[precharge_index])
        self.precharge.setPixmap(pixmap_precharge.scaled(265, 40))

        pixmap_discharge = QPixmap(self.discharge_images[discharge_index])
        self.discharge.setPixmap(pixmap_discharge.scaled(265, 40))

        pixmap_charge = QPixmap(self.charge_images[charge_index])
        self.charge.setPixmap(pixmap_charge.scaled(265, 40))

        pixmap_all = QPixmap(self.all_images[all_index])
        self.all.setPixmap(pixmap_all.scaled(265, 40))

        pixmap_dischargepre = QPixmap(self.dischargepre_images[dischargepre_index])
        self.dischargepre.setPixmap(pixmap_dischargepre.scaled(265, 40))

        pixmap_chargepre = QPixmap(self.chargepre_images[chargepre_index])
        self.chargepre.setPixmap(pixmap_chargepre.scaled(265, 40))

        pixmap_allpre = QPixmap(self.allpre_images[allpre_index])
        self.allpre.setPixmap(pixmap_allpre.scaled(265, 40))

        pixmap_error = QPixmap(self.error_images[error_index])
        self.error.setPixmap(pixmap_error.scaled(265, 40))

        pixmap_calibrated = QPixmap(self.calibrated_images[0])
        self.calibrated.setPixmap(pixmap_calibrated.scaled(265, 40))

        pixmap_connected = QPixmap(self.connected_images[0])
        self.connected.setPixmap(pixmap_connected.scaled(265, 40))

        pixmap_standalone = QPixmap(self.standalone_images[0])
        self.standalone.setPixmap(pixmap_standalone.scaled(265, 40))

        pixmap_sense = QPixmap(self.sense_images[0])
        self.sense.setPixmap(pixmap_sense.scaled(265, 40))

        pixmap_overvolt = QPixmap(self.overvolt_images[0])
        self.overvolt.setPixmap(pixmap_overvolt.scaled(265, 40))

        pixmap_undervolt = QPixmap(self.undervolt_images[0])
        self.undervolt.setPixmap(pixmap_undervolt.scaled(265, 40))

        pixmap_overtemp = QPixmap(self.overtemp_images[0])
        self.overtemp.setPixmap(pixmap_overtemp.scaled(265, 40))

        pixmap_undertemp = QPixmap(self.undertemp_images[0])
        self.undertemp.setPixmap(pixmap_undertemp.scaled(265, 40))

        pixmap_criticaloc = QPixmap(self.criticaloc_images[0])
        self.criticaloc.setPixmap(pixmap_criticaloc.scaled(265, 40))

        pixmap_criticalov = QPixmap(self.criticalov_images[0])
        self.criticalov.setPixmap(pixmap_criticalov.scaled(265, 40))

        pixmap_criticaluv = QPixmap(self.criticaluv_images[0])
        self.criticaluv.setPixmap(pixmap_criticaluv.scaled(265, 40))

        pixmap_balancing = QPixmap(self.balancing_images[0])
        self.balancing.setPixmap(pixmap_balancing.scaled(265, 40))

        pixmap_fail = QPixmap(self.fail_images[0])
        self.fail.setPixmap(pixmap_fail.scaled(265, 40))

        pixmap_invalid = QPixmap(self.invalid_images[0])
        self.invalid.setPixmap(pixmap_invalid.scaled(265, 40))

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
        self.telemetry_widget.setPixmap(pixmap_telemetry.scaled(320, 320))
        #Drive Widget
        pixmap_drive = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Drive_widget.png")
        painter_drive = QPainter(pixmap_drive)
        painter_drive.setFont(QFont('Bookman Uralic', 25))  
        painter_drive.setPen(Qt.white)  
        painter_drive.drawText(35, 150, "PW OUT:" + str(self.pw_out)+ "V")
        painter_drive.drawText(35, 265, "TEMP:" + str(self.temp)+ "°C")
        painter_drive.end()
        self.drive_widget.setPixmap(pixmap_drive.scaled(320, 320))
        #Strategy Widget
        pixmap_strategy = QPixmap("/home/jaay/Agnirath/Agnirath_LVS_Strategy/aarush_ros/src/daq_package/daq_package/assets/Startegy_widget.png")
        painter_strategy = QPainter(pixmap_strategy)
        painter_strategy.setFont(QFont('Bookman Uralic', 25)) 
        painter_strategy.setPen(Qt.white)  
        painter_strategy.drawText(55, 150, "PW OUT:" + str(self.pw_out)+ "V")
        painter_strategy.drawText(55, 265, "TEMP:" + str(self.temp)+ "°C")
        painter_strategy.end()
        self.strategy_widget.setPixmap(pixmap_strategy.scaled(320, 320))
    
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

def main():
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())

        
if __name__ == "__main__":
    main()
    

