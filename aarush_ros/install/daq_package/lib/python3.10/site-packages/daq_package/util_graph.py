# import sys
# import numpy as np
# from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
# from PyQt5.QtCore import Qt, QTimer
# import pyqtgraph as pg

# class LiveDataPlot(QMainWindow):
#     def __init__(self):
#         super().__init__()

#         self.initUI()

#     def initUI(self):
#         self.setWindowTitle("Live Data Plot with PyQtGraph")
#         self.setGeometry(100, 100, 800, 600)

#         # Create a central widget and layout
#         central_widget = QWidget(self)
#         layout = QVBoxLayout()
#         central_widget.setLayout(layout)
#         self.setCentralWidget(central_widget)

#         # Create a PlotWidget from PyQtGraph
#         self.plot_widget = pg.PlotWidget()
#         layout.addWidget(self.plot_widget)

#         # Initialize the x-axis and y-axis data
#         # self.x_data = []
#         self.y_data = []

#         axis = pg.DateAxisItem()
#         self.plot_widget.setAxisItems({'bottom':axis})

#         # Create a QTimer to update live data and plot it
#         self.timer = QTimer(self)
#         self.timer.timeout.connect(self.updateLivePlot)
#         self.timer.start(1000)  # Update data every 1 second

#     def updateLivePlot(self):
#         # Generate random data
#         # new_x = len(self.x_data)
#         new_y = np.random.rand()

#         # Append the new data point
#         # self.x_data.append(new_x)
#         self.y_data.append(new_y)

#         # Update the plot
#         self.plot_widget.plot(self.y_data, pen='b', clear=True)

# def main():
#     app = QApplication(sys.argv)
#     window = LiveDataPlot()
#     window.show()
#     sys.exit(app.exec_())

# if __name__ == "__main__":
#     main()

import pyqtgraph as pg



