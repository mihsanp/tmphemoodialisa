from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPixmap, QFont, QIcon
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QComboBox, QHBoxLayout, QTabWidget, QProgressBar, QLabel, QLineEdit
from PyQt5.QtSerialPort import QSerialPortInfo
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import csv
import struct
import sys
import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph import PlotWidget


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pressure Monitor")
        self.serial_port = None
        self.setWindowIcon(QIcon("D:/Ihsan/pressure_hd/GUI/bme.png"))

        # Initialize empty lists to store the data
        self.DialInPressure_data = []
        self.DialOutPressure_data = []
        self.ArteriPressure_data = []
        self.VenaPressure_data = []
        self.actual_data = []
        self.flow_data = []
        self.tmp_data = []
        self.blood_pump_status = '0'
        self.actuator_status = '0'
        self.pump_status = '0'
        self.tmp_set = '300'
        self.flow_set = '500'

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_canvasdi_plot)
        self.timer.start(100)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_canvasdo_plot)
        self.timer.start(100)
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_canvasa_plot)
        self.timer.start(100)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_canvasv_plot)
        self.timer.start(100)

        self.actual_timer = QtCore.QTimer()
        self.actual_timer.timeout.connect(self.update_actual_plot)
        self.actual_timer.start(100)

        # Create the main widget and layout
        main_widget = QWidget(self)
        main_widget.setStyleSheet("background-color: #131624;")  # Set dark background color
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        # Create the header widget and layout
        header_widget = QWidget()
        header_layout = QHBoxLayout(header_widget)
        header_layout.setAlignment(Qt.AlignTop | Qt.AlignRight)  # Align to the top and right

        # Create the header label
        header_label = QLabel("Desain Mekanisme Pengaturan Tekanan Transmembran dalam Proses Ultrafiltrasi pada Mesin Hemodialisis")
        header_label.setStyleSheet("font-size: 20pt; font-weight: bold; color: white;")
        header_layout.addWidget(header_label)

        # Create a spacer item to push the images to the right side
        header_layout.addStretch(1)

        # Create the image widget for the first image
        image1 = QLabel()
        pixmap1 = QPixmap("D:/Ihsan/pressure_hd/GUI/bme.png")
        image1.setPixmap(pixmap1.scaledToHeight(100))  # Adjust the height as desired
        header_layout.addWidget(image1)

        # Create the image widget for the second image
        image2 = QLabel()
        pixmap2 = QPixmap("D:/Ihsan/pressure_hd/GUI/its.png")
        image2.setPixmap(pixmap2.scaledToHeight(100))  # Adjust the height as desired
        header_layout.addWidget(image2)

        main_layout.addWidget(header_widget)

        # Create the name widget and layout
        name_widget = QWidget()
        name_layout = QHBoxLayout(name_widget)

        # Create the name label
        name_label = QLabel("Muhammad Ihsan Pamungkas (07311940000041)")
        name_label.setStyleSheet("font-size: 15pt; font-weight: bold; color: white;")
        name_layout.addWidget(name_label)

        main_layout.addWidget(name_widget)
        # Create the tab widget
        self.tab_widget = QTabWidget()

        # Set the font size of the tab labels
        font = QFont()
        font.setPointSize(10)
        self.tab_widget.setFont(font)

        # Set the background color of the tabs
        self.tab_widget.setStyleSheet("QTabWidget::tab { background-color: #5465FF; }")

        # Add the tab widget to the main layout
        main_layout.addWidget(self.tab_widget)

        # Create the pressure tab
        pressure_tab = QWidget()
        self.tab_widget.addTab(pressure_tab, "Pressure")

        # ... Rest of the code remains the same ...


        # Create the pressure tab layout
        tab_layout = QHBoxLayout(pressure_tab)

        # Create the button widget and layout for pressure tab
        button_widget = QWidget()
        button_layout = QVBoxLayout(button_widget)
        button_layout.setAlignment(Qt.AlignTop)
        pressure_widget = QWidget()
        pressure_layout = QVBoxLayout(pressure_widget)
        pressure_layout.setAlignment(Qt.AlignTop)
        tab_layout.addWidget(button_widget)
        tab_layout.addWidget(pressure_widget)

        # Create the buttons for pressure tab
        self.button = QPushButton('Turn On Pump')
        self.button.setStyleSheet("background-color:#5465FF; border-radius:28px; border:1px solid #18ab29; display:inline-block; cursor:pointer; color:#ffffff; font-family:Arial; font-size:17px; padding:16px 31px; text-decoration:none; text-shadow:0px 1px 0px #2f6627;")
        self.button.clicked.connect(self.button_clicked)
        button_layout.addWidget(self.button)

        self.button_naik = QPushButton('Pressure Control')
        self.button_naik.setStyleSheet("background-color:#5465FF; border-radius:28px; border:1px solid #18ab29; display:inline-block; cursor:pointer; color:#ffffff; font-family:Arial; font-size:17px; padding:16px 31px; text-decoration:none; text-shadow:0px 1px 0px #2f6627;")
        self.button_naik.clicked.connect(self.button_naik_clicked)
        button_layout.addWidget(self.button_naik)

        # Create the record button for pressure tab
        self.record_button = QPushButton('Record Data')
        self.record_button.setStyleSheet("background-color:#5465FF; border-radius:28px; border:1px solid #18ab29; display:inline-block; cursor:pointer; color:#ffffff; font-family:Arial; font-size:17px; padding:16px 31px; text-decoration:none; text-shadow:0px 1px 0px #2f6627;")        
        self.record_button.clicked.connect(self.record_button_clicked)
        button_layout.addWidget(self.record_button)

        self.port_label = QLabel("Select Port: ")
        self.port_label.setStyleSheet("font-size: 18px; color : white;")
        button_layout.addWidget(self.port_label)

        # Create the serial port combo box for pressure tab
        self.port_combo_box = QComboBox()
        self.port_combo_box.setStyleSheet("background-color: #FFFFFF")  
        button_layout.addWidget(self.port_combo_box)

        # Create a label for displaying text
        self.flow_label = QLabel("Set Flow (ml/mim): ")
        self.flow_label.setStyleSheet("font-size: 18px; color : white;")
        button_layout.addWidget(self.flow_label)

        # Create the text input field for pressure tab
        self.flow_input = QLineEdit()
        self.flow_input.setStyleSheet("background-color: #FFFFFF")
        button_layout.addWidget(self.flow_input)

        # Create the send button for pressure tab
        self.flow_button = QPushButton('SET FLOW')
        self.flow_button.setStyleSheet("background-color:#5465FF; border-radius:28px; border:1px solid #18ab29; display:inline-block; cursor:pointer; color:#ffffff; font-family:Arial; font-size:17px; padding:16px 31px; text-decoration:none; text-shadow:0px 1px 0px #2f6627;")        
        self.flow_button.clicked.connect(self.flow_button_clicked)
        button_layout.addWidget(self.flow_button)

        self.tmp_label = QLabel("Set TMP (mmHg): ")
        self.tmp_label.setStyleSheet("font-size: 18px; color : white;")
        button_layout.addWidget(self.tmp_label)
        # Create the text input field for pressure tab
        self.tmp_input = QLineEdit()
        self.tmp_input.setStyleSheet("background-color: #FFFFFF")
        button_layout.addWidget(self.tmp_input)
                # Create the send button for pressure tab
        self.tmp_button = QPushButton('SET TMP')
        self.tmp_button.setStyleSheet("background-color:#5465FF; border-radius:28px; border:1px solid #18ab29; display:inline-block; cursor:pointer; color:#ffffff; font-family:Arial; font-size:17px; padding:16px 31px; text-decoration:none; text-shadow:0px 1px 0px #2f6627;")        
        self.tmp_button.clicked.connect(self.tmp_button_clicked)
        button_layout.addWidget(self.tmp_button)




        # Create the matplotlib figure and canvas for pressure tab
        self.canvasdi = pg.PlotWidget()
        pressure_layout.addWidget(self.canvasdi)
        self.canvasdi_plot_item= self.canvasdi.plot(pen='g')
        self.canvasdi.setYRange(-500.0, 500.0)
                # Create the matplotlib figure and canvas for pressure tab
        self.canvasdo = pg.PlotWidget()
        pressure_layout.addWidget(self.canvasdo)
        self.canvasdo_plot_item= self.canvasdo.plot(pen='y')
        self.canvasdo.setYRange(-500.0, 500.0)
                # Create the matplotlib figure and canvas for pressure tab
        self.canvasa = pg.PlotWidget()
        pressure_layout.addWidget(self.canvasa)
        self.canvasa_plot_item= self.canvasa.plot(pen='r')
        self.canvasa.setYRange(-500.0, 500.0)
                # Create the matplotlib figure and canvas for pressure tab
        self.canvasv = pg.PlotWidget()
        pressure_layout.addWidget(self.canvasv)
        self.canvasv_plot_item= self.canvasv.plot(pen='b')
        self.canvasv.setYRange(-500.0, 500.0)

        self.canvastmp = pg.PlotWidget()
        pressure_layout.addWidget(self.canvastmp)
        self.canvastmp_plot_item= self.canvastmp.plot(pen='r')
        self.canvastmp.setYRange(-500.0, 500.0)       


        # self.canvasdo = pg.PlotWidget()
        # pressure_layout.addWidget(self.canvasdo)
        # self.canvasdo_plot_item= self.canvasdo.plot(pen='r')




        # Create the actual tab
        actual_tab = QWidget()
        self.tab_widget.addTab(actual_tab, "Time Response")

        # Create the actual tab layout
        actual_layout = QHBoxLayout(actual_tab)

        # Create the record button for actual tab
        self.record_actual_button = QPushButton('Record Time Response Data')
        self.record_actual_button.setStyleSheet("background-color:#5465FF; border-radius:28px; border:1px solid #18ab29; display:inline-block; cursor:pointer; color:#ffffff; font-family:Arial; font-size:17px; padding:16px 31px; text-decoration:none; text-shadow:0px 1px 0px #2f6627;")
        self.record_actual_button.clicked.connect(self.record_actual_button_clicked)
        actual_layout.addWidget(self.record_actual_button)

        # Create actuator button
        self.actuator_button = QPushButton('Turn On Actuator')
        self.actuator_button.setStyleSheet("background-color:#5465FF; border-radius:28px; border:1px solid #18ab29; display:inline-block; cursor:pointer; color:#ffffff; font-family:Arial; font-size:17px; padding:16px 31px; text-decoration:none; text-shadow:0px 1px 0px #2f6627;")
        self.actuator_button.clicked.connect(self.actuator_button_clicked)
        actual_layout.addWidget(self.actuator_button)

        # Create the matplotlib figure and canvas for actual tab
        self.actual_canvas = pg.PlotWidget()
        actual_layout.addWidget(self.actual_canvas)

        # Set up the plot for actual tab
        self.actual_plot_item = self.actual_canvas.plot(pen='w')
        self.actual_plot_item2 = self.actual_canvas.plot(pen='r')
        # self.actual_ax.legend()

        # Create a background for blitting for actual tab

        # Start the update timer
        self.timer = self.startTimer(1)  # Update every 16 milliseconds (approx. 60 FPS)

        # Create a flag to indicate recording state for pressure tab
        self.is_recording = False

        # Create a flag to indicate recording state for actual tab
        self.is_recording_actual = False

        # Create a filename for the CSV file for pressure tab
        self.filename = 'pressure_data.csv'

        # Create a filename for the CSV file for actual tab
        self.actual_filename = 'flowdata.csv'

        # Populate the serial ports
        self.populate_serial_ports()

    def populate_serial_ports(self):
        self.port_combo_box.clear()

        ports = QSerialPortInfo.availablePorts()
        for port in ports:
            self.port_combo_box.addItem(port.portName())

        if self.port_combo_box.count() > 0:
            self.serial_port = serial.Serial()
            self.serial_port.port = self.port_combo_box.currentText()
            self.serial_port.baudrate = 115200
            self.serial_port.timeout = 1
            self.serial_port.open()
            self.serial_port.flush()

    # def update_plots(self):
    #     for i, pressure in enumerate([self.DialInPressure_data, self.DialOutPressure_data, self.ArteriPressure_data, self.VenaPressure_data]):
    #         if pressure:
    #             x = np.arange(len(pressure)) * 0.2
    #             self.plot_items[i].setData(x, pressure)

    #     # Update the values of the progress bars based on the pressure data
    #     for i, pressure in enumerate([self.DialInPressure_data, self.DialOutPressure_data,
    #                                 self.ArteriPressure_data, self.VenaPressure_data]):
    #         if pressure:
    #             value = int(pressure[-1])
    #             self.progress_bars[i].setValue(value)

    def update_actual_plot(self):
        x = np.arange(len(self.flow_data)) * 0.2
        self.actual_plot_item.setData(x, self.actual_data)
        self.actual_plot_item2.setData(x, self.flow_data)

    def update_canvasdi_plot(self):
        x = np.arange(len(self.DialInPressure_data)) * 0.2
        self.canvasdi_plot_item.setData(x, self.DialInPressure_data)
    def update_canvasdo_plot(self):
        x = np.arange(len(self.DialOutPressure_data)) * 0.2
        self.canvasdo_plot_item.setData(x, self.DialOutPressure_data)
    def update_canvasa_plot(self):
        x = np.arange(len(self.ArteriPressure_data)) * 0.2
        self.canvasa_plot_item.setData(x, self.ArteriPressure_data)
    def update_canvasv_plot(self):
        x = np.arange(len(self.VenaPressure_data)) * 0.2
        self.canvasv_plot_item.setData(x, self.VenaPressure_data)
    def update_canvastmp_plot(self):
        x = np.arange(len(self.tmp_data)) * 0.2
        self.canvastmp_plot_item.setData(x, self.tmp_data)

    def button_clicked(self):
        blood_pump_status = '1' if self.button.text() == 'Turn On Pump' else '0'
        if blood_pump_status == '1':
            self.button.setText('Turn Off Pump')
        else:
            self.button.setText('Turn On Pump')
        if self.serial_port:
            self.serial_port.write(blood_pump_status.encode())
    def button_naik_clicked(self):
        calibration = 'n' if self.button_naik.text() == 'calibration' else 's'
        if calibration == 'n':
            self.button_naik.setText('selesai')
        else:
            self.button_naik.setText('calibration')
        if self.serial_port:
            self.serial_port.write(calibration.encode())

    def actuator_button_clicked(self):
        actuator_status = '2' if self.actuator_button.text() == 'Turn On Actuator' else '3'
        if actuator_status == '2':
            self.actuator_button.setText('Turn Off Actuator')
        else:
            self.actuator_button.setText('Turn On Actuator')
        if self.serial_port:
            self.serial_port.write(actuator_status.encode())   

        # Send the blood pump status to Teensy

    def record_button_clicked(self):
        self.is_recording = not self.is_recording

        if self.is_recording:
            self.record_button.setText('Stop Recording')
            self.start_recording()
        else:
            self.record_button.setText('Record Data')
            self.stop_recording()
    def flow_button_clicked(self):
        flowset = self.flow_input.text()
        flow_set = 'a' + flowset
        if self.serial_port:
            self.serial_port.write(flow_set.encode())
        print(flow_set)
        # Use the text variable to send the string to Teensy
        # Replace the following line with your actual code to send the text
    def angle_button_clicked(self):
        tmp_set = 'p'
        if self.serial_port:
            self.serial_port.write(tmp_set.encode())       
    def tmp_button_clicked(self):
        tmpset = self.tmp_input.text()
        tmp_set = 'b' + tmpset
        if self.serial_port:
            self.serial_port.write(tmp_set.encode())
        print(tmp_set)
        # Use the text variable to send the string to Teensy
        # Replace the following line with your actual code to send the text

    def record_actual_button_clicked(self):
        self.is_recording_actual = not self.is_recording_actual

        if self.is_recording_actual:
            self.record_actual_button.setText('Stop Recording')
            self.start_recording_actual()
        else:
            self.record_actual_button.setText('Record Data')
            self.stop_recording_actual()

    def start_recording(self):
        self.recorded_data = []

    def stop_recording(self):
        if len(self.recorded_data) > 0:
            self.save_data_to_csv()

    def start_recording_actual(self):
        self.recorded_actual_data = []

    def stop_recording_actual(self):
        if len(self.recorded_actual_data) > 0:
            self.save_actual_data_to_csv()

    def save_data_to_csv(self):
        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for data_point in self.recorded_data:
                writer.writerow(data_point)

    def save_actual_data_to_csv(self):
        with open(self.actual_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for data_point in self.recorded_actual_data:
                writer.writerow(data_point)

    def timerEvent(self, event):
        if self.serial_port and self.serial_port.in_waiting >= 28:
            data = self.serial_port.read(28)  # Increase the read size to accommodate the additional float
            DialInPressure, DialOutPressure, ArteriPressure, VenaPressure, actual, flow, tmp = struct.unpack('fffffff', data)

            # Append the new data to the respective lists for pressure tab
            self.DialInPressure_data.append(DialInPressure)
            self.DialOutPressure_data.append(DialOutPressure)
            self.ArteriPressure_data.append(ArteriPressure)
            self.VenaPressure_data.append(VenaPressure)
            self.actual_data.append(actual)
            self.flow_data.append(flow)
            self.tmp_data.append(tmp)


            # # Update the plots for pressure tab
            # self.update_plots()
            self.update_canvasdi_plot()
            self.update_canvasdo_plot()
            self.update_canvasa_plot()
            self.update_canvasv_plot()   
            self.update_canvastmp_plot()                     
            # Update the plot for actual tab
            self.update_actual_plot()



            if self.is_recording:
                self.recorded_data.append([DialInPressure, DialOutPressure, ArteriPressure, VenaPressure, tmp])

            if self.is_recording_actual:
                self.recorded_actual_data.append([actual, flow])
        


    def closeEvent(self, event):
        if self.serial_port:
            self.serial_port.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec_())

