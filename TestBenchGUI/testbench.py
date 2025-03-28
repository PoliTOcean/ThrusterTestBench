import sys
import json
import numpy as np
import struct
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QComboBox, QFileDialog, QScrollArea, QInputDialog, 
    QMessageBox, QMenuBar, QAction, QMenu, 
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QKeySequence, QIcon
from scipy.interpolate import interp1d, BarycentricInterpolator
import serial
import serial.tools.list_ports
import time

PWM_MIN, PWM_MAX = 1100, 1900
THRUSTER_COUNT = 8
DEFAULT_FREQUENCY = 20  # Hz
DEFAULT_BAUD = 115200

def main():
    app = QApplication(sys.argv)
    window = ThrusterGUI()
    app.setWindowIcon(QIcon("./icon_bgless.png"))

    # set stylesheet
    # file = QFile(":/dark/stylesheet.qss")
    # file.open(QFile.ReadOnly | QFile.Text)
    # stream = QTextStream(file)
    # app.setStyleSheet(stream.readAll())

    window.show()
    sys.exit(app.exec())

class ThrusterGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROV Thruster PWM Controller")
        self.setGeometry(100, 100, 1200, 800)

        self.interpolation_methods = {
            "Linear": "linear",
            "Constant": "constant",
            "Polynomial": "polynomial"
        }
        self.selected_interpolation = "linear"

        self.baud_rate = DEFAULT_BAUD
        self.serial_port = None
        self.output_frequency = DEFAULT_FREQUENCY

        self.cached_pwms = [([], [])] * 8
        self.serial_timer = QTimer(self)
        self.serial_timer.timeout.connect(self.send_pwms)
        self.serial_conn = None
        self.serial_status = 0

        self.points = [[] for _ in range(THRUSTER_COUNT)]
        self.setup_ui()

    def setup_ui(self):
        """Set up the UI with graphs and controls."""
        central_widget = QWidget()
        main_layout = QVBoxLayout()
        control_layout = QHBoxLayout()
        menu_layout = QHBoxLayout()

        menubar = QMenuBar(self)

        self.file_menu = menubar.addMenu("File")
        self.serial_menu = menubar.addMenu("Serial")

        # Scrollable area for graphs
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        graph_container = QWidget()
        graph_layout = QVBoxLayout()

        # Graphs
        self.graphs = []
        for i in range(THRUSTER_COUNT):
            plot = pg.PlotWidget(title=f"Thruster {i+1}")
            plot.setBackground("w")
            plot.showGrid(x=True, y=True)
            plot.setLabel("left", "PWM Value")
            plot.setLabel("bottom", "Time (s)")

            # y axis
            plot.setYRange(PWM_MIN, PWM_MAX, padding=1)
            plot.disableAutoRange(axis=pg.ViewBox.YAxis)
            plot.getViewBox().setMouseEnabled(y=False)

            # X axis
            plot.setXRange(0, 20, padding=0)
            plot.getViewBox().setLimits(xMin=0)

            plot.setMinimumHeight(350)
            self.graphs.append(plot)
            graph_layout.addWidget(plot)

        graph_container.setLayout(graph_layout)
        scroll_area.setWidget(graph_container)

        # Add, edit and remove points buttons
        self.add_button = QPushButton("Add Point")
        self.edit_button = QPushButton("Edit Point")
        self.remove_button = QPushButton("Remove Point")

        self.add_button.clicked.connect(self.add_point_dialog)
        self.add_button.setShortcut(QKeySequence("Ctrl+A"))
        self.edit_button.clicked.connect(self.edit_point_dialog)
        self.edit_button.setShortcut(QKeySequence("Ctrl+E"))
        self.remove_button.clicked.connect(self.remove_point_dialog)
        self.remove_button.setShortcut(QKeySequence("Ctrl+R"))

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.add_button)
        button_layout.addWidget(self.edit_button)
        button_layout.addWidget(self.remove_button)

        menu_layout.addWidget(menubar)
        main_layout.addLayout(menu_layout)
        
        main_layout.addLayout(button_layout)

        # Save / Load

        save_action = QAction("Save to file", self)
        save_action.triggered.connect(self.json_save_sequence)
        self.file_menu.addAction(save_action)

        load_action = QAction("Load from file", self)
        load_action.triggered.connect(self.json_load_sequence)
        self.file_menu.addAction(load_action)

        # Serial settings

        self.serial_menu.aboutToShow.connect(self.populate_serial_menu)
        self.serial_menu.addAction(save_action)

        # Interpolation selection
        interpolation_label = QLabel("Interpolation:")
        self.interpolation_combo = QComboBox()
        self.interpolation_combo.addItems(self.interpolation_methods.keys())
        self.interpolation_combo.currentTextChanged.connect(self.change_interpolation)
        
        control_layout.addWidget(interpolation_label)
        control_layout.addWidget(self.interpolation_combo)

        # start / stop button
        start_stop_btn = QPushButton("Start / Stop")
        start_stop_btn.clicked.connect(self.serial_start_stop)
        start_stop_btn.setShortcut(QKeySequence("Ctrl+Return"))

        control_layout.addWidget(start_stop_btn)

        # Idle Button
        idle_btn = QPushButton("Idle")
        idle_btn.clicked.connect(self.serial_send_idle)
        idle_btn.setShortcut(QKeySequence("Ctrl+Shift+Return"))

        control_layout.addWidget(idle_btn)

        # Status indicator
        self.status_label = QLabel("Status: STOPPED")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")

        control_layout.addWidget(self.status_label)
        
        main_layout.addWidget(scroll_area)
        main_layout.addLayout(control_layout)

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def set_serial_port(self, p: str):
        self.serial_port = p

    def set_baud_rate(self, b: int):
        self.baud_rate = b

    def populate_serial_menu(self):
        """Refresh serial options dynamically when opening the menu."""
        self.serial_menu.clear()  # Clear previous items

        # ---- PORT SELECTION ----
        port_menu = QMenu("Port", self)

        available_ports = [port.device for port in serial.tools.list_ports.comports()]
        for port in available_ports:
            port_action = QAction(port, self, checkable=True)
            port_action.setChecked(port == self.serial_port)
            port_action.triggered.connect(lambda checked, p=port: self.set_serial_port(p))
            port_menu.addAction(port_action)

        self.serial_menu.addMenu(port_menu)

        # ---- BAUD RATE SELECTION ----
        baud_menu = QMenu("Baud Rate", self)
        baud_rates = [9600, 14400, 19200, 38400, 57600, 115200, 128000, 256000]
        for baud in baud_rates:
            baud_action = QAction(str(baud), self, checkable=True)
            baud_action.setChecked(baud == self.baud_rate)
            baud_action.triggered.connect(lambda checked, b=baud: self.set_baud_rate(b))
            baud_menu.addAction(baud_action)

        self.serial_menu.addMenu(baud_menu)

        # ---- FREQUENCY SELECTION ----
        freq_menu = QMenu("Frequency (Hz)", self)
        freq_values = [1, 5, 10, 20, 30, 50, 100]
        for freq in freq_values:
            freq_action = QAction(str(freq), self, checkable=True)
            freq_action.setChecked(freq == self.output_frequency)
            freq_action.triggered.connect(lambda checked, f=freq: self.change_frequency(f))
            freq_menu.addAction(freq_action)

        self.serial_menu.addMenu(freq_menu)

    def add_point_dialog(self):
        thruster_num, ok = QInputDialog.getInt(self, "Add Point", "Enter thruster number (1-8):", min=1, max=8)
        if not ok:
            return
        
        time_value, ok = QInputDialog.getDouble(self, "Add Point", "Enter time (s):", min=0, decimals=4)
        if not ok:
            return
        
        pwm_value, ok = QInputDialog.getInt(self, "Add Point", "Enter PWM value (1100-1900):", min=1100, max=1900)
        if not ok:
            return
        
        self.points[thruster_num - 1].append((time_value, pwm_value))
        self.points[thruster_num - 1].sort()  # Keep sorted by time
        self.update_graph(thruster_num - 1)

    def edit_point_dialog(self):
        thruster_num, ok = QInputDialog.getInt(self, "Edit Point", "Enter thruster number (1-8):", min=1, max=8)
        if not ok:
            return
        
        thruster_index = thruster_num - 1
        if not self.points[thruster_index]:
            QMessageBox.warning(self, "No Points", "No points to edit!")
            return
        
        point_list = [f"t={t:.2f}s, PWM={p}" for t, p in self.points[thruster_index]]
        selected_point, ok = QInputDialog.getItem(self, "Edit Point", "Select a point:", point_list, editable=False)
        if not ok : return
        
        selected_index = point_list.index(selected_point)
        new_time, ok = QInputDialog.getDouble(self, "Edit Point", "Enter new time (s):", min=0, decimals=4)
        if not ok:
            return
        
        new_pwm, ok = QInputDialog.getInt(self, "Edit Point", "Enter new PWM value (1100-1900):", min=1100, max=1900)
        if not ok:
            return
        
        self.points[thruster_index][selected_index] = (new_time, new_pwm)
        self.points[thruster_index].sort()
        self.update_graph(thruster_index)
    
    def remove_point_dialog(self):
        thruster_num, ok = QInputDialog.getInt(self, "Remove Point", "Enter thruster number (1-8):", min=1, max=8)
        if not ok:
            return
        
        thruster_index = thruster_num - 1
        if not self.points[thruster_index]:
            QMessageBox.warning(self, "No Points", "No points to remove!")
            return
        
        point_list = [f"t={t:.2f}s, PWM={p}" for t, p in self.points[thruster_index]]
        selected_point, ok = QInputDialog.getItem(self, "Remove Point", "Select a point to remove:", point_list, editable=False)
        if not ok:
            return
        
        selected_index = point_list.index(selected_point)
        del self.points[thruster_index][selected_index]
        self.update_graph(thruster_index)

    def update_graph(self, idx):
        """Re-draw the graph with the latest data."""
        points = self.points[idx]
        if points:
            x, y = zip(*sorted(points))
            self.graphs[idx].clear()
            self.graphs[idx].plot(x, y, pen='b', symbol='o', symbolBrush='r')
        else :
            self.graphs[idx].clear()

    def change_interpolation(self, method: str):
        """Change the interpolation method."""
        self.selected_interpolation = self.interpolation_methods[method]
        for i in range(THRUSTER_COUNT) : self.interpolate_thruster_curve(i)

    def interpolate_thruster_curve(self, thruster_index):
        points = self.points[thruster_index]

        times, pwm_values = zip(*points)
        times = np.array(times)
        pwm_values = np.array(pwm_values)

        if self.selected_interpolation == "linear":
            f = interp1d(times, pwm_values, kind="linear", fill_value="extrapolate")
        elif self.selected_interpolation == "constant":
            f = interp1d(times, pwm_values, kind="previous", fill_value="extrapolate")
        elif self.selected_interpolation == "polynomial":
            f = BarycentricInterpolator(times, pwm_values)

        self.compute_max_time()

        t_interp = np.linspace(0, round(self.max_time), int(self.max_time * self.output_frequency) + 1)
        y_interp = f(t_interp)

        y_interp = np.clip(y_interp, 1100, 1900)

        self.graphs[thruster_index].clear()
        self.graphs[thruster_index].plot(t_interp, y_interp, pen='b')  # Blue line for interpolated curve
        self.graphs[thruster_index].plot(times, pwm_values, pen=None, symbol='o', symbolBrush='r')  # Red dots for points

    def setup_serial(self):
        """Initialize the serial connection."""
        if self.serial_status == 1:
            self.serial_conn.close()
            self.serial_status = 0
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            if self.serial_conn.is_open :
                print(f"Connected to {self.serial_port} at {self.baud_rate} baud")
                self.serial_status = 1
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self.serial_conn = None
            self.serial_status = 0

    def change_frequency(self, freq):
        """Change the output frequency of serial messages."""
        self.output_frequency = int(freq)
        if self.serial_timer.isActive():
            self.serial_timer.start(1000 // self.output_frequency)
            
    def crc8(self, data: bytearray) -> int:
        # Polinomio CRC-8: 0x07 (x^8 + x^7 + x^6 + x^5 + x^4 + x^3 + x^2 + 1)
        polynomial = 0x07
        crc = 0x00  # Inizializza il CRC a zero
        
        for byte in data:
            crc ^= byte  # XOR del byte con il CRC
            for _ in range(8):  # 8 bit per ogni byte
                if crc & 0x80:  # Se il bit più significativo è 1
                    crc = (crc << 1) ^ polynomial  # Shift a sinistra e XOR con il polinomio
                else:
                    crc <<= 1  # Solo shift a sinistra
                crc &= 0xFF  # Mantieni solo gli 8 bit meno significativi (per evitare overflow)
        
        return crc

    def serial_send(self, pwm_values):
        """
        Send an array of 8 PWM values (uint16_t) to the microcontroller via serial.
        :param pwm_values: List of 8 integer PWM values (each in range [1100, 1900])
        """
        if len(pwm_values) != 8:
            raise ValueError("Expected 8 PWM values.")

        # Ensure PWM values are clamped within [1100, 1900]
        pwm_values = [max(1100, min(1900, pwm)) for pwm in pwm_values]

        # Pack the data into 16 bytes (8 x uint16_t) in little-endian format
        print(pwm_values)
        print(*pwm_values)
        packet = struct.pack('<8H', *pwm_values)

        # Open serial port and send data
        if(self.serial_status == 1):
            fullpacket = bytearray(bytes.fromhex('AA0000')) \
                + bytearray(packet) \
                + bytearray(struct.pack('<B', self.crc8(bytearray(bytes.fromhex('AA0000')) + bytearray(packet)))) \
                + bytearray(bytes.fromhex('EE'))
            self.serial_conn.write(fullpacket)
            print(f"Sent: {pwm_values}")
            print(f"The full bytes packet was: {fullpacket}")
        else:
            raise ConnectionError("Serial connection is not open. Check your port settings.")

    def serial_send_idle(self):
        """Send an idle command (1500 to all thrusters)."""
        if(self.serial_status == 0) : self.setup_serial()
        self.serial_timer.stop()
        self.serial_send([1500] * 8)
        self.serial_conn.close()
        self.serial_status = 0

    def serial_start_stop(self):
        if self.serial_status == 1:
            print("Stopping.")
            self.serial_send_idle()
        else:
            print("Starting")
            self.serial_start()
        
    def serial_start(self):
        self.setup_serial()
        if(not self.serial_status): raise ConnectionError("Serial connection is not open. Check your port settings.")
        self.compute_pwms()
        if(self.serial_status == 1):
            self.serial_conn.write(bytes.fromhex('FF'))
            self.serial_conn.write(bytes.fromhex('00'))
            self.serial_conn.write(bytes.fromhex('01'))
            self.serial_conn.write(bytes.fromhex('00'))
            self.serial_conn.write(bytes.fromhex('00'))
            self.serial_conn.write(bytes.fromhex('52'))
            self.serial_conn.write(bytes.fromhex('EE'))
        else : ConnectionError("Serial connection is not open. Check your port settings.")
        self.com_step = 0 # at each step, we increment this, so I know what values I have to output
        self.serial_timer.start(1000 // self.output_frequency)

    def compute_max_time(self):
        self.max_time = 0
        for i in range(THRUSTER_COUNT):
            if(self.points[i]):
                times, _ = zip(*self.points[i])
                a = max(times)
                if a > self.max_time : self.max_time = a

    def compute_pwms(self):
        self.compute_max_time()
        for i in range(THRUSTER_COUNT):
            points = self.points[i]
            times, pwm_values = zip(*points)
            times = np.array(times)
            pwm_values = np.array(pwm_values)
            if self.selected_interpolation == "linear":
                f = interp1d(times, pwm_values, kind="linear", fill_value="extrapolate")
            elif self.selected_interpolation == "constant":
                f = interp1d(times, pwm_values, kind="previous", fill_value="extrapolate")
            elif self.selected_interpolation == "polynomial":
                f = BarycentricInterpolator(times, pwm_values)
            t_interp = np.linspace(0, round(self.max_time), int(self.max_time * self.output_frequency) + 1) # each step should be the same as the period
            y_interp = f(t_interp)
            y_interp = np.clip(y_interp, 1100, 1900)
            self.cached_pwms[i] = (t_interp, y_interp)
        # for i in range(THRUSTER_COUNT) : print(f"({time}, {pwm})" for time, pwm in self.cached_pwms[i])

    def send_pwms(self):
        if self.com_step <= int(self.max_time * self.output_frequency) + 1:
            self.serial_send(self.get_pwms(self.com_step))
            self.com_step += 1
        else : self.serial_send_idle()

    def get_pwms(self, step: int) -> list[int]:
        """Lookup on cached pwm values"""
        try :
            result = []
            for i in range(THRUSTER_COUNT):
                result.append(int(self.cached_pwms[i][1][step]))
        except : ...
        return result

    def json_save_sequence(self):
        """Save sequence to JSON file."""
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(
            None, "Save Sequence", "", "JSON Files (*.json);;All Files (*)", options=options
        )

        if file_path:
            data = {
                "interpolation_method": self.selected_interpolation,
                "frequency": self.output_frequency,
                "thruster_data": {i: self.points[i] for i in range(THRUSTER_COUNT)}
            }

            try:
                with open(file_path, "w") as file:
                    json.dump(data, file, indent=4)
                print(f"Sequence saved to {file_path}")
            except Exception as e:
                print(f"Error saving JSON: {e}")

    def json_load_sequence(self):
        """Load sequence from JSON file."""
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(
            None, "Load Sequence", "", "JSON Files (*.json);;All Files (*)", options=options
        )

        if file_path:
            try:
                with open(file_path, "r") as file:
                    data = json.load(file)

                self.selected_interpolation = data.get("interpolation_method", "linear")
                self.output_frequency = data.get("frequency", 20)

                for i in range(THRUSTER_COUNT):
                    self.points[i] = data["thruster_data"].get(str(i), [])
                    self.update_graph(i)
                    self.interpolate_thruster_curve(i)

                #self.compute_pwms()
                print(f"Sequence loaded from {file_path}")

            except Exception as e:
                print(f"Error loading JSON: {e}")

if __name__ == "__main__": main()
