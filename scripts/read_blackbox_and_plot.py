import sys
import serial
import serial.tools.list_ports
import numpy as np
import math
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QComboBox, QLabel, 
                             QCheckBox, QGroupBox, QGridLayout, QMessageBox, 
                             QSlider, QSpinBox)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont
# pyrefly: ignore [missing-import]
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
# pyrefly: ignore [missing-import]
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
# pyrefly: ignore [missing-import]
from matplotlib.figure import Figure
# pyrefly: ignore [missing-import]
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# pyrefly: ignore [missing-import]
from matplotlib import colors as mcolors


class SerialReaderThread(QThread):
    data_ready = pyqtSignal(list, list)
    status_update = pyqtSignal(str)
    progress_update = pyqtSignal(int)
    error_signal = pyqtSignal(str)
    finished_signal = pyqtSignal()

    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.status_update.emit(f"Connected to {self.port}. Waiting for Blackbox Dump...")
            
            in_dump = False
            headers = []
            flights = []
            current_flight = []
            current_tuning = []

            while self.running:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not in_dump:
                        if "--- BEGIN BLACKBOX DUMP ---" in line:
                            in_dump = True
                            self.status_update.emit("Reading Blackbox data...")
                        continue
                    
                    if in_dump:
                        if "--- END BLACKBOX DUMP ---" in line or "REACHED END OF FLIGHT DATA" in line:
                            if current_flight:
                                flights.append({'data': current_flight, 'tuning': current_tuning})
                            total_packets = sum(len(f['data']) for f in flights)
                            self.status_update.emit(f"Finished reading. {len(flights)} flights, {total_packets} packets.")
                            self.data_ready.emit(headers, flights)
                            break
                        
                        if "--- FLIGHT SEPARATOR ---" in line:
                            if current_flight:
                                flights.append({'data': current_flight, 'tuning': current_tuning})
                                current_flight = []
                                current_tuning = []
                            continue

                        if "--- TUNING_UPDATE:" in line:
                            try:
                                parts = line.strip("- ").split(":")[1].strip().split(",")
                                if len(parts) == 4:
                                    current_tuning.append({
                                        'index': len(current_flight),
                                        'type': parts[0],
                                        'v1': float(parts[1]),
                                        'v2': float(parts[2]),
                                        'v3': float(parts[3])
                                    })
                            except:
                                pass
                            continue

                        if not headers:
                            headers = line.split(',')
                            continue
                        
                        try:
                            row = [float(x) for x in line.split(',')]
                            if len(row) == len(headers):
                                current_flight.append(row)
                                total_read = sum(len(f['data']) for f in flights) + len(current_flight)
                                if total_read % 100 == 0:
                                    self.progress_update.emit(total_read)
                        except ValueError:
                            pass
            ser.close()
        except Exception as e:
            self.error_signal.emit(str(e))
        finally:
            self.finished_signal.emit()
            
    def stop(self):
        self.running = False


class BlackboxViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DroneFC Blackbox Data Viewer")
        
        self.headers = []
        self.all_flights_data = []
        self.data_matrix = None
        self.lines = {} 
        self.calculated_time = None
        self.calculated_yaw = None

        self.playback_timer = QTimer(self)
        self.playback_timer.timeout.connect(self.play_next_frame)
        self.playback_speed_ms = 40 # 25fps update

        self._setup_ui()
        self.refresh_ports()

    def _create_drone_geometry(self):
        vertices = []
        faces = []
        face_colors = []
        
        def add_box(center_x, center_y, center_z, l, w, h, c, yaw_rot=0):
            start_idx = len(vertices)
            V = np.array([
                [-l, -w, -h], [l, -w, -h], [l, w, -h], [-l, w, -h],
                [-l, -w,  h], [l, -w,  h], [l, w,  h], [-l, w,  h]
            ])
            rad = np.radians(yaw_rot)
            R = np.array([[np.cos(rad), -np.sin(rad), 0],
                          [np.sin(rad),  np.cos(rad), 0],
                          [0,            0,           1]])
            V = (R @ V.T).T
            V[:, 0] += center_x
            V[:, 1] += center_y
            V[:, 2] += center_z
            
            vertices.extend(V.tolist())
            f = [
                [0, 1, 2, 3], [4, 5, 6, 7], [1, 5, 6, 2],
                [0, 4, 7, 3], [2, 6, 7, 3], [0, 1, 5, 4]
            ]
            for face in f:
                faces.append([idx + start_idx for idx in face])
                face_colors.append(c)
                
        # Central body
        add_box(0, 0, 0, 0.25, 0.25, 0.1, '#222222')
        
        # Arms
        arm_l = 0.6
        add_box(arm_l/2, arm_l/2, 0, arm_l/2, 0.04, 0.04, '#888888', 45)
        add_box(arm_l/2, -arm_l/2, 0, arm_l/2, 0.04, 0.04, '#888888', -45)
        add_box(-arm_l/2, arm_l/2, 0, arm_l/2, 0.04, 0.04, '#888888', 135)
        add_box(-arm_l/2, -arm_l/2, 0, arm_l/2, 0.04, 0.04, '#888888', -135)
        
        # Rotors index recording
        self.rotor_start_face_idx = len(faces)
        
        # Front-Right (M2 - CCW) -> Red
        add_box(arm_l, arm_l, 0.1, 0.2, 0.2, 0.02, '#FF3333')
        # Front-Left (M4 - CW) -> Blue
        add_box(arm_l, -arm_l, 0.1, 0.2, 0.2, 0.02, '#3333FF')
        # Back-Right (M1 - CW) -> Blue
        add_box(-arm_l, arm_l, 0.1, 0.2, 0.2, 0.02, '#3333FF')
        # Back-Left (M3 - CCW) -> Red
        add_box(-arm_l, -arm_l, 0.1, 0.2, 0.2, 0.02, '#FF3333')
        
        # Indicator Arrow (Front)
        add_box(0.35, 0, 0.05, 0.1, 0.02, 0.02, '#00FF00')
        
        return np.array(vertices).T, faces, face_colors

    def _setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- Top Bar (Controls) ---
        control_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        control_layout.addWidget(QLabel("Serial Port:"))
        control_layout.addWidget(self.port_combo)
        
        refresh_btn = QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self.refresh_ports)
        control_layout.addWidget(refresh_btn)
        
        self.connect_btn = QPushButton("Connect & Read Data")
        self.connect_btn.clicked.connect(self.start_reading)
        self.connect_btn.setStyleSheet("background-color: #0078D7; color: white; font-weight: bold; padding: 5px;")
        control_layout.addWidget(self.connect_btn)

        self.flight_combo = QComboBox()
        self.flight_combo.setMinimumWidth(100)
        self.flight_combo.setEnabled(False)
        self.flight_combo.currentIndexChanged.connect(self.select_flight)
        control_layout.addWidget(QLabel(" Flight:"))
        control_layout.addWidget(self.flight_combo)

        self.status_lbl = QLabel("Ready")
        control_layout.addWidget(self.status_lbl)
        control_layout.addStretch()

        self.tuning_label = QLabel("<b>Active Tuning:</b> Default")
        self.tuning_label.setStyleSheet("background-color: #333; color: white; padding: 5px; border-radius: 3px; font-family: monospace; font-size: 10px;")
        control_layout.addWidget(self.tuning_label)

        main_layout.addLayout(control_layout)

        # --- Middle Section: Checkboxes + 3D View ---
        mid_layout = QHBoxLayout()
        
        self.checkbox_group = QGroupBox("Select Data to Plot")
        self.checkbox_layout = QGridLayout()
        self.checkbox_group.setLayout(self.checkbox_layout)
        self.checkboxes = {}
        mid_layout.addWidget(self.checkbox_group, stretch=1)
        
        self.fig_3d = Figure(figsize=(6, 5))
        self.canvas_3d = FigureCanvas(self.fig_3d)
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
        mid_layout.addWidget(self.canvas_3d, stretch=3)
        
        main_layout.addLayout(mid_layout, stretch=2)

        # --- Bottom Section: 2D Graph ---
        self.fig_2d = Figure(figsize=(10, 4))
        self.canvas_2d = FigureCanvas(self.fig_2d)
        self.toolbar_2d = NavigationToolbar(self.canvas_2d, self)
        self.ax_2d = self.fig_2d.add_subplot(111)
        
        main_layout.addWidget(self.toolbar_2d)
        main_layout.addWidget(self.canvas_2d, stretch=3)
        
        # --- Timeline Slider and Playback Controls ---
        slider_layout = QHBoxLayout()
        
        self.play_btn = QPushButton("Play")
        self.play_btn.setCheckable(True)
        self.play_btn.setEnabled(False)
        self.play_btn.toggled.connect(self.toggle_playback)
        
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setEnabled(False)
        self.slider.valueChanged.connect(self.slider_changed)
        
        self.time_lbl = QLabel("Time: 0.00s")
        self.time_lbl.setMinimumWidth(80)
        
        self.window_spin = QSpinBox()
        self.window_spin.setRange(1, 300)
        self.window_spin.setValue(10)
        self.window_spin.valueChanged.connect(self.update_x_axis)
        
        self.show_all_cb = QCheckBox("Show All Time")
        self.show_all_cb.setChecked(True)
        self.show_all_cb.stateChanged.connect(self.update_x_axis)
        
        slider_layout.addWidget(self.play_btn)
        slider_layout.addWidget(QLabel("Playback:"))
        slider_layout.addWidget(self.slider)
        slider_layout.addWidget(self.time_lbl)
        slider_layout.addWidget(QLabel(" | View Window (s):"))
        slider_layout.addWidget(self.window_spin)
        slider_layout.addWidget(self.show_all_cb)
        main_layout.addLayout(slider_layout)
        
        self._init_plots()

    def _init_plots(self):
        # 2D Graph Setup
        self.fig_2d.patch.set_facecolor('#F0F0F0')
        self.ax_2d.set_facecolor('#FFFFFF')
        self.ax_2d.grid(True, linestyle='--', alpha=0.7)
        self.ax_2d.set_title("Blackbox Telemetry")
        self.ax_2d.set_xlabel("Time (s)")
        self.ax_2d.set_ylabel("Value")

        # 3D Graph Setup
        self.ax_3d.set_title('3D Orientation (Drone)')
        self.ax_3d.set_xlim([-1.0, 1.0])
        self.ax_3d.set_ylim([-1.0, 1.0])
        self.ax_3d.set_zlim([-1.0, 1.0])
        self.ax_3d.set_xlabel('X (Forward)')
        self.ax_3d.set_ylabel('Y (Left)')
        self.ax_3d.set_zlabel('Z (Up)')
        self.ax_3d.set_box_aspect((1, 1, 1))
        
        self.ax_3d.set_xticklabels([])
        self.ax_3d.set_yticklabels([])
        self.ax_3d.set_zticklabels([])

        # Drone Geometry
        self.drone_V, self.drone_faces, self.face_colors = self._create_drone_geometry()

        self.poly3d = Poly3DCollection([], facecolors=self.face_colors, edgecolors='black', linewidths=0.5, alpha=0.9)
        self.ax_3d.add_collection3d(self.poly3d)

        self.axis_line_x, = self.ax_3d.plot([], [], [], color='red', linewidth=3)
        self.axis_line_y, = self.ax_3d.plot([], [], [], color='green', linewidth=3)
        self.axis_line_z, = self.ax_3d.plot([], [], [], color='blue', linewidth=3)
        self.thrust_line, = self.ax_3d.plot([], [], [], color='yellow', linewidth=5, zorder=5)

        self.update_3d_drone(0.0, 0.0, 0.0, 1000, 1000, 1000, 1000)
        
        # Make margins tight to prevent UserWarning
        self.fig_3d.subplots_adjust(left=0, right=1, bottom=0, top=1)
        self.fig_2d.tight_layout()

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            # Filter to only show USB serial connections
            if p.vid is not None or "usb" in p.device.lower() or "acm" in p.device.lower():
                self.port_combo.addItem(p.device)
            
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports found")

    def start_reading(self):
        port = self.port_combo.currentText()
        if not port or port == "No ports found":
            QMessageBox.warning(self, "Error", "No valid serial port selected.")
            return

        self.connect_btn.setEnabled(False)
        self.status_lbl.setText("Connecting...")

        self.reader_thread = SerialReaderThread(port)
        self.reader_thread.status_update.connect(self.update_status)
        self.reader_thread.progress_update.connect(self.update_progress)
        self.reader_thread.data_ready.connect(self.process_data)
        self.reader_thread.error_signal.connect(self.handle_error)
        self.reader_thread.finished_signal.connect(self.reading_finished)
        self.reader_thread.start()

    def update_status(self, msg):
        self.status_lbl.setText(msg)

    def update_progress(self, packets):
        self.status_lbl.setText(f"Reading... {packets} packets loaded.")

    def handle_error(self, err_msg):
        QMessageBox.critical(self, "Serial Error", f"Error reading serial port:\n{err_msg}")
        self.status_lbl.setText("Error occurred.")

    def reading_finished(self):
        self.connect_btn.setEnabled(True)

    def process_data(self, headers, flights):
        if not flights or all(len(f['data']) == 0 for f in flights):
            QMessageBox.warning(self, "No Data", "No blackbox data was read.")
            return
            
        self.headers = headers
        self.all_flights_data = flights
        
        self.flight_combo.blockSignals(True)
        self.flight_combo.clear()
        for i, f in enumerate(flights):
            self.flight_combo.addItem(f"Flight {i+1} ({len(f['data'])} pkts)")
        self.flight_combo.setEnabled(True)
        self.flight_combo.blockSignals(False)
        
        if self.flight_combo.currentIndex() != 0:
            self.flight_combo.setCurrentIndex(0)
        else:
            self.select_flight(0)

    def select_flight(self, idx):
        if idx < 0 or idx >= len(self.all_flights_data):
            return
            
        flight_info = self.all_flights_data[idx]
        flight_data = flight_info['data']
        self.current_tuning_events = flight_info.get('tuning', [])
        
        if not flight_data:
            return
            
        self.data_matrix = np.array(flight_data)
        
        if 'dt_us' in self.headers:
            dt_idx = self.headers.index('dt_us')
            raw_time = self.data_matrix[:, dt_idx].copy()
            for i in range(1, len(raw_time)):
                if raw_time[i] < raw_time[i-1]:
                    raw_time[i:] += 65535
            self.calculated_time = (raw_time - raw_time[0]) / 100.0
            
            self.data_matrix[:, dt_idx] = self.calculated_time
        else:
            self.calculated_time = np.arange(len(self.data_matrix)) * 0.02
            
        self.calculated_yaw = np.zeros(len(self.data_matrix))
        if 'YawRate' in self.headers:
            yr_idx = self.headers.index('YawRate')
            for i in range(1, len(self.data_matrix)):
                dt = self.calculated_time[i] - self.calculated_time[i-1]
                yaw_change = (self.data_matrix[i, yr_idx] / 100.0) * dt
                new_yaw = self.calculated_yaw[i-1] + yaw_change
                if new_yaw > 180.0: new_yaw -= 360.0
                if new_yaw < -180.0: new_yaw += 360.0
                self.calculated_yaw[i] = new_yaw

        if 'Roll' in self.headers: self.data_matrix[:, self.headers.index('Roll')] /= 100.0
        if 'Pitch' in self.headers: self.data_matrix[:, self.headers.index('Pitch')] /= 100.0
        if 'YawRate' in self.headers: self.data_matrix[:, self.headers.index('YawRate')] /= 100.0
        if 'PID_R' in self.headers: self.data_matrix[:, self.headers.index('PID_R')] /= 100.0
        if 'PID_P' in self.headers: self.data_matrix[:, self.headers.index('PID_P')] /= 100.0
        if 'PID_Y' in self.headers: self.data_matrix[:, self.headers.index('PID_Y')] /= 100.0
        if 'RC_Roll' in self.headers: self.data_matrix[:, self.headers.index('RC_Roll')] /= 100.0
        if 'RC_Pitch' in self.headers: self.data_matrix[:, self.headers.index('RC_Pitch')] /= 100.0
        if 'RC_Yaw' in self.headers: self.data_matrix[:, self.headers.index('RC_Yaw')] /= 100.0

        self.slider.setMaximum(len(self.data_matrix) - 1)
        self.slider.setValue(0)
        self.slider.setEnabled(True)
        self.play_btn.setEnabled(True)

        self.build_checkboxes()
        self.plot_all()
        self.slider_changed(0) 
        self.play_btn.setChecked(True)

    def build_checkboxes(self):
        checked_states = {h: cb.isChecked() for h, cb in self.checkboxes.items()}
        for i in reversed(range(self.checkbox_layout.count())): 
            widget = self.checkbox_layout.itemAt(i).widget()
            if widget is not None:
                widget.setParent(None)
        
        self.checkboxes.clear()
        
        groups = {
            'Angles': ['Roll', 'Pitch', 'YawRate'],
            'PID': ['PID_R', 'PID_P', 'PID_Y'],
            'RC': ['RC_Roll', 'RC_Pitch', 'RC_Yaw', 'RC_Throttle'],
            'Motors': ['M1', 'M2', 'M3', 'M4'],
            'Other': []
        }
        
        mono_font = QFont("Monospace")
        mono_font.setStyleHint(QFont.StyleHint.TypeWriter)
        
        for i, header in enumerate(self.headers):
            if header == 'dt_us': continue
            
            cb = QCheckBox(f"{header}: {0.0:8.2f}")
            cb.setFont(mono_font)
            if header in checked_states:
                cb.setChecked(checked_states[header])
            elif header in ['Roll', 'Pitch']: 
                cb.setChecked(True)
            else: 
                cb.setChecked(False)
            cb.stateChanged.connect(self.update_plot_visibility)
            self.checkboxes[header] = cb
            
            placed = False
            for c_idx, (group_name, group_items) in enumerate(groups.items()):
                if header in group_items:
                    self.checkbox_layout.addWidget(cb, group_items.index(header), c_idx)
                    placed = True
                    break
            if not placed:
                self.checkbox_layout.addWidget(cb, len(groups['Other']), list(groups.keys()).index('Other'))
                groups['Other'].append(header)

    def plot_all(self):
        self.ax_2d.clear()
        self.ax_2d.grid(True, linestyle='--', alpha=0.7)
        self.ax_2d.set_title("Blackbox Telemetry")
        self.ax_2d.set_xlabel("Time (s)")
        self.ax_2d.set_ylabel("Value")
        
        self.lines.clear()
        colors = ['#FF0000', '#00AA00', '#0000FF', '#FFA500', '#800080', '#00AAAA', '#A52A2A', '#FF00FF']
        
        c_idx = 0
        for i, header in enumerate(self.headers):
            if header == 'dt_us': continue
            y_data = self.data_matrix[:, i]
            line, = self.ax_2d.plot(self.calculated_time, y_data, label=header, color=colors[c_idx % len(colors)], linewidth=1.5)
            if header in self.checkboxes:
                line.set_visible(self.checkboxes[header].isChecked())
            self.lines[header] = line
            c_idx += 1
            
        self.ax_2d.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0))
        
        # 1. Add vertical time cursor
        self.time_cursor = self.ax_2d.axvline(x=self.calculated_time[0], color='black', linestyle='-', linewidth=1.5, zorder=10)
        
        # 2. Add highlight for Throttle > 1300
        if 'RC_Throttle' in self.headers:
            t_idx = self.headers.index('RC_Throttle')
            throttle_data = self.data_matrix[:, t_idx]
            self.ax_2d.fill_between(self.calculated_time, 0, 1, where=(throttle_data > 1300),
                                    color='red', alpha=0.15, transform=self.ax_2d.get_xaxis_transform(),
                                    label='Hovering (>1300)')
                                    
        self.update_plot_visibility()

    def update_plot_visibility(self):
        visible_y_min = float('inf')
        visible_y_max = float('-inf')
        any_visible = False
        
        for header, cb in self.checkboxes.items():
            if header in self.lines:
                is_checked = cb.isChecked()
                self.lines[header].set_visible(is_checked)
                if is_checked:
                    any_visible = True
                    ydata = self.lines[header].get_ydata()
                    visible_y_min = min(visible_y_min, np.min(ydata))
                    visible_y_max = max(visible_y_max, np.max(ydata))
        
        if any_visible:
            margin = (visible_y_max - visible_y_min) * 0.1
            if margin == 0: margin = 1.0
            self.ax_2d.set_ylim(visible_y_min - margin, visible_y_max + margin)
            
        self.canvas_2d.draw_idle()

    def euler_to_rotmat(self, r_deg, p_deg, y_deg):
        cx, sx = np.cos(np.radians(r_deg)), np.sin(np.radians(r_deg))
        cy, sy = np.cos(np.radians(p_deg)), np.sin(np.radians(p_deg))
        cz, sz = np.cos(np.radians(y_deg)), np.sin(np.radians(y_deg))

        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def update_3d_drone(self, r_val, p_val, y_val, m1, m2, m3, m4):
        # Lock elevation to 30 degrees, allow user to rotate azimuth (Z axis)
        self.ax_3d.view_init(elev=30, azim=self.ax_3d.azim)
        
        R = self.euler_to_rotmat(r_val, p_val, y_val)
        V_rot = R @ self.drone_V

        rotated_faces = []
        for face in self.drone_faces:
            rotated_faces.append([V_rot[:, idx] for idx in face])
        self.poly3d.set_verts(rotated_faces)

        x_axis = R @ np.array([2.0, 0, 0])
        self.axis_line_x.set_data([0, x_axis[0]], [0, x_axis[1]])
        self.axis_line_x.set_3d_properties([0, x_axis[2]])

        y_axis = R @ np.array([0, 2.0, 0])
        self.axis_line_y.set_data([0, y_axis[0]], [0, y_axis[1]])
        self.axis_line_y.set_3d_properties([0, y_axis[2]])

        z_axis = R @ np.array([0, 0, 2.0])
        self.axis_line_z.set_data([0, z_axis[0]], [0, z_axis[1]])
        self.axis_line_z.set_3d_properties([0, z_axis[2]])

        # Net Force Vector Visualization (Thrust + Gravity)
        HOVER_THROTTLE = 1300.0
        
        # Linear map: idle (4000) -> 0.0, hover (4 * 1300) -> 1.0 (cancels gravity)
        total_thrust = m1 + m2 + m3 + m4
        hover_total = 4.0 * HOVER_THROTTLE
        idle_total = 4000.0
        thrust_mag = max((total_thrust - idle_total) / (hover_total - idle_total), 0.0)
        
        # Calculate exaggerated torque for the yellow vector so it leans visibly
        # Pitch Torque (X axis - Forward): Rear (M1, M3) - Front (M2, M4)
        torque_x = ((m1 + m3) - (m2 + m4)) / 1000.0
        # Roll Torque (Y axis - Left): Right (M1, M2) - Left (M3, M4)
        torque_y = ((m1 + m2) - (m3 + m4)) / 1000.0
        
        torque_exaggeration = 5.0
        thrust_vector_local = np.array([torque_x * torque_exaggeration, torque_y * torque_exaggeration, thrust_mag])
        
        # Rotate local thrust to global, and add global gravity vector
        thrust_vector_global = R @ thrust_vector_local
        gravity_vector_global = np.array([0, 0, -1.0])
        
        net_vector = thrust_vector_global + gravity_vector_global
        
        # Scale by 1.5 for better visibility in the 3D plot
        net_vector *= 1.5
        
        self.thrust_line.set_data([0, net_vector[0]], [0, net_vector[1]])
        self.thrust_line.set_3d_properties([0, net_vector[2]])

        # Update rotor colors based on RPM (1000 - 2000)
        # 6 faces per rotor
        def get_rotor_color(val, cw):
            # clamp and normalize between 1000 and 2000
            norm = min(max((val - 1000) / 1000.0, 0.0), 1.0)
            if cw:
                # Dark Blue to Bright Cyan/Blue for CW
                return mcolors.to_rgba((0, norm, norm * 0.5 + 0.5))
            else:
                # Dark Red to Bright Yellow/Red for CCW
                return mcolors.to_rgba((norm * 0.5 + 0.5, norm, 0))

        # FR (M2, CCW), FL (M4, CW), BR (M1, CW), BL (M3, CCW)
        fc = self.face_colors.copy()
        start = self.rotor_start_face_idx
        
        # M2 (FR - CCW)
        c_m2 = get_rotor_color(m2, cw=False)
        for i in range(6): fc[start + 0 + i] = c_m2
        
        # M4 (FL - CW)
        c_m4 = get_rotor_color(m4, cw=True)
        for i in range(6): fc[start + 6 + i] = c_m4
            
        # M1 (BR - CW)
        c_m1 = get_rotor_color(m1, cw=True)
        for i in range(6): fc[start + 12 + i] = c_m1
            
        # M3 (BL - CCW)
        c_m3 = get_rotor_color(m3, cw=False)
        for i in range(6): fc[start + 18 + i] = c_m3

        self.poly3d.set_facecolors(fc)
        self.canvas_3d.draw_idle()

    def toggle_playback(self, playing):
        if playing:
            self.play_btn.setText("Pause")
            self.playback_timer.start(self.playback_speed_ms)
        else:
            self.play_btn.setText("Play")
            self.playback_timer.stop()

    def play_next_frame(self):
        if self.data_matrix is None: return
        
        # Advance slider by roughly the time passed (e.g. 40ms)
        # Since dt can vary, we search for the next index that matches
        current_idx = self.slider.value()
        target_time = self.calculated_time[current_idx] + (self.playback_speed_ms / 1000.0)
        
        # Find next index
        next_idx = current_idx
        while next_idx < len(self.calculated_time) and self.calculated_time[next_idx] < target_time:
            next_idx += 1
            
        if next_idx >= self.slider.maximum():
            self.slider.setValue(0) # Loop back to the beginning
        else:
            self.slider.setValue(next_idx)

    def update_x_axis(self):
        if self.data_matrix is None: return
        if self.show_all_cb.isChecked():
            self.ax_2d.set_xlim(self.calculated_time[0], self.calculated_time[-1])
        else:
            t = self.calculated_time[self.slider.value()]
            window = self.window_spin.value()
            self.ax_2d.set_xlim(max(0, t - window/2), t + window/2)
        self.canvas_2d.draw_idle()

    def slider_changed(self, value):
        if self.data_matrix is None or value >= len(self.data_matrix):
            return
            
        t = self.calculated_time[value]
        self.time_lbl.setText(f"Time: {t:.2f} s")
        
        # Update the time cursor position on the 2D plot
        if hasattr(self, 'time_cursor'):
            self.time_cursor.set_xdata([t, t])
        
        # Find active tuning for this time index
        if hasattr(self, 'current_tuning_events'):
            active_tuning = {}
            for ev in self.current_tuning_events:
                if ev['index'] <= value:
                    active_tuning[ev['type']] = ev
            
            text = "<b>Active Tuning:</b> "
            if not active_tuning:
                text += "Default"
            else:
                parts = []
                if 'PID_RP' in active_tuning:
                    ev = active_tuning['PID_RP']
                    parts.append(f"RP({ev['v1']:.2f}, {ev['v2']:.2f}, {ev['v3']:.2f})")
                if 'PID_YAW' in active_tuning:
                    ev = active_tuning['PID_YAW']
                    parts.append(f"YAW({ev['v1']:.2f}, {ev['v2']:.2f}, {ev['v3']:.2f})")
                if 'BIAS' in active_tuning:
                    ev = active_tuning['BIAS']
                    parts.append(f"BIAS({ev['v1']:.2f}, {ev['v2']:.2f}, {ev['v3']:.2f})")
                if 'EKF' in active_tuning:
                    ev = active_tuning['EKF']
                    parts.append(f"EKF({ev['v1']:.5f}, {ev['v2']:.5f}, {ev['v3']:.2f})")
                text += " | ".join(parts)
            self.tuning_label.setText(text)
        
        self.update_x_axis()
        
        # Update checkbox texts with current values
        for header, cb in self.checkboxes.items():
            if header in self.headers:
                val = self.data_matrix[value, self.headers.index(header)]
                cb.setText(f"{header}: {val:8.2f}")
        
        r_val = self.data_matrix[value, self.headers.index('Roll')] if 'Roll' in self.headers else 0.0
        p_val = self.data_matrix[value, self.headers.index('Pitch')] if 'Pitch' in self.headers else 0.0
        y_val = self.calculated_yaw[value]
        
        m1 = self.data_matrix[value, self.headers.index('M1')] if 'M1' in self.headers else 1000
        m2 = self.data_matrix[value, self.headers.index('M2')] if 'M2' in self.headers else 1000
        m3 = self.data_matrix[value, self.headers.index('M3')] if 'M3' in self.headers else 1000
        m4 = self.data_matrix[value, self.headers.index('M4')] if 'M4' in self.headers else 1000

        self.update_3d_drone(r_val, p_val, y_val, m1, m2, m3, m4)

    def closeEvent(self, event):
        if hasattr(self, 'reader_thread') and self.reader_thread.isRunning():
            self.reader_thread.stop()
            self.reader_thread.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = BlackboxViewer()
    window.showFullScreen()
    sys.exit(app.exec())
