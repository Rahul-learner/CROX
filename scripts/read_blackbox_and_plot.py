import sys
import serial
import serial.tools.list_ports
import numpy as np
import math
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QComboBox, QLabel, 
                             QCheckBox, QGroupBox, QGridLayout, QMessageBox, 
                             QSlider, QSpinBox, QScrollArea, QFrame)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont, QColor
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
        self.active_flight_indices = []
        self.deleted_flight_indices = []
        self.current_main_idx = -1
        
        self.data_matrix = None
        self.calculated_time = None
        self.calculated_yaw = None
        self.current_tuning_events = []
        
        self.flight_plots = [] # list of dicts with 'idx', 'fig', 'canvas', 'ax', 'lines', 'time_cursor'
        self.line_colors = ['#FF0000', '#00AA00', '#0000FF', '#FFA500', '#800080', '#00AAAA', '#A52A2A', '#FF00FF']

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

        control_layout.addSpacing(20)

        # Flight Management
        self.flight_combo = QComboBox()
        self.flight_combo.setMinimumWidth(100)
        self.flight_combo.setEnabled(False)
        self.flight_combo.currentIndexChanged.connect(self.select_main_flight)
        
        self.delete_btn = QPushButton("Delete")
        self.delete_btn.setEnabled(False)
        self.delete_btn.clicked.connect(self.delete_current_flight)
        
        self.sort_btn = QPushButton("Sort by Duration")
        self.sort_btn.setEnabled(False)
        self.sort_btn.clicked.connect(self.sort_active_flights)
        
        control_layout.addWidget(QLabel(" Main Flight:"))
        control_layout.addWidget(self.flight_combo)
        control_layout.addWidget(self.delete_btn)
        control_layout.addWidget(self.sort_btn)

        control_layout.addSpacing(20)
        
        self.deleted_combo = QComboBox()
        self.deleted_combo.setMinimumWidth(100)
        self.deleted_combo.setEnabled(False)
        self.restore_btn = QPushButton("Restore")
        self.restore_btn.setEnabled(False)
        self.restore_btn.clicked.connect(self.restore_deleted_flight)
        
        control_layout.addWidget(QLabel(" Deleted:"))
        control_layout.addWidget(self.deleted_combo)
        control_layout.addWidget(self.restore_btn)

        control_layout.addStretch()
        self.status_lbl = QLabel("Ready")
        control_layout.addWidget(self.status_lbl)

        main_layout.addLayout(control_layout)

        # --- Middle Section: Checkboxes + 3D View ---
        mid_layout = QHBoxLayout()
        
        self.checkbox_group = QGroupBox("Select Data to Plot (Main Flight values shown)")
        self.checkbox_layout = QGridLayout()
        self.checkbox_group.setLayout(self.checkbox_layout)
        self.checkboxes = {}
        mid_layout.addWidget(self.checkbox_group, stretch=1)
        
        self.fig_3d = Figure(figsize=(6, 5))
        self.canvas_3d = FigureCanvas(self.fig_3d)
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
        mid_layout.addWidget(self.canvas_3d, stretch=3)
        
        main_layout.addLayout(mid_layout, stretch=2)

        # --- Bottom Section: 2D Graphs (Scrollable) ---
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        
        self.fig_2d = Figure()
        self.fig_2d.patch.set_facecolor('#F0F0F0')
        self.canvas_2d = FigureCanvas(self.fig_2d)
        
        self.canvas_container = QWidget()
        self.canvas_layout = QVBoxLayout(self.canvas_container)
        self.canvas_layout.setContentsMargins(0, 0, 0, 0)
        self.canvas_layout.addWidget(self.canvas_2d)
        
        self.scroll_area.setWidget(self.canvas_container)
        
        self.toolbar_2d = NavigationToolbar(self.canvas_2d, self)
        
        main_layout.addWidget(self.toolbar_2d)
        main_layout.addWidget(self.scroll_area, stretch=3)
        
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
        
        self._init_3d_plot()

    def _init_3d_plot(self):
        # 3D Graph Setup
        self.ax_3d.set_title('3D Orientation (Main Flight)')
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
        self.fig_3d.subplots_adjust(left=0, right=1, bottom=0, top=1)

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
        self.active_flight_indices = list(range(len(flights)))
        self.deleted_flight_indices = []
        
        self.build_checkboxes()
        
        self.update_dropdowns()
        self.select_main_flight(0)

    def update_dropdowns(self):
        self.flight_combo.blockSignals(True)
        self.flight_combo.clear()
        for i, idx in enumerate(self.active_flight_indices):
            f = self.all_flights_data[idx]
            self.flight_combo.addItem(f"Flight {idx+1} ({len(f['data'])} pkts)", userData=idx)
        self.flight_combo.blockSignals(False)
        
        has_active = len(self.active_flight_indices) > 0
        self.flight_combo.setEnabled(has_active)
        self.delete_btn.setEnabled(has_active)
        self.sort_btn.setEnabled(has_active)
        
        self.deleted_combo.blockSignals(True)
        self.deleted_combo.clear()
        for i, idx in enumerate(self.deleted_flight_indices):
            f = self.all_flights_data[idx]
            self.deleted_combo.addItem(f"Flight {idx+1} ({len(f['data'])} pkts)", userData=idx)
        self.deleted_combo.blockSignals(False)
        
        has_deleted = len(self.deleted_flight_indices) > 0
        self.deleted_combo.setEnabled(has_deleted)
        self.restore_btn.setEnabled(has_deleted)

    def delete_current_flight(self):
        if not self.active_flight_indices: return
        idx_to_delete = self.active_flight_indices.pop(0) # main flight is always 0
        self.deleted_flight_indices.append(idx_to_delete)
        self.update_dropdowns()
        if self.active_flight_indices:
            self.select_main_flight(0)
        else:
            self.fig_2d.clear()
            self.canvas_2d.draw_idle()

    def restore_deleted_flight(self):
        if not self.deleted_flight_indices: return
        list_idx = self.deleted_combo.currentIndex()
        if list_idx < 0: return
        flight_to_restore = self.deleted_flight_indices.pop(list_idx)
        self.active_flight_indices.append(flight_to_restore)
        self.update_dropdowns()
        self.select_main_flight(len(self.active_flight_indices) - 1)

    def sort_active_flights(self):
        if not self.active_flight_indices: return
        # Sort by duration/length (longest to shortest)
        self.active_flight_indices.sort(key=lambda idx: len(self.all_flights_data[idx]['data']), reverse=True)
        self.update_dropdowns()
        self.select_main_flight(0)

    def select_main_flight(self, dropdown_idx):
        if dropdown_idx < 0 or dropdown_idx >= len(self.active_flight_indices):
            return
            
        # Ensure the selected flight is always at the top of the list
        if dropdown_idx != 0:
            selected_flight_idx = self.active_flight_indices.pop(dropdown_idx)
            self.active_flight_indices.insert(0, selected_flight_idx)
            self.update_dropdowns()
            
        self.flight_combo.blockSignals(True)
        self.flight_combo.setCurrentIndex(0)
        self.flight_combo.blockSignals(False)
        self.current_main_idx = 0
            
        main_flight_idx = self.active_flight_indices[0]
        flight_info = self.all_flights_data[main_flight_idx]
        flight_data = flight_info['data']
        self.current_tuning_events = flight_info.get('tuning', [])
        
        if not flight_data:
            return
            
        self.data_matrix = np.array(flight_data)
        
        # Calculate time and yaw for 3D visualizer
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

        for h in ['Roll', 'Pitch', 'YawRate', 'PID_R', 'PID_P', 'PID_Y', 'RC_Roll', 'RC_Pitch', 'RC_Yaw']:
            if h in self.headers:
                self.data_matrix[:, self.headers.index(h)] /= 100.0

        self.slider.setMaximum(len(self.data_matrix) - 1)
        self.slider.setValue(0)
        self.slider.setEnabled(True)
        self.play_btn.setEnabled(True)

        self.plot_all()
        self.slider_changed(0) 

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
        
        c_idx = 0
        for i, header in enumerate(self.headers):
            if header == 'dt_us': continue
            
            cb = QCheckBox(f"{header}: {0.0:8.2f}")
            cb.setFont(mono_font)
            
            # Match the checkbox color to the line color
            color = self.line_colors[c_idx % len(self.line_colors)]
            cb.setStyleSheet(f"""
                QCheckBox::indicator {{
                    width: 14px;
                    height: 14px;
                    border: 2px solid {color};
                    border-radius: 3px;
                }}
                QCheckBox::indicator:checked {{
                    background-color: {color};
                }}
            """)
            
            if header in checked_states:
                cb.setChecked(checked_states[header])
            elif header in ['Roll', 'Pitch']: 
                cb.setChecked(True)
            else: 
                cb.setChecked(False)
            cb.stateChanged.connect(self.update_plot_visibility)
            self.checkboxes[header] = cb
            
            placed = False
            for group_idx, (group_name, group_items) in enumerate(groups.items()):
                if header in group_items:
                    self.checkbox_layout.addWidget(cb, group_items.index(header), group_idx)
                    placed = True
                    break
            if not placed:
                self.checkbox_layout.addWidget(cb, len(groups['Other']), list(groups.keys()).index('Other'))
                groups['Other'].append(header)
                
            c_idx += 1

    def plot_all(self):
        self.fig_2d.clear()
        self.flight_plots = []
        
        if not self.active_flight_indices:
            self.canvas_2d.draw_idle()
            return
            
        num_plots = len(self.active_flight_indices)
        
        # Ensure the canvas is tall enough to scroll comfortably
        self.canvas_container.setMinimumHeight(400 * num_plots)
        self.fig_2d.set_figheight(4.0 * num_plots)
        
        main_ax = None
        
        for plot_i, flight_idx in enumerate(self.active_flight_indices):
            # Share X-axis for synchronized zooming
            ax = self.fig_2d.add_subplot(num_plots, 1, plot_i + 1, sharex=main_ax)
            if plot_i == 0:
                main_ax = ax
                
            ax.set_facecolor('#FFFFFF')
            ax.grid(True, linestyle='--', alpha=0.7)
            if plot_i == num_plots - 1:
                ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            
            flight_info = self.all_flights_data[flight_idx]
            data_matrix = np.array(flight_info['data'])
            
            if len(data_matrix) == 0: continue
            
            if 'dt_us' in self.headers:
                dt_idx = self.headers.index('dt_us')
                raw_time = data_matrix[:, dt_idx].copy()
                for i in range(1, len(raw_time)):
                    if raw_time[i] < raw_time[i-1]:
                        raw_time[i:] += 65535
                calc_time = (raw_time - raw_time[0]) / 100.0
                data_matrix[:, dt_idx] = calc_time
            else:
                calc_time = np.arange(len(data_matrix)) * 0.02
                
            for h in ['Roll', 'Pitch', 'YawRate', 'PID_R', 'PID_P', 'PID_Y', 'RC_Roll', 'RC_Pitch', 'RC_Yaw']:
                if h in self.headers:
                    data_matrix[:, self.headers.index(h)] /= 100.0
                    
            tuning_str = "Default"
            tuning_events = flight_info.get('tuning', [])
            if tuning_events:
                active_tuning = {}
                for ev in tuning_events:
                    active_tuning[ev['type']] = ev
                parts = []
                for t_type, ev in active_tuning.items():
                    if t_type == 'PID_RP': parts.append(f"RP({ev['v1']:.2f}, {ev['v2']:.2f}, {ev['v3']:.2f})")
                    elif t_type == 'PID_YAW': parts.append(f"YAW({ev['v1']:.2f}, {ev['v2']:.2f}, {ev['v3']:.2f})")
                    elif t_type == 'BIAS': parts.append(f"BIAS({ev['v1']:.2f}, {ev['v2']:.2f}, {ev['v3']:.2f})")
                    elif t_type == 'EKF': parts.append(f"EKF({ev['v1']:.5f}, {ev['v2']:.5f}, {ev['v3']:.2f})")
                tuning_str = " | ".join(parts)
            
            title = "Main Selected Flight" if plot_i == 0 else f"Flight {flight_idx + 1}"
            ax.set_title(f"{title}  [Tuning: {tuning_str}]")
            
            lines = {}
            c_idx = 0
            for i, header in enumerate(self.headers):
                if header == 'dt_us': continue
                y_data = data_matrix[:, i]
                color = self.line_colors[c_idx % len(self.line_colors)]
                line, = ax.plot(calc_time, y_data, label=header, color=color, linewidth=1.5)
                lines[header] = line
                c_idx += 1
                
            time_cursor = ax.axvline(x=calc_time[0], color='black', linestyle='-', linewidth=1.5, zorder=10)
            
            if 'RC_Throttle' in self.headers:
                t_idx = self.headers.index('RC_Throttle')
                ax.fill_between(calc_time, 0, 1, where=(data_matrix[:, t_idx] > 1300),
                                color='red', alpha=0.15, transform=ax.get_xaxis_transform(),
                                label='Hovering (>1300)')
            
            self.flight_plots.append({
                'idx': flight_idx,
                'ax': ax,
                'lines': lines,
                'time_cursor': time_cursor,
                'calc_time': calc_time
            })
            
        self.fig_2d.tight_layout()
        self.update_plot_visibility()

    def update_plot_visibility(self):
        for plot_dict in self.flight_plots:
            ax = plot_dict['ax']
            lines = plot_dict['lines']
            
            visible_y_min = float('inf')
            visible_y_max = float('-inf')
            any_visible = False
            
            for header, cb in self.checkboxes.items():
                if header in lines:
                    is_checked = cb.isChecked()
                    lines[header].set_visible(is_checked)
                    if is_checked:
                        any_visible = True
                        ydata = lines[header].get_ydata()
                        visible_y_min = min(visible_y_min, np.min(ydata))
                        visible_y_max = max(visible_y_max, np.max(ydata))
            
            if any_visible:
                margin = (visible_y_max - visible_y_min) * 0.1
                if margin == 0: margin = 1.0
                ax.set_ylim(visible_y_min - margin, visible_y_max + margin)
                
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

        HOVER_THROTTLE = 1300.0
        total_thrust = m1 + m2 + m3 + m4
        hover_total = 4.0 * HOVER_THROTTLE
        idle_total = 4000.0
        thrust_mag = max((total_thrust - idle_total) / (hover_total - idle_total), 0.0)
        
        torque_x = ((m1 + m3) - (m2 + m4)) / 1000.0
        torque_y = ((m1 + m2) - (m3 + m4)) / 1000.0
        
        torque_exaggeration = 5.0
        thrust_vector_local = np.array([torque_x * torque_exaggeration, torque_y * torque_exaggeration, thrust_mag])
        
        thrust_vector_global = R @ thrust_vector_local
        gravity_vector_global = np.array([0, 0, -1.0])
        
        net_vector = thrust_vector_global + gravity_vector_global
        net_vector *= 1.5
        
        self.thrust_line.set_data([0, net_vector[0]], [0, net_vector[1]])
        self.thrust_line.set_3d_properties([0, net_vector[2]])

        def get_rotor_color(val, cw):
            norm = min(max((val - 1000) / 1000.0, 0.0), 1.0)
            if cw:
                return mcolors.to_rgba((0, norm, norm * 0.5 + 0.5))
            else:
                return mcolors.to_rgba((norm * 0.5 + 0.5, norm, 0))

        fc = self.face_colors.copy()
        start = self.rotor_start_face_idx
        
        c_m2 = get_rotor_color(m2, cw=False)
        for i in range(6): fc[start + 0 + i] = c_m2
        
        c_m4 = get_rotor_color(m4, cw=True)
        for i in range(6): fc[start + 6 + i] = c_m4
            
        c_m1 = get_rotor_color(m1, cw=True)
        for i in range(6): fc[start + 12 + i] = c_m1
            
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
        
        current_idx = self.slider.value()
        target_time = self.calculated_time[current_idx] + (self.playback_speed_ms / 1000.0)
        
        next_idx = current_idx
        while next_idx < len(self.calculated_time) and self.calculated_time[next_idx] < target_time:
            next_idx += 1
            
        if next_idx >= self.slider.maximum():
            self.slider.setValue(0)
        else:
            self.slider.setValue(next_idx)

    def update_x_axis(self):
        if not self.flight_plots: return
        
        if self.show_all_cb.isChecked():
            max_t = 0
            for p in self.flight_plots:
                if len(p['calc_time']) > 0:
                    max_t = max(max_t, p['calc_time'][-1])
            if max_t > 0:
                self.flight_plots[0]['ax'].set_xlim(0, max_t)
        else:
            t = self.calculated_time[self.slider.value()]
            window = self.window_spin.value()
            self.flight_plots[0]['ax'].set_xlim(max(0, t - window/2), t + window/2)
            
        self.canvas_2d.draw_idle()

    def slider_changed(self, value):
        if self.data_matrix is None or value >= len(self.data_matrix):
            return
            
        t = self.calculated_time[value]
        self.time_lbl.setText(f"Time: {t:.2f} s")
        
        for plot_dict in self.flight_plots:
            plot_dict['time_cursor'].set_xdata([t, t])
        
        self.update_x_axis()
        
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
