import serial
import threading
import queue
import re
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import TextBox, Button, CheckButtons

# Queue to communicate between serial thread and GUI thread
data_queue = queue.Queue()
ser_port = None

# We will keep a history of the last N samples for the 2D plot
HISTORY_SIZE = 200
history = {}
lines_2d = {}

# Regex pattern to match Key: Value pairs (e.g. "Roll: 12.34" or "PID_RP: -0.05")
pattern = re.compile(r"([a-zA-Z0-9_]+):\s*([-0-9.]+)")

def serial_reader(port, baudrate):
    global ser_port
    try:
        ser_port = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baud.")
    except Exception as e:
        print(f"Failed to connect to serial port: {e}")
        sys.exit(1)

    while True:
        try:
            line = ser_port.readline().decode('utf-8', errors='replace').strip()
            if not line:
                continue
                
            matches = pattern.findall(line)
            if matches:
                # We found telemetry data, parse all variables into a dictionary
                data_dict = {k: float(v) for k, v in matches}
                data_queue.put(data_dict)
            else:
                # Regular output (like ACK prints)
                print(line)
        except Exception as e:
            print(f"Serial read error: {e}")
            break

def get_rotation_matrix(roll, pitch, yaw):
    r = np.radians(roll)
    p = np.radians(pitch)
    y = np.radians(yaw)
    
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

# Setup Matplotlib Figure
fig = plt.figure(figsize=(14, 8))
fig.canvas.manager.set_window_title('Drone Telemetry & Tuning Panel')

# Adjust layout to make room for tuning panel AND checkboxes on the left
plt.subplots_adjust(left=0.3)

# --- 3D Subplot ---
ax_3d = fig.add_subplot(211, projection='3d')
base_arm1 = np.array([[ 1, -1], [ 1, -1], [ 0,  0]])
base_arm2 = np.array([[ 1, -1], [-1,  1], [ 0,  0]])
base_front = np.array([[0, 1.5], [0, 0], [0, 0]])

line_arm1, = ax_3d.plot([], [], [], 'b-', linewidth=3, label="Arm")
line_arm2, = ax_3d.plot([], [], [], 'b-', linewidth=3)
line_front, = ax_3d.plot([], [], [], 'r-', linewidth=4, label="Front")

ax_3d.set_xlim([-2, 2])
ax_3d.set_ylim([-2, 2])
ax_3d.set_zlim([-2, 2])
ax_3d.set_xlabel('X')
ax_3d.set_ylabel('Y')
ax_3d.set_zlabel('Z')
ax_3d.set_title("3D Orientation")

# --- 2D Subplot ---
ax_2d = fig.add_subplot(212)
ax_2d.set_xlim(0, HISTORY_SIZE)
ax_2d.set_ylim(-180, 180)
ax_2d.set_title("Telemetry (Degrees)")
ax_2d.grid(True)

# ==============================================================================
# TUNING PANEL (Left Side)
# ==============================================================================
h = 0.04
w = 0.08
x_box = 0.18  # Moved further right to make room for checkboxes

# Roll/Pitch PID
ax_rp_p = plt.axes([x_box, 0.85, w, h])
tb_rp_p = TextBox(ax_rp_p, 'RP_P: ', initial='1.75')
ax_rp_i = plt.axes([x_box, 0.80, w, h])
tb_rp_i = TextBox(ax_rp_i, 'RP_I: ', initial='0.05')
ax_rp_d = plt.axes([x_box, 0.75, w, h])
tb_rp_d = TextBox(ax_rp_d, 'RP_D: ', initial='0.47')

ax_send_rp = plt.axes([x_box, 0.70, w, h])
btn_send_rp = Button(ax_send_rp, 'Send RP')

# Yaw PID
ax_y_p = plt.axes([x_box, 0.60, w, h])
tb_y_p = TextBox(ax_y_p, 'YAW_P: ', initial='1.3')
ax_y_i = plt.axes([x_box, 0.55, w, h])
tb_y_i = TextBox(ax_y_i, 'YAW_I: ', initial='0.0')
ax_y_d = plt.axes([x_box, 0.50, w, h])
tb_y_d = TextBox(ax_y_d, 'YAW_D: ', initial='0.0')

ax_send_yaw = plt.axes([x_box, 0.45, w, h])
btn_send_yaw = Button(ax_send_yaw, 'Send YAW')

# Bias
ax_b_r = plt.axes([x_box, 0.35, w, h])
tb_b_r = TextBox(ax_b_r, 'BIAS_R: ', initial='-3.95')
ax_b_p = plt.axes([x_box, 0.30, w, h])
tb_b_p = TextBox(ax_b_p, 'BIAS_P: ', initial='1.87')
ax_b_y = plt.axes([x_box, 0.25, w, h])
tb_b_y = TextBox(ax_b_y, 'BIAS_Y: ', initial='0.0')

ax_send_bias = plt.axes([x_box, 0.20, w, h])
btn_send_bias = Button(ax_send_bias, 'Send BIAS')

# Callbacks for sending data
def send_rp(event):
    if ser_port:
        cmd = f"PID_RP,{tb_rp_p.text},{tb_rp_i.text},{tb_rp_d.text}\n"
        ser_port.write(cmd.encode())
        print(f"Sent: {cmd.strip()}")

def send_yaw(event):
    if ser_port:
        cmd = f"PID_YAW,{tb_y_p.text},{tb_y_i.text},{tb_y_d.text}\n"
        ser_port.write(cmd.encode())
        print(f"Sent: {cmd.strip()}")

def send_bias(event):
    if ser_port:
        cmd = f"BIAS,{tb_b_r.text},{tb_b_p.text},{tb_b_y.text}\n"
        ser_port.write(cmd.encode())
        print(f"Sent: {cmd.strip()}")

btn_send_rp.on_clicked(send_rp)
btn_send_yaw.on_clicked(send_yaw)
btn_send_bias.on_clicked(send_bias)

# ==============================================================================
# DYNAMIC CHECKBOXES (Far Left Side)
# ==============================================================================
ax_checks = plt.axes([0.02, 0.2, 0.1, 0.6])
ax_checks.set_axis_off()
ax_checks.set_title("Plot Variables")
check_buttons = None

def toggle_line(label):
    if label in lines_2d:
        lines_2d[label].set_visible(not lines_2d[label].get_visible())
        fig.canvas.draw_idle()


# ==============================================================================
# ANIMATION LOOP
# ==============================================================================
def update(frame):
    global check_buttons
    
    # Drain the queue to get the latest data
    latest_data = None
    while not data_queue.empty():
        try:
            latest_data = data_queue.get_nowait()
        except queue.Empty:
            break
            
    if latest_data is not None:
        
        # 1. Initialize variables dynamically on first receive
        if not history:
            labels = list(latest_data.keys())
            for k in labels:
                history[k] = []
                # Create a plot line for every variable
                line, = ax_2d.plot([], [], label=k)
                lines_2d[k] = line
            
            # Setup CheckButtons
            # By default, only show Roll, Pitch, Yaw
            visibility = [True if k in ['Roll', 'Pitch', 'Yaw'] else False for k in labels]
            for k, vis in zip(labels, visibility):
                lines_2d[k].set_visible(vis)
                
            check_buttons = CheckButtons(ax_checks, labels, visibility)
            check_buttons.on_clicked(toggle_line)
            
            # Place legend outside
            ax_2d.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0))
        
        # 2. Append new data
        for k, v in latest_data.items():
            if k in history:
                history[k].append(v)
                if len(history[k]) > HISTORY_SIZE:
                    history[k].pop(0)
            
        # 3. Update 2D lines
        x_data = range(len(history[list(history.keys())[0]]))
        
        min_val = float('inf')
        max_val = float('-inf')
        any_visible = False
        
        for k, line in lines_2d.items():
            line.set_data(x_data, history[k])
            
            # Only scale Y-axis for visible lines
            if line.get_visible() and history[k]:
                any_visible = True
                min_val = min(min_val, min(history[k]))
                max_val = max(max_val, max(history[k]))
                
        # Adjust Y limits dynamically based on active lines
        if any_visible and min_val != float('inf'):
            margin = max(20.0, (max_val - min_val) * 0.1)
            ax_2d.set_ylim(min_val - margin, max_val + margin)
            
        # 4. Update 3D Visualizer
        r = latest_data.get('Roll', 0.0)
        p = latest_data.get('Pitch', 0.0)
        y = latest_data.get('Yaw', 0.0)
        
        R = get_rotation_matrix(r, p, y)
        rot_arm1 = R @ base_arm1
        rot_arm2 = R @ base_arm2
        rot_front = R @ base_front
        
        line_arm1.set_data(rot_arm1[0], rot_arm1[1])
        line_arm1.set_3d_properties(rot_arm1[2])
        
        line_arm2.set_data(rot_arm2[0], rot_arm2[1])
        line_arm2.set_3d_properties(rot_arm2[2])
        
        line_front.set_data(rot_front[0], rot_front[1])
        line_front.set_3d_properties(rot_front[2])
        
    return line_arm1, line_arm2, line_front, *lines_2d.values()

def main():
    parser = argparse.ArgumentParser(description="Drone & Ground Station Debug GUI")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Serial port to connect to")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    # Start serial reading thread (daemon=True so it exits when main thread exits)
    thread = threading.Thread(target=serial_reader, args=(args.port, args.baud), daemon=True)
    thread.start()

    print("Starting GUI... Close the window to exit.")
    
    # Run animation
    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    
    # Attempt to open fullscreen
    try:
        fig.canvas.manager.full_screen_toggle()
    except Exception:
        pass # Fullscreen toggle depends on the matplotlib backend
    
    plt.show()

if __name__ == "__main__":
    main()
