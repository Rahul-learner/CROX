import serial
import time
import math
import numpy as np
import threading
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import CheckButtons
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# --- Shared Data for GUI and Thread ---
running = True
current_dt_s = 0.0

# Current calculated angles (Assuming incoming Roll/Pitch are in degrees, and Yaw_Rate is deg/s)
current_roll = 0.0
current_pitch = 0.0
current_yaw = 0.0

# Deques for fast scrolling plot
max_points = 250
plot_times = deque(maxlen=max_points)
plot_rolls = deque(maxlen=max_points)
plot_pitches = deque(maxlen=max_points)
plot_yaws = deque(maxlen=max_points)

def serial_worker(port='/dev/ttyACM0', baud=115200):
    global running
    global current_dt_s
    global current_roll, current_pitch, current_yaw

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to Pico on {port}. Waiting for telemetry strings...")
        mock_mode = False
    except serial.SerialException as e:
        print(f"Failed to connect on {port}. Entering MOCK/SIMULATION mode.")
        mock_mode = True

    start_time = time.time()
    t = 0.0

    while running:
        if mock_mode:
            # Generate fake telemetry strings to test the parser
            time.sleep(0.02)
            t += 0.02
            f_roll = math.sin(t) * 10.0
            f_pitch = math.cos(t * 0.8) * 10.0
            f_yaw_rate = math.sin(t * 0.5) * 5.0
            f_dt, f_dt_s = 0.02, 0.02

            line = f"Roll: {f_roll:.6f}, Pitch: {f_pitch:.6f}, Yaw_Rate: {f_yaw_rate:.6f}, dt: {f_dt:.6f}, dt_s: {f_dt_s:.6f}"
        else:
            # Read line from Serial Port
            if ser.in_waiting == 0:
                time.sleep(0.001)
                continue
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception:
                continue

        # Parse the string: "Roll: -0.914490, Pitch: -1.658951, Yaw_Rate: 0.034249, dt: 0.001981, dt_s: 0.001458"
        if line.startswith("Roll:"):
            try:
                parts = line.split(',')
                # Extract floats by splitting each part by ':' and grabbing the second half
                r_val = float(parts[0].split(':')[1])
                p_val = float(parts[1].split(':')[1])
                yr_val = float(parts[2].split(':')[1])
                dt_val = float(parts[3].split(':')[1])
                dts_val = float(parts[4].split(':')[1])

                # Update Globals
                current_roll = r_val
                current_pitch = p_val
                current_dt_s = dts_val

                # Integrate Yaw Rate to get Yaw Angle
                current_yaw += yr_val * dts_val

                # Wrap Yaw to -180 to 180 degrees to keep it clean
                if current_yaw > 180.0: current_yaw -= 360.0
                if current_yaw < -180.0: current_yaw += 360.0

                # Append to plotting arrays
                curr_time = time.time() - start_time
                plot_times.append(curr_time)
                plot_rolls.append(current_roll)
                plot_pitches.append(current_pitch)
                plot_yaws.append(current_yaw)

            except (IndexError, ValueError) as e:
                # Silently ignore corrupted serial lines
                pass

    if not mock_mode:
        ser.close()

# --- GUI & Plotting Main Loop ---
def run_telemetry_viewer():
    global running

    # 1. Start Serial Worker Thread
    thread = threading.Thread(target=serial_worker, daemon=True)
    thread.start()

    # 2. Setup Matplotlib Plot
    fig = plt.figure(figsize=(12, 7))
    gs = fig.add_gridspec(1, 2, width_ratios=[2, 1.2])

    # --- 2D Graph Setup ---
    ax_2d = fig.add_subplot(gs[0, 0])
    ax_2d.set_title('Real-Time 6-DoF EKF Angles')
    ax_2d.set_ylabel('Angle (Degrees)')
    ax_2d.set_xlabel('Time (s)')

    line_r, = ax_2d.plot([], [], label='Roll (X)', color='red', linewidth=2)
    line_p, = ax_2d.plot([], [], label='Pitch (Y)', color='green', linewidth=2)
    line_y, = ax_2d.plot([], [], label='Yaw (Z)', color='blue', linewidth=2)

    ax_2d.legend(loc='upper right')
    ax_2d.grid(True)

    # --- 3D Orientation Setup ---
    ax_3d = fig.add_subplot(gs[0, 1], projection='3d')
    ax_3d.set_title('3D Orientation (Drone)')
    ax_3d.set_xlim([-2, 2])
    ax_3d.set_ylim([-2, 2])
    ax_3d.set_zlim([-2, 2])
    ax_3d.set_xlabel('X (Forward)')
    ax_3d.set_ylabel('Y (Left/Right)')
    ax_3d.set_zlabel('Z (Up/Down)')
    ax_3d.set_box_aspect((1, 1, 1))

    # Define 3D Drone Box
    l, w, h = 1.5, 1.0, 0.2
    V = np.array([
        [-l, -w, -h], [l, -w, -h], [l, w, -h], [-l, w, -h],
        [-l, -w,  h], [l, -w,  h], [l, w,  h], [-l, w,  h]
    ]).T

    faces = [
        [0, 1, 2, 3], # Bottom
        [4, 5, 6, 7], # Top
        [1, 5, 6, 2], # Front
        [0, 4, 7, 3], # Back
        [2, 6, 7, 3], # Left
        [0, 1, 5, 4]  # Right
    ]

    face_colors = ['#444444', '#00FFFF', '#FF0000', '#0000FF', '#00FF00', '#FFA500']
    poly3d = Poly3DCollection([], facecolors=face_colors, edgecolors='black', alpha=0.9)
    ax_3d.add_collection3d(poly3d)

    axis_line_x, = ax_3d.plot([], [], [], color='red', linewidth=3, label='X Axis')
    axis_line_y, = ax_3d.plot([], [], [], color='green', linewidth=3, label='Y Axis')
    axis_line_z, = ax_3d.plot([], [], [], color='blue', linewidth=3, label='Z Axis')

    # Live data text at the bottom
    info_text = fig.text(0.5, 0.05, '', ha='center', va='bottom', fontsize=12, fontweight='bold', family='monospace')

    # Adjust layout to make room for checkboxes
    plt.subplots_adjust(bottom=0.20, right=0.85, wspace=0.2)

    # Setup Checkboxes
    ax_check = plt.axes([0.86, 0.4, 0.12, 0.20])
    check = CheckButtons(
        ax_check,
        ['Roll (X)', 'Pitch (Y)', 'Yaw (Z)', 'Auto-Zoom', 'Apply Yaw'],
        [True, True, True, True, True]
    )

    state = {'auto_scroll': True, 'apply_yaw_3d': True}

    def toggle_lines(label):
        if label == 'Roll (X)': line_r.set_visible(not line_r.get_visible())
        elif label == 'Pitch (Y)': line_p.set_visible(not line_p.get_visible())
        elif label == 'Yaw (Z)': line_y.set_visible(not line_y.get_visible())
        elif label == 'Auto-Zoom': state['auto_scroll'] = not state['auto_scroll']
        elif label == 'Apply Yaw': state['apply_yaw_3d'] = not state['apply_yaw_3d']

    check.on_clicked(toggle_lines)

    # Convert Euler angles (degrees) to Rotation Matrix
    def euler_to_rotmat(r_deg, p_deg, y_deg):
        cx, sx = np.cos(np.radians(r_deg)), np.sin(np.radians(r_deg))
        cy, sy = np.cos(np.radians(p_deg)), np.sin(np.radians(p_deg))
        cz, sz = np.cos(np.radians(y_deg)), np.sin(np.radians(y_deg))

        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

        return Rz @ Ry @ Rx

    # 4. Animation Update Function
    def animate(i):
        if len(plot_times) > 0:
            # Update Live Text
            info_text.set_text(f"Roll: {current_roll:7.2f}° | Pitch: {current_pitch:7.2f}° | Yaw: {current_yaw:7.2f}° | dt: {current_dt_s:.5f}s")

            # Update 2D Graph
            line_r.set_data(plot_times, plot_rolls)
            line_p.set_data(plot_times, plot_pitches)
            line_y.set_data(plot_times, plot_yaws)

            if state['auto_scroll']:
                if len(plot_times) > 1:
                    ax_2d.set_xlim(plot_times[0], plot_times[-1])

                visible_y = []
                if line_r.get_visible(): visible_y.extend(plot_rolls)
                if line_p.get_visible(): visible_y.extend(plot_pitches)
                if line_y.get_visible(): visible_y.extend(plot_yaws)

                if visible_y:
                    min_y, max_y = min(visible_y), max(visible_y)
                    margin = max(0.5, (max_y - min_y) * 0.1)
                    ax_2d.set_ylim(min_y - margin, max_y + margin)

            # Update 3D Object
            if state['apply_yaw_3d']:
                R = euler_to_rotmat(current_roll, current_pitch, current_yaw)
            else:
                R = euler_to_rotmat(current_roll, current_pitch, 0.0) # Force Yaw to 0

            V_rot = R @ V

            rotated_faces = []
            for face in faces:
                rotated_faces.append([V_rot[:, idx] for idx in face])
            poly3d.set_verts(rotated_faces)

            # Update axis arrows
            x_axis = R @ np.array([2.0, 0, 0])
            axis_line_x.set_data([0, x_axis[0]], [0, x_axis[1]])
            axis_line_x.set_3d_properties([0, x_axis[2]])

            y_axis = R @ np.array([0, 2.0, 0])
            axis_line_y.set_data([0, y_axis[0]], [0, y_axis[1]])
            axis_line_y.set_3d_properties([0, y_axis[2]])

            z_axis = R @ np.array([0, 0, 2.0])
            axis_line_z.set_data([0, z_axis[0]], [0, z_axis[1]])
            axis_line_z.set_3d_properties([0, z_axis[2]])

        return line_r, line_p, line_y

    ani = animation.FuncAnimation(fig, animate, interval=20, blit=False, cache_frame_data=False)

    def on_close(event):
        global running
        running = False
        print("\nViewer stopped.")

    fig.canvas.mpl_connect('close_event', on_close)
    plt.show()

if __name__ == "__main__":
    run_telemetry_viewer()
