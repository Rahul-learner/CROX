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

# Queue to communicate between serial thread and GUI thread
data_queue = queue.Queue()

# We will keep a history of the last N samples for the 2D plot
HISTORY_SIZE = 200
history_roll = []
history_pitch = []
history_yaw = []

# Regex pattern to match the main loop print:
# "Roll: %.2f, Pitch: %.2f, Yaw: %.2f, RC_Roll: ..."
# We use re.search so it matches anywhere in the line
pattern = re.compile(r"Roll:\s*([-0-9.]+),\s*Pitch:\s*([-0-9.]+),\s*Yaw:\s*([-0-9.]+)")

def serial_reader(port, baudrate):
    """
    Background thread that reads from the serial port.
    If it matches the telemetry pattern, it extracts the data.
    Otherwise, it prints the line to the console.
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baud.")
    except Exception as e:
        print(f"Failed to connect to serial port: {e}")
        sys.exit(1)

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if not line:
                continue
                
            match = pattern.search(line)
            if match:
                # We found telemetry data
                r = float(match.group(1))
                p = float(match.group(2))
                y = float(match.group(3))
                data_queue.put((r, p, y))
            else:
                # It's a regular DEBUG_PRINT or other output, print it to console
                print(line)
        except Exception as e:
            print(f"Serial read error: {e}")
            break

def get_rotation_matrix(roll, pitch, yaw):
    """
    Returns a 3D rotation matrix given roll, pitch, and yaw in degrees.
    Assuming:
    Roll -> Rotation around X axis
    Pitch -> Rotation around Y axis
    Yaw -> Rotation around Z axis
    """
    r = np.radians(roll)
    p = np.radians(pitch)
    y = np.radians(yaw)
    
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(r), -np.sin(r)],
        [0, np.sin(r), np.cos(r)]
    ])
    
    Ry = np.array([
        [np.cos(p), 0, np.sin(p)],
        [0, 1, 0],
        [-np.sin(p), 0, np.cos(p)]
    ])
    
    Rz = np.array([
        [np.cos(y), -np.sin(y), 0],
        [np.sin(y), np.cos(y), 0],
        [0, 0, 1]
    ])
    
    return Rz @ Ry @ Rx

# Setup Matplotlib Figure
fig = plt.figure(figsize=(10, 8))
fig.canvas.manager.set_window_title('Drone Telemetry & 3D Visualizer')

# --- Top Subplot: 3D Drone Visualizer ---
ax_3d = fig.add_subplot(211, projection='3d')

# Define drone geometry (X forward, Y left, Z up)
# Arm 1: Front-Left (1, 1, 0) to Rear-Right (-1, -1, 0)
base_arm1 = np.array([
    [ 1, -1],
    [ 1, -1],
    [ 0,  0]
])
# Arm 2: Front-Right (1, -1, 0) to Rear-Left (-1, 1, 0)
base_arm2 = np.array([
    [ 1, -1],
    [-1,  1],
    [ 0,  0]
])
# Front indicator line (from center to forward X)
base_front = np.array([
    [0, 1.5],
    [0, 0],
    [0, 0]
])

line_arm1, = ax_3d.plot([], [], [], 'b-', linewidth=3, label="Arm")
line_arm2, = ax_3d.plot([], [], [], 'b-', linewidth=3)
line_front, = ax_3d.plot([], [], [], 'r-', linewidth=4, label="Front")

ax_3d.set_xlim([-2, 2])
ax_3d.set_ylim([-2, 2])
ax_3d.set_zlim([-2, 2])
ax_3d.set_xlabel('X')
ax_3d.set_ylabel('Y')
ax_3d.set_zlabel('Z')
ax_3d.set_title("3D Drone Orientation")
ax_3d.legend()

# --- Bottom Subplot: 2D Rolling Plot ---
ax_2d = fig.add_subplot(212)
line_r, = ax_2d.plot([], [], 'r-', label='Roll')
line_p, = ax_2d.plot([], [], 'g-', label='Pitch')
line_y, = ax_2d.plot([], [], 'b-', label='Yaw')

ax_2d.set_xlim(0, HISTORY_SIZE)
ax_2d.set_ylim(-180, 180)
ax_2d.set_title("Telemetry (Degrees)")
ax_2d.set_ylabel("Angle")
ax_2d.set_xlabel("Samples")
ax_2d.legend(loc='upper right')
ax_2d.grid(True)

def update(frame):
    # Drain the queue to get the latest data
    latest_data = None
    while not data_queue.empty():
        try:
            latest_data = data_queue.get_nowait()
        except queue.Empty:
            break
            
    if latest_data is not None:
        r, p, y = latest_data
        
        # Update 2D history
        history_roll.append(r)
        history_pitch.append(p)
        history_yaw.append(y)
        
        if len(history_roll) > HISTORY_SIZE:
            history_roll.pop(0)
            history_pitch.pop(0)
            history_yaw.pop(0)
            
        # Update 2D lines
        x_data = range(len(history_roll))
        line_r.set_data(x_data, history_roll)
        line_p.set_data(x_data, history_pitch)
        line_y.set_data(x_data, history_yaw)
        
        # Adjust Y limits dynamically if yaw wraps around or goes wild
        min_val = min(min(history_roll), min(history_pitch), min(history_yaw))
        max_val = max(max(history_roll), max(history_pitch), max(history_yaw))
        if min_val < -180 or max_val > 180:
            # Add 20 deg padding
            ax_2d.set_ylim(min_val - 20, max_val + 20)
        else:
            ax_2d.set_ylim(-180, 180)
            
        # Update 3D Visualizer
        # Calculate rotation matrix
        R = get_rotation_matrix(r, p, y)
        
        # Rotate the base drone arms
        rot_arm1 = R @ base_arm1
        rot_arm2 = R @ base_arm2
        rot_front = R @ base_front
        
        # Update 3D lines
        line_arm1.set_data(rot_arm1[0], rot_arm1[1])
        line_arm1.set_3d_properties(rot_arm1[2])
        
        line_arm2.set_data(rot_arm2[0], rot_arm2[1])
        line_arm2.set_3d_properties(rot_arm2[2])
        
        line_front.set_data(rot_front[0], rot_front[1])
        line_front.set_3d_properties(rot_front[2])
        
    return line_arm1, line_arm2, line_front, line_r, line_p, line_y

def main():
    parser = argparse.ArgumentParser(description="Drone Serial Debug Monitor & 3D Visualizer")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Serial port to connect to")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    # Start serial reading thread (daemon=True so it exits when main thread exits)
    thread = threading.Thread(target=serial_reader, args=(args.port, args.baud), daemon=True)
    thread.start()

    print("Starting visualizer... Close the window to exit.")
    
    # Run animation
    # interval=50 means 20 fps, which is standard for matplotib to remain responsive
    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    
    # This will block until the window is closed
    plt.show()

if __name__ == "__main__":
    main()
