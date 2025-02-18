import serial
import time
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # needed for 3D plotting

# ======== User Settings =========
# Predefined node positions (set these values before running)
nodes = [
    (0, 0, 0),
    (1, 1, 0),
    (0, 1, 0),
    (1.0, 0.6, 1.0)
]

SERIAL_PORT = 'COM6'
BAUD_RATE = 115200
# =================================

# Set up the serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)             # Allow time for the connection to establish
ser.reset_input_buffer()  # Clear any initial incomplete data

# Create the figure and 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the 4 static nodes as blue triangles
nodes_x = [n[0] for n in nodes]
nodes_y = [n[1] for n in nodes]
nodes_z = [n[2] for n in nodes]
ax.scatter(nodes_x, nodes_y, nodes_z, c='b', marker='^', s=100, label='Nodes')

# Create the sphere marker (red circle) at an initial position.
# We'll update its position in our loop.
sphere_marker = ax.scatter([0], [0], [0], c='r', marker='o', s=200, label='Sphere')

# Set up the axes limits and labels
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_zlim(0, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

# This variable will hold the latest valid position read from serial.
latest_position = None

def read_serial():
    global latest_position
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) != 3:
                print("Skipping invalid line:", line)
                continue
            x, y, z = map(float, parts)
            # Update the latest position (overwrite any previous value)
            latest_position = (x, y, z)
        except Exception as e:
            print("Error reading/parsing line:", e)
        # A short sleep prevents this thread from hogging the CPU.
        time.sleep(0.005)

# Start the serial reading in a separate thread.
serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

# Enable interactive mode so we can update the plot in a loop.
plt.ion()
plt.show()

# Main loop: update the sphere's position whenever new data arrives.
try:
    while True:
        if latest_position is not None:
            x, y, z = latest_position
            # Update the sphere's position. For 3D scatter, update _offsets3d.
            sphere_marker._offsets3d = ([x], [y], [z])
            # Reset latest_position to wait for the next update.
            latest_position = None
            # Redraw the canvas.
            fig.canvas.draw_idle()
        # A short pause to allow the GUI event loop to process events.
        plt.pause(0.01)
except KeyboardInterrupt:
    print("Exiting...")
