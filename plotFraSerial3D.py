import serial
import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # needed for 3D plotting
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.widgets import Button
from itertools import combinations

# ======== User Settings =========
# Predefined node positions
nodes = [
    (0.5, 0.3, 0.45),
    (0.2, 0.47, 0.45),
    (0.2, 0.13, 0.45),
    (0.33, 0.33, 0.12)
]




SERIAL_PORT = 'COM5'
BAUD_RATE = 115200

# Parameters for the net object:
NET_BOTTOM = (0.3, 0.3, 0.1)  # bottom point of net
NET_RADIUS = 0.2              # radius of half-sphere (net bowl)
NET_HEIGHT = 0.5              # total height (if NET_HEIGHT > NET_BOTTOM_z+NET_RADIUS, add a cylinder)

# --- Resolution parameters for the net (lower = less CPU load) ---
HALF_SPHERE_RES_PHI   = 15   # for the half-sphere
HALF_SPHERE_RES_THETA = 15
CYLINDER_RES_Z        = 10   # for the cylinder
CYLINDER_RES_THETA    = 15
# =================================

# Global serial variable; initially no connection.
ser = None

# Global variable to hold the latest serial data (x, y, z)
latest_position = None

# Create the figure and 3D axes.
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the 4 static nodes as blue triangles.
nodes_x = [n[0] for n in nodes]
nodes_y = [n[1] for n in nodes]
nodes_z = [n[2] for n in nodes]
ax.scatter(nodes_x, nodes_y, nodes_z, c='b', marker='^', s=100, label='Nodes')

# Create the sphere marker (red circle) at an initial position.
sphere_marker = ax.scatter([0], [0], [0], c='r', marker='o', s=200, label='Sphere')

# Set up the axes limits and labels.
ax.set_xlim(0.05, 0.55)
ax.set_ylim(0.05, 0.55)
ax.set_zlim(0, 0.6)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

# Create a text box in the top left (using text2D) for connection status and last point.
status_text = ax.text2D(0.01, 0.98,
                        "Status: Disconnected\nLast point: N/A",
                        transform=ax.transAxes,
                        fontsize=10,
                        verticalalignment='top',
                        bbox=dict(facecolor='white', alpha=0.8, edgecolor='black'))

def update_status(text):
    status_text.set_text(text)
    fig.canvas.draw_idle()

def connect_com(event):
    """Attempt to connect to the COM port and start reading data."""
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Allow time for connection
        ser.reset_input_buffer()
        update_status("Status: Connected\nLast point: N/A")
        threading.Thread(target=read_serial, daemon=True).start()
    except Exception as e:
        update_status(f"Status: Connection failed: {e}\nLast point: N/A")

# --- Button: Connect COM ---
button_ax_connect = fig.add_axes([0.75, 0.6, 0.2, 0.075])
connect_button = Button(button_ax_connect, 'Connect COM')
connect_button.on_clicked(connect_com)

# --- Toggle Volume ---
volume_poly = None
volume_edges = []  # List to store edge line artists
volume_visible = False

def toggle_volume(event):
    """Toggle display of a transparent volume connecting the nodes and its perimeter edges."""
    global volume_poly, volume_edges, volume_visible

    if not volume_visible:
        faces = [
            [nodes[0], nodes[1], nodes[2]],
            [nodes[0], nodes[1], nodes[3]],
            [nodes[0], nodes[2], nodes[3]],
            [nodes[1], nodes[2], nodes[3]]
        ]
        volume_poly = Poly3DCollection(faces, alpha=0.3, facecolors='cyan')
        ax.add_collection3d(volume_poly)
        for i, j in combinations(range(len(nodes)), 2):
            line, = ax.plot(
                [nodes[i][0], nodes[j][0]],
                [nodes[i][1], nodes[j][1]],
                [nodes[i][2], nodes[j][2]],
                color='k', linewidth=1, alpha=0.5)
            volume_edges.append(line)
        volume_visible = True
    else:
        if volume_poly is not None:
            volume_poly.remove()
        for line in volume_edges:
            line.remove()
        volume_edges.clear()
        volume_visible = False
    fig.canvas.draw_idle()

button_ax_vol = fig.add_axes([0.75, 0.05, 0.2, 0.075])
toggle_vol_button = Button(button_ax_vol, 'Toggle Volume')
toggle_vol_button.on_clicked(toggle_volume)

# --- Toggle Net ---
net_half_sphere = None
net_cylinder = None
net_visible = False

def toggle_net(event):
    """
    Toggle display of a net object composed of:
      1. A half-sphere (only the bottom half) with radius NET_RADIUS (its bottom touches NET_BOTTOM).
      2. If NET_HEIGHT > (NET_BOTTOM_z + NET_RADIUS), attach a cylinder of constant radius up to NET_HEIGHT.
    """
    global net_half_sphere, net_cylinder, net_visible

    if net_visible:
        if net_half_sphere is not None:
            net_half_sphere.remove()
            net_half_sphere = None
        if net_cylinder is not None:
            net_cylinder.remove()
            net_cylinder = None
        net_visible = False
    else:
        N_x, N_y, N_z = NET_BOTTOM
        r = NET_RADIUS
        # For a half-sphere whose bottom touches NET_BOTTOM, place its center at (N_x, N_y, N_z + r).
        center = (N_x, N_y, N_z + r)
        phi = np.linspace(np.pi/2, np.pi, HALF_SPHERE_RES_PHI)
        theta = np.linspace(0, 2*np.pi, HALF_SPHERE_RES_THETA)
        phi, theta = np.meshgrid(phi, theta)
        Xs = center[0] + r * np.sin(phi) * np.cos(theta)
        Ys = center[1] + r * np.sin(phi) * np.sin(theta)
        Zs = center[2] + r * np.cos(phi)
        net_half_sphere = ax.plot_surface(Xs, Ys, Zs, color='green', alpha=0.7,
                                          rstride=1, cstride=1, edgecolor='none')
        z_equator = N_z + r  # The equator (top) of the half-sphere.
        if NET_HEIGHT > z_equator:
            z_vals = np.linspace(z_equator, NET_HEIGHT, CYLINDER_RES_Z)
            theta_vals = np.linspace(0, 2*np.pi, CYLINDER_RES_THETA)
            theta_vals, z_vals = np.meshgrid(theta_vals, z_vals)
            Xc = N_x + r * np.cos(theta_vals)
            Yc = N_y + r * np.sin(theta_vals)
            Zc = z_vals
            net_cylinder = ax.plot_surface(Xc, Yc, Zc, color='green', alpha=0.7,
                                           rstride=1, cstride=1, edgecolor='none')
        net_visible = True
    fig.canvas.draw_idle()

button_ax_net = fig.add_axes([0.75, 0.35, 0.2, 0.075])
toggle_net_button = Button(button_ax_net, 'Toggle Net')
toggle_net_button.on_clicked(toggle_net)

def read_serial():
    """Continuously read serial data and update latest_position and the status text."""
    global latest_position
    while True:
        if ser is not None:
            try:
                line = ser.readline().decode('utf-8').strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) != 3:
                    continue
                x, y, z = map(float, parts)
                latest_position = (x, y, z)
                update_status(f"Status: Connected\nLast point: ({x:.2f}, {y:.2f}, {z:.2f})")
            except Exception as e:
                update_status(f"Status: Error: {e}\nLast point: N/A")
        time.sleep(0.005)

plt.ion()
plt.show()

# Main loop: update sphere marker with latest point.
try:
    while True:
        if latest_position is not None:
            x, y, z = latest_position
            sphere_marker._offsets3d = ([x], [y], [z])
            latest_position = None
            fig.canvas.draw_idle()
        plt.pause(0.01)
except KeyboardInterrupt:
    print("Exiting...")
