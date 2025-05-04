import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def simulate_tdoa(anchors, true_position, c=343.0, noise_std=1e-5):
    """
    Simulate arrival times at each anchor for a source emitting a ping.
    
    Parameters:
        anchors (ndarray): (N, 3) array with positions of the anchors.
        true_position (ndarray): (3,) array with the true source position.
        c (float): Speed of sound in m/s.
        noise_std (float): Standard deviation of Gaussian noise added to arrival times.
    
    Returns:
        arrival_times_noisy (ndarray): Simulated arrival times (seconds) with noise.
        distances (ndarray): True distances from the source to each anchor.
    """
    distances = np.linalg.norm(anchors - true_position, axis=1)
    arrival_times = distances / c
    arrival_times_noisy = arrival_times + np.random.normal(0, noise_std, size=arrival_times.shape)
    return arrival_times_noisy, distances

def tdoa_multilateration(anchors, arrival_times, c=343.0):
    """
    Estimate the 3D source position using TDOA measurements.
    
    Uses the first anchor as the reference. For anchors i = 1,...,N-1, the relation is:
    
        ||x - p_i|| = ||x - p_0|| + c*(t_i - t_0)
    
    After squaring and subtracting the equation for anchor 0, we get a linear system.
    
    Parameters:
        anchors (ndarray): (N, 3) array of anchor positions.
        arrival_times (ndarray): Arrival times at each anchor (in seconds).
        c (float): Speed of sound in m/s.
        
    Returns:
        estimated_position (ndarray): Estimated 3D position of the source.
        estimated_d0 (float): Estimated distance from the source to the reference anchor.
    """
    p0 = anchors[0]
    t0 = arrival_times[0]
    delta_ts = arrival_times[1:] - t0
    N = anchors.shape[0]
    
    A = []
    b = []
    for i in range(1, N):
        pi = anchors[i]
        delta_t = delta_ts[i - 1]
        row = np.concatenate((2 * (p0 - pi), np.array([-2 * c * delta_t])))
        A.append(row)
        b_val = c**2 * (delta_t**2) - (np.dot(pi, pi) - np.dot(p0, p0))
        b.append(b_val)
    
    A = np.array(A)
    b = np.array(b)
    
    sol, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    estimated_position = sol[:3]
    estimated_d0 = sol[3]
    return estimated_position, estimated_d0

# Set up anchor positions (fixed)
anchors = np.array([
    [0, 0, 0],    # Reference anchor
    [5, 0, 0],
    [0, 5, 0],
    [0, 0, 5],
    [5, 5, 5]
])
c = 343.0  # Speed of sound (m/s)

# Create a figure and 3D axes.
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot fixed anchor positions.
ax.scatter(anchors[:, 0], anchors[:, 1], anchors[:, 2],
           c='blue', marker='^', s=100, label='Anchors')

# Create scatter objects for true and estimated source positions.
true_scatter = ax.scatter([], [], [], c='green', marker='o', s=100, label='True Source')
est_scatter = ax.scatter([], [], [], c='red', marker='x', s=100, label='Estimated Source')

# Create line objects for the trajectories.
true_line, = ax.plot([], [], [], 'g-', label='True Trajectory')
est_line, = ax.plot([], [], [], 'r-', label='Estimated Trajectory')

ax.set_xlim(-1, 7)
ax.set_ylim(-1, 7)
ax.set_zlim(-1, 7)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.legend()

# Lists to store trajectory data.
true_traj = []
est_traj = []

def update(frame):
    global true_traj, est_traj
    
    # Define a moving trajectory for the true source.
    # Here, it moves in a circle in the x-y plane and oscillates in z.
    t = frame / 10.0
    true_pos = np.array([
        2 + np.cos(t),
        2 + np.sin(t),
        2 + 0.5 * np.sin(2 * t)
    ])
    
    # Simulate arrival times (with noise) at the anchors.
    arrival_times, _ = simulate_tdoa(anchors, true_pos, c=c, noise_std=1e-5)
    
    # Compute and print time differences relative to the first anchor.
    time_differences = arrival_times - arrival_times[0]
    print(f"Frame {frame}: Time differences (s): {time_differences}")
    
    # Estimate the source position using TDOA multilateration.
    est_pos, _ = tdoa_multilateration(anchors, arrival_times, c=c)
    
    # Append positions to trajectory lists.
    true_traj.append(true_pos)
    est_traj.append(est_pos)
    
    # Update scatter data for the current positions.
    true_scatter._offsets3d = ([true_pos[0]], [true_pos[1]], [true_pos[2]])
    est_scatter._offsets3d = ([est_pos[0]], [est_pos[1]], [est_pos[2]])
    
    # Update trajectory lines.
    true_traj_arr = np.array(true_traj)
    est_traj_arr = np.array(est_traj)
    true_line.set_data(true_traj_arr[:, 0], true_traj_arr[:, 1])
    true_line.set_3d_properties(true_traj_arr[:, 2])
    est_line.set_data(est_traj_arr[:, 0], est_traj_arr[:, 1])
    est_line.set_3d_properties(est_traj_arr[:, 2])
    
    return true_scatter, est_scatter, true_line, est_line

# Create an animation that updates every 100 ms.
ani = FuncAnimation(fig, update, frames=200, interval=100, blit=False)

plt.show()
