import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from scipy.optimize import least_squares
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from time import time
from collections import deque

# === Konfigurasjon ===
SERIAL_PORT = 'COM5' 
BAUD_RATE = 115200

mic_positions = np.array([
    [0.008, 0.17, 0.323],
    [0.384, 0.06, 0.323],
    [0.319, 0.485, 0.323],
    [0.218, 0.241, 0]
])

speed_of_sound = 343.0


# === Estimeringsfunksjoner ===

def initial_guess(times, mic_pos, c=speed_of_sound):
    closest = np.argsort(times)[:3]
    p1, p2, p3 = mic_pos[closest]
    A = 2 * np.vstack((p2 - p1, p3 - p1))
    b = c**2 * (times[closest[0]]**2 - times[closest[1:]]**2)
    b += np.sum([p2**2 - p1**2, p3**2 - p1**2], axis=1)
    initial_pos, *_ = np.linalg.lstsq(A, b, rcond=None)
    return initial_pos

def tdoa_equations(pos, tdoas, mic_positions, speed_of_sound):
    r = np.linalg.norm(pos - mic_positions[1:], axis=1)
    r0 = np.linalg.norm(pos - mic_positions[0])
    return (r - r0) - speed_of_sound * tdoas

def jacobian(pos, mic_pos):
    """Eksplisitt Jacobian for TDoA-systemet"""
    J = np.zeros((3, 3))
    r0 = np.linalg.norm(pos - mic_pos[0])
    for i in range(1, 4):
        ri = np.linalg.norm(pos - mic_pos[i])
        J[i-1, :] = (pos - mic_pos[i])/ri - (pos - mic_pos[0])/r0
    return J

def estimate_position(arrival_times, previous_estimate = None):
    t_ref = arrival_times[0]
    tdoas = arrival_times[1:] - t_ref
    bounds=([0, 0, 0], [0.5, 0.5, 0.5]) 
    if previous_estimate is not None and np.all(np.isfinite(previous_estimate)):
        init_pos = np.clip(previous_estimate, bounds[0], bounds[1])
    else:
        init_pos = initial_guess(arrival_times, mic_positions)
        init_pos = np.clip(init_pos, bounds[0], bounds[1])

    result = least_squares(
        fun=lambda x: tdoa_equations(x, tdoas, mic_positions, speed_of_sound),
        x0=init_pos,
        jac=lambda x: jacobian(x, mic_positions),
        bounds=bounds
    )
    return result.x

# === Serial-tilkobling ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# === Matplotlib-setup ===
fig = plt.figure(figsize=(12, 6))
gs = gridspec.GridSpec(3, 2, width_ratios=[2, 1])


# 3D-plotet tar hele venstre side
ax = fig.add_subplot(gs[:, 0], projection='3d')
ax.set_xlim(0, 0.5)
ax.set_ylim(0, 0.5)
ax.set_zlim(0, 0.6)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Posisjonsestimering')



# Første 2D-plot: høyde (z) som funksjon av tid
ax_height = fig.add_subplot(gs[0, 1])
ax_height.set_title("Dybde")
ax_height.set_xlabel("Tid (s)")
ax_height.set_ylabel("Dybde (m)")

# Andre 2D-plot: fart som funksjon av tid
ax_speed = fig.add_subplot(gs[1, 1])
ax_speed.set_title("Fart")
ax_speed.set_xlabel("Tid (s)")
ax_speed.set_ylabel("Fart (m/s)")
speed_line, = ax_speed.plot([], [], color='orange')

# Tredje 2D-plot: akselerasjon som funksjon av tid
ax_accel = fig.add_subplot(gs[2, 1])
ax_accel.set_title("Akselerasjon")
ax_accel.set_xlabel("Tid (s)")
ax_accel.set_ylabel("Akselerasjon (m/s²)")
accel_line, = ax_accel.plot([], [], color='green')




 #Plott mikrofoner
ax.scatter(mic_positions[:,0], mic_positions[:,1], mic_positions[:,2], color='red', label='Mikrofoner')
for i, pos in enumerate(mic_positions):
    ax.text(pos[0], pos[1], pos[2], f'M{i}', fontsize=10)

# Punkt for estimert posisjon
scatter = ax.scatter([], [], [], color='blue', s=100, label='Estimert posisjon')



# Liste (med maks lengde) for spor
max_trail_len = 20
trail = deque(maxlen=max_trail_len)
last_estimate = None
#For dybde (m)
height_times = deque(maxlen=200)  # Tidsstempler
height_values = deque(maxlen=200)  # Z-verdier
# For fart (m/s)
positions = deque(maxlen=500)
timestamps = deque(maxlen=500)
speed_times = deque(maxlen=500)
speed_values = deque(maxlen=500)
# For akselerasjon (m/s^2)
accel_times = deque(maxlen=500)
accel_values = deque(maxlen=500)


start_time = time()
height_line, = ax_height.plot([], [], color='green')

# Lag en tom Line3DCollection for sporet
trail_line = Line3DCollection([], linewidth=2, alpha=1.0)
trail_line.set_segments([[[0, 0, 0], [0, 0, 0]]])
ax.add_collection3d(trail_line)


def update(frame):
    global last_estimate
    try:
        raw_line = ser.readline()
        if not raw_line:
            print("Tom linje mottatt")
            return scatter,

        line = raw_line.decode('utf-8', errors='ignore').strip()
        if line.startswith("#"):
            print(line)  # valgfritt for å vise faktisk posisjon
            return scatter,

        times = np.array([float(x) for x in line.split(',')])
        if len(times) != 4:
            print(f"Feil: forventet 4 tider, fikk {len(times)}")
            return scatter,

        est = estimate_position(times, previous_estimate = last_estimate)
        positions.append(est)
        timestamps.append(time() - start_time)

        last_estimate = est
        print(f"Estimert posisjon: {est}")

        if est is None or len(est) != 3 or not np.all(np.isfinite(est)):
            print(f"Ugyldig estimering: {est}")
            return scatter,

        # Oppdater punktet i plottet
        scatter._offsets3d = ([est[0]], [est[1]], [est[2]])
        trail.append(est)

        now = time() - start_time
        height_times.append(now)
        height_values.append(est[2])  # Z-koordinat

        # Fjern gamle verdier hvis mer enn 60 sekunder har gått
        if now > 60:
            while height_times and (now - height_times[0] > 60):
                height_times.popleft()
                height_values.popleft()
            ax_height.set_xlim(now - 60, now)
        else:
            ax_height.set_xlim(0, 60)  # Fast x-akse i starten (0–60 sek)

        
        height_line.set_data(height_times, height_values)
        ax_height.relim()
        ax_height.autoscale_view()

        # Beregn fart
        if len(positions) >= 6 and len(timestamps) >= 6:
            speeds = []
            for i in range(-6, -1): 
                dp = np.array(positions[i+1]) - np.array(positions[i])
                dt = timestamps[i+1] - timestamps[i]
                if dt > 0:
                    speeds.append(np.linalg.norm(dp) / dt)
            if speeds:
                speed = np.mean(speeds)
                speed_times.append(timestamps[-1])
                speed_values.append(speed)
            
                # Filtrer ut verdier eldre enn 60 sekunder
                recent_speed_times = [t for t in speed_times if timestamps[-1] - t <= 60]
                recent_speed_values = [v for i, v in enumerate(speed_values) if timestamps[-1] - speed_times[i] <= 60]

                speed_line.set_data(recent_speed_times, recent_speed_values)

                if recent_speed_times:
                    ax_speed.set_xlim(max(0, timestamps[-1] - 60), timestamps[-1])
                else:
                    ax_speed.set_xlim(0, 60)

                if recent_speed_values:
                    ax_speed.set_ylim(0, max(recent_speed_values) * 1.2)
                else:
                    ax_speed.set_ylim(0, 0.1)

        # --- Akselerasjon ---
        if len(speed_times) >= 2:
            dv = speed_values[-1] - speed_values[-2]
            dt = speed_times[-1] - speed_times[-2]
            if dt > 0:
                accel = dv / dt
                accel_times.append(speed_times[-1])
                accel_values.append(accel)

            # Filtrer ut verdier eldre enn 60 sekunder
            recent_accel_times = [t for t in accel_times if timestamps[-1] - t <= 60]
            recent_accel_values = [v for i, v in enumerate(accel_values) if timestamps[-1] - accel_times[i] <= 60]

            accel_line.set_data(recent_accel_times, recent_accel_values)

            if recent_accel_times:
                ax_accel.set_xlim(max(0, timestamps[-1] - 60), timestamps[-1])
            else:
                ax_accel.set_xlim(0, 60)

            if recent_accel_values:
                amin = min(recent_accel_values)
                amax = max(recent_accel_values)
                if amin == amax:
                    margin = 0.1  # liten margin hvis flat linje
                    ax_accel.set_ylim(amin - margin, amax + margin)
                else:
                    ax_accel.set_ylim(amin * 1.2, amax * 1.2)
            else:
                ax_accel.set_ylim(-0.1, 0.1)


        # --- Oppdater spor ---
        if len(trail) > 1:
            points = np.array(trail)
            segments = [[points[i], points[i+1]] for i in range(len(points) - 1)]
            alphas = np.linspace(0.1, 1.0, len(segments))
            colors = [(0, 0, 1, alpha) for alpha in alphas]
            trail_line.set_segments(segments)
            trail_line.set_color(colors)
        else:
            trail_line.set_segments([])

    except Exception as e:
        print(f"Feil i update(): {e}")
    
    return scatter,



ani = FuncAnimation(fig, update, interval=200)
plt.legend()
plt.tight_layout()
plt.show()  