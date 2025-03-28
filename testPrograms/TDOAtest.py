import numpy as np

def distance(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def calculate_tdoa_position_3d(receivers, tdoa_array, propagation_speed, initial_guess):
    d12 = propagation_speed * (tdoa_array[0] - tdoa_array[1])
    d13 = propagation_speed * (tdoa_array[0] - tdoa_array[2])
    d14 = propagation_speed * (tdoa_array[0] - tdoa_array[3])
    x, y, z = initial_guess
    max_iterations = 100
    tol = 1e-6

    # Convert receivers to numpy arrays for easier arithmetic.
    R = [np.array(r) for r in receivers]

    for _ in range(max_iterations):
        guess = np.array([x, y, z])
        d1 = distance(guess, R[0])
        d2 = distance(guess, R[1])
        d3 = distance(guess, R[2])
        d4 = distance(guess, R[3])
        
        # Prevent division by zero.
        if d1 < 1e-9 or d2 < 1e-9 or d3 < 1e-9 or d4 < 1e-9:
            print("Division by near-zero; aborting iteration.")
            return None

        # Define the nonlinear functions:
        # f = d1 - d2 - d12,  g = d1 - d3 - d13,  h = d1 - d4 - d14.
        f_val = d1 - d2 - d12
        g_val = d1 - d3 - d13
        h_val = d1 - d4 - d14

        # Compute partial derivatives for the Jacobian.
        df_dx = ((x - R[0][0]) / d1) - ((x - R[1][0]) / d2)
        df_dy = ((y - R[0][1]) / d1) - ((y - R[1][1]) / d2)
        df_dz = ((z - R[0][2]) / d1) - ((z - R[1][2]) / d2)

        dg_dx = ((x - R[0][0]) / d1) - ((x - R[2][0]) / d3)
        dg_dy = ((y - R[0][1]) / d1) - ((y - R[2][1]) / d3)
        dg_dz = ((z - R[0][2]) / d1) - ((z - R[2][2]) / d3)

        dh_dx = ((x - R[0][0]) / d1) - ((x - R[3][0]) / d4)
        dh_dy = ((y - R[0][1]) / d1) - ((y - R[3][1]) / d4)
        dh_dz = ((z - R[0][2]) / d1) - ((z - R[3][2]) / d4)

        # Assemble the Jacobian matrix.
        J = np.array([
            [df_dx, df_dy, df_dz],
            [dg_dx, dg_dy, dg_dz],
            [dh_dx, dh_dy, dh_dz]
        ])
        F = np.array([f_val, g_val, h_val])
        
        # Solve the linear system: J * delta = -F
        try:
            delta = np.linalg.solve(J, -F)
        except np.linalg.LinAlgError:
            print("Singular Jacobian encountered; aborting iteration.")
            return None
        
        # Update the guess.
        x += delta[0]
        y += delta[1]
        z += delta[2]
        
        if np.linalg.norm(delta) < tol:
            break

    if np.isnan(x) or np.isnan(y) or np.isnan(z) or np.isinf(x) or np.isinf(y) or np.isinf(z):
        print("Error: Calculation resulted in NaN or Infinity.")
        return None

    return (x, y, z)

def calculate_toa(receivers, transmitter, propagation_speed):
    """
    Compute time-of-arrival (TOA) in microseconds for each receiver.
    propagation_speed should be in m/µs.
    """
    toas = []
    for receiver in receivers:
        d = distance(np.array(transmitter), np.array(receiver))
        t = d / propagation_speed  # t in microseconds
        toas.append(t)
    return toas

def main():
    # Fixed receiver coordinates (4 points in 3D space)
    receivers = [
        (0.5, 0.3, 0.45),
        (0.2, 0.47, 0.45),
        (0.2, 0.13, 0.45),
        (0.33, 0.33, 0.12)
    ]
    
    print("Receiver coordinates:")
    for i, rec in enumerate(receivers, start=1):
        print(f"Receiver {i}: {rec}")
    
    # Propagation speed: use 0.00034 m/µs (equivalent to 340 m/s)
    propagation_speed = 0.00034  # m/µs

    # Ask user which mode to use.
    mode = input("\nChoose mode: 'coords' to use transmitter coordinates, 'timestamps' to input TOA values in microseconds: ").strip().lower()

    if mode == "coords":
        # Get transmitter coordinates from user.
        tx_x = float(input("Enter transmitter X coordinate (meters): "))
        tx_y = float(input("Enter transmitter Y coordinate (meters): "))
        tx_z = float(input("Enter transmitter Z coordinate (meters): "))
        transmitter = (tx_x, tx_y, tx_z)
    
        # Calculate TOA for each receiver (in µs).
        toas = calculate_toa(receivers, transmitter, propagation_speed)
        print("\nCalculated time-of-arrival (TOA) for each receiver (µs):")
        for i, t in enumerate(toas, start=1):
            print(f"Receiver {i}: {t:.6f} µs")
    elif mode == "timestamps":
        # Let the user input the TOA values manually (in microseconds).
        toas = []
        print("\nEnter the time-of-arrival values in microseconds for each receiver:")
        for i in range(4):
            t = float(input(f"Receiver {i+1} (µs): "))
            toas.append(t)
    else:
        print("Unknown mode selected.")
        return
    
    # For TDOA, use receiver 0 as the reference.
    # tdoa_array: subtract receiver 0's TOA from each.
    tdoa_array = [t - toas[0] for t in toas]
    print("\nTDOA (relative to Receiver 1) (µs):")
    for i, t in enumerate(tdoa_array, start=1):
        print(f"Receiver {i}: {t:.6f} µs")
    
    # Set an initial guess for the transmitter position.
    initial_guess = (0.3, 0.3, 0.3)
    
    # Estimate transmitter position using the Newton–Raphson solver.
    estimated_position = calculate_tdoa_position_3d(receivers, tdoa_array, propagation_speed, initial_guess)
    if estimated_position is not None:
        print("\nEstimated Transmitter Position (x,y,z):")
        print("{:.6f}, {:.6f}, {:.6f}".format(*estimated_position))
    else:
        print("\nTransmitter position could not be estimated.")

if __name__ == "__main__":
    main()
