import numpy as np

def calculate_tdoa(receivers, transmitter, propagation_speed):
    """
    Calculate the time of arrival at each receiver from the transmitter.

    Parameters:
    - receivers: List of receiver coordinates [(x, y, z), ...]
    - transmitter: Tuple of transmitter coordinates (x, y, z)
    - propagation_speed: Speed of signal propagation (meters/second)

    Returns:
    - Times for each receiver in milliseconds
    """
    distances = [np.linalg.norm(np.array(transmitter) - np.array(receiver)) for receiver in receivers]
    times = [distance / propagation_speed * 1e6 for distance in distances]  # Convert to microseconds
    return times

def main():
    # Default receiver coordinates (4 points in 3D space)
    receivers = [
        (0, 0, 0),
        (1, 0, 0),
        (0, 1, 0),
        (0, 0.7, 1)
    ]

    print("Receiver coordinates are fixed at:")
    for i, receiver in enumerate(receivers, 1):
        print(f"Receiver {i}: {receiver}")

    # Get propagation speed from user
    propagation_speed = 340

    # Get transmitter coordinates from user
    tx_x = float(input("Enter transmitter X coordinate (meters): "))
    tx_y = float(input("Enter transmitter Y coordinate (meters): "))
    tx_z = float(input("Enter transmitter Z coordinate (meters): "))

    transmitter = (tx_x, tx_y, tx_z)

    # Calculate time of arrival for each receiver
    times = calculate_tdoa(receivers, transmitter, propagation_speed)

    # Display results
    print("\nTime of arrival for each receiver (in microseconds):")
    for i, time in enumerate(times, 1):
        print(f"Receiver {i}: {time:.6f} µs")

if __name__ == "__main__":
    main()
