import numpy as np
import matplotlib.pyplot as plt

# Constants
SAMPLE_FREQUENCY = 390000  # Hz
TARGET_FREQUENCY = 39000   # Hz
REF_TABLE_SIZE = 10
Q15_SCALE = 32767
SIGNAL_LENGTH = 60          # samples
WINDOW_SIZE = REF_TABLE_SIZE
THRESHOLD = 4e15              # Adjustable threshold for detection
VOLTAGE_FULL_SCALE = 1.65  # Max voltage corresponds to Q15 max

# Parameters in volts
NOISE_AMPLITUDE_VOLT = 0.005  # Peak amplitude of noise in volts
BURST_AMPLITUDE_VOLT = 0.010   # Amplitude of signal in volts
BURST_START = 10

# Convert voltage to Q15 format
def volt_to_q15(voltage):
    return int(voltage / VOLTAGE_FULL_SCALE * Q15_SCALE)

NOISE_AMPLITUDE = volt_to_q15(NOISE_AMPLITUDE_VOLT)
BURST_AMPLITUDE = volt_to_q15(BURST_AMPLITUDE_VOLT)

# Generate sine/cos reference tables (Q15 format)
def generate_reference_tables():
    angle_step = 2 * np.pi * TARGET_FREQUENCY / SAMPLE_FREQUENCY
    sine = np.array([np.sin(i * angle_step) / REF_TABLE_SIZE for i in range(REF_TABLE_SIZE)])
    cosine = np.array([np.cos(i * angle_step) / REF_TABLE_SIZE for i in range(REF_TABLE_SIZE)])
    sine_q15 = np.round(sine * Q15_SCALE).astype(np.int16)
    cosine_q15 = np.round(cosine * Q15_SCALE).astype(np.int16)
    return sine_q15, cosine_q15

# Generate synthetic signal with a burst and noise
def generate_signal():
    noise = np.random.uniform(-NOISE_AMPLITUDE, NOISE_AMPLITUDE, SIGNAL_LENGTH)
    signal = noise.copy()
    for i in range(BURST_START, SIGNAL_LENGTH):
        signal[i] += BURST_AMPLITUDE * np.sin(2 * np.pi * TARGET_FREQUENCY / SAMPLE_FREQUENCY * (i - BURST_START))
    signal_q15 = np.clip(np.round(signal).astype(np.int16), -32768, 32767)
    return signal_q15

# Normalize signal to Q15 and prevent overflow
def normalize_signal_q15(signal):
    abs_max = np.max(np.abs(signal))
    if abs_max == 0:
        abs_max = 1
    scale_factor = ((Q15_SCALE << 15) // abs_max)
    signal = ((signal.astype(np.int32) * scale_factor) >> 15).astype(np.int16)
    return signal // REF_TABLE_SIZE

# Calculate signal power over a sliding window
def calculate_signal_power(signal, sine_ref, cos_ref):
    power = np.zeros(SIGNAL_LENGTH - REF_TABLE_SIZE + 1, dtype=np.uint64)
    for i in range(SIGNAL_LENGTH - REF_TABLE_SIZE + 1):
        segment = signal[i:i + REF_TABLE_SIZE].astype(np.int64)
        I = np.dot(segment, sine_ref.astype(np.int64))
        Q = np.dot(segment, cos_ref.astype(np.int64))
        power[i] = I * I + Q * Q
    return power

# Plot signal and power with threshold line
def plot_signal_and_power(original_signal, normalized_signal, power_original, power_normalized):
    time_axis = np.arange(SIGNAL_LENGTH) * 2.56  # in microseconds
    power_time_axis = np.arange(len(power_original)) * 2.56 + (REF_TABLE_SIZE // 2) * 2.56  # Centered on window
    min_time = min(time_axis[0], power_time_axis[0])
    max_time = max(time_axis[-1], power_time_axis[-1])

    fig, axs = plt.subplots(2, 2, figsize=(14, 6))

    axs[0][0].plot(time_axis, original_signal, label='Original Signal')
    axs[0][0].set_ylabel('Amplitude (Q15)')
    axs[0][0].set_title('Original Signal')
    axs[0][0].legend()
    axs[0][0].grid(True)
    axs[0][0].set_xlim(min_time, max_time)

    axs[1][0].plot(power_time_axis, power_original, label='Power (Original)')
    axs[1][0].axhline(y=THRESHOLD, color='r', linestyle='--', label=f'Threshold ({THRESHOLD:.0e})')
    axs[1][0].set_ylabel('Power')
    axs[1][0].set_xlabel('Time (us)')
    axs[1][0].legend()
    axs[1][0].grid(True)
    axs[1][0].set_xlim(min_time, max_time)

    axs[0][1].plot(time_axis, normalized_signal, label='Normalized Signal')
    axs[0][1].set_title('Normalized Signal')
    axs[0][1].legend()
    axs[0][1].grid(True)
    axs[0][1].set_xlim(min_time, max_time)

    axs[1][1].plot(power_time_axis, power_normalized, label='Power (Normalized)')
    axs[1][1].axhline(y=THRESHOLD, color='r', linestyle='--', label=f'Threshold ({THRESHOLD:.0e})')
    axs[1][1].set_ylabel('Power')
    axs[1][1].set_xlabel('Time (us)')
    axs[1][1].legend()
    axs[1][1].grid(True)
    axs[1][1].set_xlim(min_time, max_time)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    sine_ref, cos_ref = generate_reference_tables()
    signal_original = generate_signal()
    signal_normalized = normalize_signal_q15(signal_original.copy())
    power_original = calculate_signal_power(signal_original, sine_ref, cos_ref)
    power_normalized = calculate_signal_power(signal_normalized, sine_ref, cos_ref)
    plot_signal_and_power(signal_original, signal_normalized, power_original, power_normalized)
