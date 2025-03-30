#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <sstream>
#include <algorithm>
#include <limits>

using namespace std;
// found values with noise amplitude 20.0 and signal/burst amplitude 40.0 used as base for signals in real design
// unnormalized signal threshold 135 has a true positive percentage above 98.78%
// 0.9878^4 > 0.95 which means the probability of not reading a signal at 40.0 mv amplitude is less than 5%
// false positive percentage would be above 0.08% which is most likely unnaceptable
// unnormalized signal threshold 200 has a false positive percentage of < 0.00055% which could be usable
// the true positive percentage would be above 97.25%
// 0.9725^4 > 0.8944 which means the probability of not reading a signal at 40.0 mv amplitude is about 10%
// The latter threshold is probably preferable as the vast majority of buffers will just contain noise
// and when the signal amplitude is low for one buffer it is most likely higher for others leading to
// a much higher realized succesfully read signal percentage


// normalized threshold value: 600000 
// -2 samples mean net error 95% of measured difference between -6 and 0 samples
// mean net error changes with signal amplitude but absolute delta is of 6 is consistent
// with fine tuned offset we could reach an average accuracy of less than 3 mm
// calulated with 3 * 2.56(time between samples us) * 0.343 (speed of sound in mm/us)


// Simulation constants
const int SAMPLE_LENGTH = 60;
const int REF_TABLE_SIZE = 10;

const double ADC_REF_VOLTAGE = 3.3;
const int ADC_MAX_VALUE = 4095;
const double ADC_STEP = ADC_REF_VOLTAGE / ADC_MAX_VALUE; // ≈0.00080586 V per count

// Set noise and burst amplitudes (in mV), then convert to ADC counts.
// Adjust these values as needed.
const double NOISE_AMPLITUDE_MV = 20.0;
const double BURST_AMPLITUDE_MV = 1000.0;
const int NOISE_AMPLITUDE_COUNTS = static_cast<int>(round(NOISE_AMPLITUDE_MV / (ADC_STEP * 1000)));
const int BURST_AMPLITUDE_COUNTS = static_cast<int>(round(BURST_AMPLITUDE_MV / (ADC_STEP * 1000)));

// Global reference tables (Q15 format)
vector<int16_t> ref_sine_table_Q15(REF_TABLE_SIZE);
vector<int16_t> ref_cos_table_Q15(REF_TABLE_SIZE);

// Forward declarations
void run_simulation_threshold(uint64_t threshold, double signalProb, unsigned int numBuffers);
void run_simulation_start_estimation(uint64_t threshold, unsigned int numBuffers, double confLevel, int maxSteps);

// Initialize the Q15 sine table.
// The table is scaled by dividing by the table size to avoid overflow in dot product calculations.
void initialize_sine_table_Q15() {
    double angleStep = 2 * M_PI / REF_TABLE_SIZE;
    for (int i = 0; i < REF_TABLE_SIZE; i++) {
        double value = sin(i * angleStep) / REF_TABLE_SIZE;
        ref_sine_table_Q15[i] = static_cast<int16_t>(round(value * 32767));
    }
}

// Initialize the Q15 cosine table.
void initialize_cos_table_Q15() {
    double angleStep = 2 * M_PI / REF_TABLE_SIZE;
    for (int i = 0; i < REF_TABLE_SIZE; i++) {
        double value = cos(i * angleStep) / REF_TABLE_SIZE;
        ref_cos_table_Q15[i] = static_cast<int16_t>(round(value * 32767));
    }
}

// Computes signal strength (R² + I²) over a window of REF_TABLE_SIZE samples.
// This function simulates the DSPS dot product method for Q15 arithmetic.
// Each multiplication of two Q15 numbers produces a Q30 result, which is then shifted right by 15 bits.
uint64_t calculate_signal_strength(const vector<int16_t>& window) {
    int32_t R = 0, I = 0;
    for (int i = 0; i < REF_TABLE_SIZE; i++) {
        // Multiply Q15 numbers (window[i] and ref_cos_table_Q15[i]) => Q30, then accumulate.
        R += static_cast<int32_t>(window[i]) * ref_cos_table_Q15[i];
        I += static_cast<int32_t>(window[i]) * ref_sine_table_Q15[i];
    }
    // To mimic DSPS behavior, shift the accumulated results right by 15 bits.
    R >>= 15;
    I >>= 15;
    return static_cast<uint64_t>(R) * R + static_cast<uint64_t>(I) * I;
}

// Generates a simulated buffer of SAMPLE_LENGTH samples with noise.
// If hasBurst is true, a sine burst is added starting at burstStart.
vector<int16_t> simulate_signal(bool hasBurst, int burstStart) {
    vector<int16_t> signal(SAMPLE_LENGTH, 0);
    static random_device rd;
    static mt19937 gen(rd());
    uniform_int_distribution<> noise_dist(-NOISE_AMPLITUDE_COUNTS, NOISE_AMPLITUDE_COUNTS);
    for (int i = 0; i < SAMPLE_LENGTH; i++) {
        signal[i] = noise_dist(gen);
    }
    if (hasBurst) {
        // Ensure burst fits in the signal buffer.
        if (burstStart > SAMPLE_LENGTH - 5)
            burstStart = SAMPLE_LENGTH - 5;
        for (int i = burstStart; i < SAMPLE_LENGTH; i++) {
            double phase = 2 * M_PI * (i - burstStart) / REF_TABLE_SIZE;
            int burst_val = static_cast<int>(round(BURST_AMPLITUDE_COUNTS * sin(phase)));
            signal[i] += burst_val;
        }
    }
    return signal;
}

// Normalizes the signal into Q15 format.
// The normalization scales the signal so that its maximum absolute value becomes 32767,
// and then divides each sample by REF_TABLE_SIZE to match the scaling used in the dot product.
void normalize_signal(vector<int16_t>& signal) {
    int16_t abs_max = 0;
    for (int16_t val : signal)
        abs_max = max(abs_max, static_cast<int16_t>(abs(val)));
    if (abs_max == 0) abs_max = 1;
    double scale = 32767.0 / abs_max;
    for (size_t i = 0; i < signal.size(); i++) {
        signal[i] = static_cast<int16_t>(round(signal[i] * scale));
        signal[i] /= REF_TABLE_SIZE;
    }
}

// Mode 1: Threshold detection simulation.
// Processes numBuffers buffers using a mixture of signal and noise based on signalProb,
// and reports detection metrics.
void run_simulation_threshold(uint64_t threshold, double signalProb, unsigned int numBuffers) {
    uint64_t totalSignal = 0, totalNoise = 0;
    uint64_t truePos = 0, falseNeg = 0, trueNeg = 0, falsePos = 0;
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> burstDist(0, SAMPLE_LENGTH - 5);
    uniform_real_distribution<> probDist(0.0, 1.0);

    for (unsigned int i = 0; i < numBuffers; i++) {
        bool isSignal = (probDist(gen) < signalProb);
        int burstStart = burstDist(gen);
        vector<int16_t> signal = simulate_signal(isSignal, burstStart);
        // Use the last REF_TABLE_SIZE samples as the window.
        vector<int16_t> window(signal.end() - REF_TABLE_SIZE, signal.end());
        uint64_t strength = calculate_signal_strength(window);
        if (isSignal) {
            totalSignal++;
            if (strength >= threshold)
                truePos++;
            else
                falseNeg++;
        } else {
            totalNoise++;
            if (strength < threshold)
                trueNeg++;
            else
                falsePos++;
        }
    }

    double pctTruePos = (totalSignal > 0) ? (100.0 * truePos / totalSignal) : 0.0;
    double pctFalseNeg = (totalSignal > 0) ? (100.0 * falseNeg / totalSignal) : 0.0;
    double pctTrueNeg = (totalNoise > 0) ? (100.0 * trueNeg / totalNoise) : 0.0;
    double pctFalsePos = (totalNoise > 0) ? (100.0 * falsePos / totalNoise) : 0.0;

    cout << "\n[Threshold Simulation]" << endl;
    cout << "Threshold: " << threshold << "\n";
    if (signalProb == 0.0)
        cout << "All buffers contain noise (0% signal)." << endl;
    else if (signalProb == 1.0)
        cout << "All buffers contain signal (0% noise)." << endl;
    else {
        double noiseToSignal = (1.0 - signalProb) / signalProb;
        cout << "Noise-to-Signal Ratio: " << noiseToSignal << ":1" << endl;
    }
    cout << "Total Signal Buffers: " << totalSignal << "   Total Noise Buffers: " << totalNoise << "\n";
    cout << "True Positives: " << truePos << " (" << pctTruePos << "%)" << endl;
    cout << "False Negatives: " << falseNeg << " (" << pctFalseNeg << "%)" << endl;
    cout << "True Negatives: " << trueNeg << " (" << pctTrueNeg << "%)" << endl;
    cout << "False Positives: " << falsePos << " (" << pctFalsePos << "%)" << endl;
}

// Mode 2: Start estimation simulation.
// For each signal buffer, after normalization, a binary search estimates the burst start.
// The differences (estimated - actual) are collected, and statistics are computed.
void run_simulation_start_estimation(uint64_t threshold, unsigned int numBuffers, double confLevel, int maxSteps) {
    vector<int> differences;
    random_device rd;
    mt19937 gen(rd());
    // Choose actual burst start such that the window is valid.
    uniform_int_distribution<> burstDist(0, SAMPLE_LENGTH - REF_TABLE_SIZE - 1);
    
    for (unsigned int i = 0; i < numBuffers; i++) {
        int actualBurstStart = burstDist(gen);
        vector<int16_t> signal = simulate_signal(true, actualBurstStart);
        normalize_signal(signal);
        
        // Binary search to estimate burst start.
        int low = 0, high = SAMPLE_LENGTH - REF_TABLE_SIZE, steps = 0, estimated = high;
        // Check lower boundary.
        if (calculate_signal_strength(vector<int16_t>(signal.begin(), signal.begin() + REF_TABLE_SIZE)) >= threshold)
            estimated = 0;
        // Check upper boundary.
        else if (calculate_signal_strength(vector<int16_t>(signal.begin() + high, signal.begin() + high + REF_TABLE_SIZE)) < threshold)
            estimated = high;
        else {
            while (low <= high && steps < maxSteps) {
                int mid = low + (high - low) / 2;
                vector<int16_t> window(signal.begin() + mid, signal.begin() + mid + REF_TABLE_SIZE);
                uint64_t strength = calculate_signal_strength(window);
                if (strength >= threshold) {
                    estimated = mid;
                    high = mid - 1;
                } else {
                    low = mid + 1;
                }
                steps++;
            }
        }
        differences.push_back(estimated - actualBurstStart);
    }
    
    // Sort differences to compute percentiles.
    sort(differences.begin(), differences.end());
    size_t n = differences.size();
    int lowerIndex = static_cast<int>(0.025 * n);
    int upperIndex = static_cast<int>(0.975 * n);
    if(lowerIndex < 0) lowerIndex = 0;
    if(upperIndex >= n) upperIndex = n - 1;
    
    int lowerBound = differences[lowerIndex];
    int upperBound = differences[upperIndex];
    
    // Compute the mean net difference.
    double sum = 0.0;
    for (int diff : differences)
        sum += diff;
    double meanDiff = sum / n;
    
    cout << "\n[Start Estimation Simulation]" << endl;
    cout << "Threshold: " << threshold << "\n";
    cout << "Total Signal Buffers Analyzed: " << n << "\n";
    cout << "Mean net error (estimated - actual): " << meanDiff << " samples" << endl;
    cout << "Approximately 95% of measured differences fall between " 
         << lowerBound << " and " << upperBound << " samples" << endl;
}

int main() {
    // Initialize reference tables in Q15 format.
    initialize_sine_table_Q15();
    initialize_cos_table_Q15();

    cout << "Select simulation mode:" << endl;
    cout << "  1 - Threshold detection simulation" << endl;
    cout << "  2 - Start estimation simulation (with normalization & binary search)" << endl;
    cout << "Your choice: ";
    string mode_input;
    cin >> mode_input;
    int mode = stoi(mode_input);

    double signalProb = 0.0;
    if (mode == 1) {
        cout << "\nEnter the noise-to-signal ratio:" << endl;
        cout << "A positive value (e.g., 100) indicates 1 signal for every 100 noise buffers." << endl;
        cout << "A negative value (e.g., -100) indicates 100 signals for every 1 noise buffer." << endl;
        cout << "Enter 0 for all noise." << endl;
        cout << "Your input: ";
        string ratio_input;
        cin >> ratio_input;
        if (ratio_input == "q" || ratio_input == "Q")
            return 0;
        int ratioValue;
        stringstream ss(ratio_input);
        ss >> ratioValue;
        if (ratioValue == 0)
            signalProb = 0.0;
        else if (ratioValue < 0)
            signalProb = static_cast<double>(-ratioValue) / (-ratioValue + 1.0);
        else
            signalProb = 1.0 / (ratioValue + 1.0);
        cout << "Using a signal probability of " << signalProb 
             << " (" << (signalProb * 100) << "% of buffers will contain a signal)." << endl;
    }
    else if (mode == 2) {
        // In start estimation mode, only signal buffers are analyzed.
        signalProb = 1.0;
        cout << "\n[Start Estimation Mode] Only signal buffers will be analyzed." << endl;
        
        cout << "\nEnter desired confidence level percentage (e.g., 95 for 95%): ";
        string conf_input;
        cin >> conf_input;
        double confLevel = stod(conf_input);
        
        cout << "\nEnter the maximum number of binary search steps to use: ";
        string steps_input;
        cin >> steps_input;
        int maxSteps = stoi(steps_input);
        
        cout << "\nEnter the total number of buffers to analyze: ";
        string buf_input;
        cin >> buf_input;
        if (buf_input == "q" || buf_input == "Q")
            return 0;
        unsigned int numBuffers = stoul(buf_input);
        
        while (true) {
            cout << "\nEnter a threshold value (or 'q' to quit): ";
            string thresh_input;
            cin >> thresh_input;
            if (thresh_input == "q" || thresh_input == "Q")
                break;
            uint64_t threshold = stoull(thresh_input);
            run_simulation_start_estimation(threshold, numBuffers, confLevel, maxSteps);
        }
        return 0;
    }
    else {
        cout << "Invalid mode selected. Exiting." << endl;
        return 1;
    }
    
    // For mode 1.
    cout << "\nEnter the total number of buffers to analyze: ";
    string buf_input;
    cin >> buf_input;
    if (buf_input == "q" || buf_input == "Q")
        return 0;
    unsigned int numBuffers = stoul(buf_input);
    
    while (true) {
        cout << "\nEnter a threshold value (or 'q' to quit): ";
        string thresh_input;
        cin >> thresh_input;
        if (thresh_input == "q" || thresh_input == "Q")
            break;
        uint64_t threshold = stoull(thresh_input);
        run_simulation_threshold(threshold, signalProb, numBuffers);
    }
    return 0;
}
