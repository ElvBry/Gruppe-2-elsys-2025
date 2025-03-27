#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <sstream>

using namespace std;

// Constants
const int SAMPLE_LENGTH = 60;
const int REF_TABLE_SIZE = 10;

const double ADC_REF_VOLTAGE = 3.3;
const int ADC_MAX_VALUE = 4095;
const double ADC_STEP = ADC_REF_VOLTAGE / ADC_MAX_VALUE; // ≈0.00080586 V per count

const double NOISE_AMPLITUDE_MV = 5.0;
const double BURST_AMPLITUDE_MV = 10.0;
const int NOISE_AMPLITUDE_COUNTS = static_cast<int>(round(NOISE_AMPLITUDE_MV / (ADC_STEP * 1000)));
const int BURST_AMPLITUDE_COUNTS = static_cast<int>(round(BURST_AMPLITUDE_MV / (ADC_STEP * 1000)));

// Global reference tables (Q15 format)
vector<int16_t> ref_sine_table_Q15(REF_TABLE_SIZE);
vector<int16_t> ref_cos_table_Q15(REF_TABLE_SIZE);

void initialize_sine_table_Q15() {
    double angleStep = 2 * M_PI / REF_TABLE_SIZE;
    for (int i = 0; i < REF_TABLE_SIZE; i++) {
        double value = sin(i * angleStep) / REF_TABLE_SIZE;
        ref_sine_table_Q15[i] = static_cast<int16_t>(round(value * 32767));
    }
}

void initialize_cos_table_Q15() {
    double angleStep = 2 * M_PI / REF_TABLE_SIZE;
    for (int i = 0; i < REF_TABLE_SIZE; i++) {
        double value = cos(i * angleStep) / REF_TABLE_SIZE;
        ref_cos_table_Q15[i] = static_cast<int16_t>(round(value * 32767));
    }
}

// Computes signal strength as R² + I² over a window of REF_TABLE_SIZE samples.
uint64_t calculate_signal_strength(const vector<int16_t>& sigArr) {
    int32_t R = 0, I = 0;
    for (int i = 0; i < REF_TABLE_SIZE; i++) {
        R += sigArr[i] * ref_cos_table_Q15[i];
        I += sigArr[i] * ref_sine_table_Q15[i];
    }
    return static_cast<uint64_t>(R) * R + static_cast<uint64_t>(I) * I;
}

// Generates a simulated buffer with noise and, if specified, a sine burst.
vector<int16_t> simulate_signal(bool hasBurst, int burstStart) {
    vector<int16_t> signal(SAMPLE_LENGTH, 0);
    static random_device rd;
    static mt19937 gen(rd());
    uniform_int_distribution<> noise_dist(-NOISE_AMPLITUDE_COUNTS, NOISE_AMPLITUDE_COUNTS);
    for (int i = 0; i < SAMPLE_LENGTH; i++) {
        signal[i] = noise_dist(gen);
    }
    if (hasBurst) {
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

// Runs the simulation over a total number of buffers (numBuffers) and reports detection metrics.
void run_simulation(uint64_t threshold, double signalProb, unsigned int numBuffers) {
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

    cout << "\nThreshold: " << threshold << "\n";
    cout << "Simulation Conditions:\n";
    if (signalProb == 0.0) {
        cout << "  All buffers contain noise (0% signal)\n";
    } else if (signalProb == 1.0) {
        cout << "  All buffers contain signal (0% noise)\n";
    } else {
        double noiseToSignal = (1.0 - signalProb) / signalProb;
        cout << "  Noise-to-Signal Ratio: " << noiseToSignal << ":1\n";
    }
    cout << "Total Signal Buffers: " << totalSignal << "   Total Noise Buffers: " << totalNoise << "\n";
    cout << "True Positives: " << truePos << " (" << pctTruePos << "%)\n";
    cout << "False Negatives: " << falseNeg << " (" << pctFalseNeg << "%)\n";
    cout << "True Negatives: " << trueNeg << " (" << pctTrueNeg << "%)\n";
    cout << "False Positives: " << falsePos << " (" << pctFalsePos << "%)\n\n";
}

int main() {
    initialize_sine_table_Q15();
    initialize_cos_table_Q15();

    cout << "Enter the noise-to-signal ratio:" << endl;
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
    
    double signalProb = 0.0;
    if (ratioValue == 0) {
        signalProb = 0.0;
    } else if (ratioValue < 0) {
        int absVal = -ratioValue;
        signalProb = static_cast<double>(absVal) / (absVal + 1.0);
    } else {
        signalProb = 1.0 / (ratioValue + 1.0);
    }
    
    cout << "Using a signal probability of " << signalProb 
         << " (" << (signalProb * 100) << "% of buffers will contain a signal)." << endl;

    cout << "\nEnter the total number of buffers to analyze: ";
    string sim_input;
    cin >> sim_input;
    if (sim_input == "q" || sim_input == "Q")
        return 0;
    unsigned int numBuffers = stoul(sim_input);

    while (true) {
        cout << "\nEnter a threshold value (or 'q' to quit): ";
        string input;
        cin >> input;
        if (input == "q" || input == "Q")
            break;
        uint64_t threshold = stoull(input);
        run_simulation(threshold, signalProb, numBuffers);
    }
    return 0;
}
