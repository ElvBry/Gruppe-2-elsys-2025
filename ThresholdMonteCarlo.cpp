#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <cstdint>
#include <cstdlib>
#include <string>


using namespace std;

const int SAMPLE_LENGTH = 60;
const int REF_TABLE_SIZE = 10;
const int NUM_TESTS_PER_SET = 10;   // Total buffers per simulation set
const int NUM_SIMULATIONS = 100000;

const double ADC_REF_VOLTAGE = 3.3;
const int ADC_MAX_VALUE = 4095;
const double ADC_STEP = ADC_REF_VOLTAGE / ADC_MAX_VALUE; // ~0.00080586 V per count

const double NOISE_AMPLITUDE_MV = 5.0;
const double BURST_AMPLITUDE_MV = 10.0;
const int NOISE_AMPLITUDE_COUNTS = static_cast<int>(round(NOISE_AMPLITUDE_MV / (ADC_STEP * 1000)));
const int BURST_AMPLITUDE_COUNTS = static_cast<int>(round(BURST_AMPLITUDE_MV / (ADC_STEP * 1000)));

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

uint64_t calculate_signal_strength(const vector<int16_t>& sigArr) {
    int32_t R = 0, I = 0;
    for (int i = 0; i < REF_TABLE_SIZE; i++) {
        R += sigArr[i] * ref_cos_table_Q15[i];
        I += sigArr[i] * ref_sine_table_Q15[i];
    }
    return static_cast<uint64_t>(R) * R + static_cast<uint64_t>(I) * I;
}

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

void run_simulation(uint64_t threshold, double signalProb) {
    uint64_t totalSignal = 0, totalNoise = 0;
    uint64_t truePos = 0, falseNeg = 0, trueNeg = 0, falsePos = 0;
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> burstDist(0, SAMPLE_LENGTH - 5);
    uniform_real_distribution<> probDist(0.0, 1.0);

    for (int sim = 0; sim < NUM_SIMULATIONS; sim++) {
        for (int test = 0; test < NUM_TESTS_PER_SET; test++) {
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
    }

    double pctTruePos = (totalSignal > 0) ? (100.0 * truePos / totalSignal) : 0.0;
    double pctFalseNeg = (totalSignal > 0) ? (100.0 * falseNeg / totalSignal) : 0.0;
    double pctTrueNeg = (totalNoise > 0) ? (100.0 * trueNeg / totalNoise) : 0.0;
    double pctFalsePos = (totalNoise > 0) ? (100.0 * falsePos / totalNoise) : 0.0;

    cout << "\nThreshold: " << threshold << "\n";
    cout << "Noise:Signal ratio: " << (1.0 / signalProb - 1) << ":1\n";
    cout << "Signal tests: " << totalSignal << "   Noise tests: " << totalNoise << "\n";
    cout << "True Positives: " << truePos << " (" << pctTruePos << "%)\n";
    cout << "False Negatives: " << falseNeg << " (" << pctFalseNeg << "%)\n";
    cout << "True Negatives: " << trueNeg << " (" << pctTrueNeg << "%)\n";
    cout << "False Positives: " << falsePos << " (" << pctFalsePos << "%)\n\n";
}

int main() {
    initialize_sine_table_Q15();
    initialize_cos_table_Q15();

    cout << "Enter noise:signal ratio (integer): ";
    string ratio_input;
    cin >> ratio_input;
    if (ratio_input == "q" || ratio_input == "Q")
        return 0;
    unsigned int ratioInt = stoul(ratio_input);
    // Compute signal probability: for a noise:signal ratio of X, signal probability = 1 / (X + 1)
    double signalProb = 1.0 / (ratioInt + 1.0);
    cout << "Using signal probability: " << signalProb << " (" << (signalProb * 100) << "% signal buffers)\n";

    while (true) {
        cout << "Enter threshold (or 'q' to quit): ";
        string input;
        cin >> input;
        if (input == "q" || input == "Q")
            break;
        uint64_t threshold = stoull(input);
        run_simulation(threshold, signalProb);
    }
    return 0;
}
