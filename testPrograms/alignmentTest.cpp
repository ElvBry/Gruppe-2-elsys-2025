#include <iostream>
#include <cstdint>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <random>
#include <algorithm>
using namespace std;

#define secondIndex 16
#define thirdIndex 32

// Structure to hold a single test result.
struct TestResult {
    uint64_t originalBuffer; // The generated 64-bit buffer (before rotation)
    int randomOffset;        // The random pointer offset applied
    bool alignmentSuccess;   // Whether alignment was found
    int extraCycles;         // Extra cycles (shift) performed (if success)
    uint64_t alignedBuffer;  // The final aligned buffer (as a 64-bit value)
    string alignedBinary;    // The final aligned buffer in binary (as string)
    bool isCorrect;          // Whether the aligned buffer has the expected header nibbles
};

// Convert a 64-element boolean array to a binary string.
string boolBufferToBinaryString(const bool buffer[64]) {
    string s;
    for (int i = 0; i < 64; i++) {
        s.push_back(buffer[i] ? '1' : '0');
    }
    return s;
}

// Convert a 64-element boolean array to a uint64_t value.
uint64_t boolBufferToHex(const bool buffer[64]) {
    uint64_t value = 0;
    for (int i = 0; i < 64; i++) {
        value = (value << 1) | (buffer[i] ? 1ULL : 0ULL);
    }
    return value;
}

// Check if the aligned buffer has correct header nibbles.
// For a correctly aligned buffer, the first 4 bits of each 16-bit word (indices 0–3, 16–19, 32–35, 48–51)
// should equal 0, 1, 2, and 3 respectively.
bool checkAlignedBuffer(const bool buffer[64]) {
    int expected[4] = {0, 1, 2, 3};
    for (int w = 0; w < 4; w++) {
        int start = w * 16;
        int nibble = (buffer[start] << 3) | (buffer[start+1] << 2) | (buffer[start+2] << 1) | buffer[start+3];
        if (nibble != expected[w])
            return false;
    }
    return true;
}

// Alignment algorithm.
// Given a 64-bit boolean buffer (already rotated by the initial pointer offset),
// it searches candidate starting positions (0 to 18) for a valid header sequence.
// When found, it computes extra_cycles = 64 - ((ID1*16 - candidate) mod 64),
// rotates the buffer by extra_cycles, and returns a TestResult.
TestResult runAlignment(const bool origBuffer[64], uint64_t originalValue, int initialOffset) {
    TestResult result;
    result.originalBuffer = originalValue;
    result.randomOffset = initialOffset;
    bool data_buffer[64];
    for (int i = 0; i < 64; i++) {
        data_buffer[i] = origBuffer[i];
    }
    
    bool found = false;
    int extra_cycles = 0;
    uint64_t alignedHex = 0;
    string alignedBin;
    
    // Try candidate offsets from 0 to 18.
    for (int i = 0; i < 19; i++) {
        uint8_t ID1, ID2, ID3;
        // Header nibble for first word: bits i, i+1, i+2, i+3.
        ID1 = (data_buffer[i] << 3) | (data_buffer[i+1] << 2) | (data_buffer[i+2] << 1) | data_buffer[i+3];
        // Header nibble for second word: bits i+16 ... i+19.
        ID2 = (data_buffer[i+secondIndex] << 3) | (data_buffer[i+secondIndex+1] << 2) |
              (data_buffer[i+secondIndex+2] << 1) | data_buffer[i+secondIndex+3];
        // Check expected sequence: ID2 should be (ID1 + 1) modulo wrap-around, and IDs must be in 0..3.
        if (((ID2 != ID1 + 1) && !(ID1 == 3 && ID2 == 0)) || (ID1 > 3 || ID2 > 3))
            continue;
        // Header nibble for third word: bits i+32 ... i+35.
        ID3 = (data_buffer[i+thirdIndex] << 3) | (data_buffer[i+thirdIndex+1] << 2) |
              (data_buffer[i+thirdIndex+2] << 1) | data_buffer[i+thirdIndex+3];
        if (((ID3 != ID2 + 1) && !(ID2 == 3 && ID3 == 0)) || (ID3 > 3))
            continue;
        
        // Valid header sequence found.
        int start_index = (ID1 * 16) - i;
        if (start_index < 0)
            start_index += 64;
        extra_cycles = 64 - start_index;
        
        // Rotate data_buffer by extra_cycles.
        bool aligned_buffer[64];
        for (int j = 0; j < 64; j++) {
            aligned_buffer[j] = data_buffer[(j + extra_cycles) % 64];
        }
        alignedHex = boolBufferToHex(aligned_buffer);
        alignedBin = boolBufferToBinaryString(aligned_buffer);
        found = true;
        break;
    }
    
    result.alignmentSuccess = found;
    if(found) {
        result.extraCycles = extra_cycles;
        result.alignedBuffer = alignedHex;
        result.alignedBinary = alignedBin;
        // Recreate a boolean array from alignedHex and check headers.
        bool finalBuffer[64];
        for (int j = 0; j < 64; j++) {
            finalBuffer[j] = ((alignedHex >> (63 - j)) & 0x1ULL) != 0;
        }
        result.isCorrect = checkAlignedBuffer(finalBuffer);
    } else {
        result.extraCycles = -1;
        result.alignedBuffer = 0;
        result.alignedBinary = "";
        result.isCorrect = false;
    }
    
    return result;
}

int main() {
    int numBuffers, numToShow;
    cout << "Enter number of buffers to generate: ";
    cin >> numBuffers;
    cout << "Enter number of results to display: ";
    cin >> numToShow;
    
    vector<TestResult> results;
    results.reserve(numBuffers);
    
    // Random number generator setup.
    random_device rd;
    mt19937 gen(rd());
    // For the three random hex digits (0 to 0xFFF) per word.
    uniform_int_distribution<int> hexDist(0, 0xFFF);
    // For random pointer offset.
    uniform_int_distribution<int> offsetDist(0, 63);
    
    // For each test, generate a buffer using the template 0x0???1???2???3???.
    for (int i = 0; i < numBuffers; i++) {
        uint16_t w0 = (0 << 12) | (hexDist(gen) & 0x0FFF);
        uint16_t w1 = (1 << 12) | (hexDist(gen) & 0x0FFF);
        uint16_t w2 = (2 << 12) | (hexDist(gen) & 0x0FFF);
        uint16_t w3 = (3 << 12) | (hexDist(gen) & 0x0FFF);
        uint64_t bufferVal = ((uint64_t)w0 << 48) | ((uint64_t)w1 << 32) | ((uint64_t)w2 << 16) | w3;
        
        int initialOffset = offsetDist(gen);
        
        // Convert bufferVal into a boolean array (MSB-first).
        bool baseBuffer[64];
        for (int j = 0; j < 64; j++) {
            baseBuffer[j] = ((bufferVal >> (63 - j)) & 0x1ULL) != 0;
        }
        
        // Apply the initial offset to simulate the ring buffer pointer.
        bool rotated[64];
        for (int j = 0; j < 64; j++) {
            rotated[j] = baseBuffer[(j + initialOffset) % 64];
        }
        
        // Run the alignment algorithm on the rotated buffer.
        TestResult tr = runAlignment(rotated, bufferVal, initialOffset);
        results.push_back(tr);
    }
    
    // Separate the results into correct and incorrect.
    vector<TestResult> correct, incorrect;
    for (const auto &r : results) {
        if (r.alignmentSuccess && r.isCorrect)
            correct.push_back(r);
        else
            incorrect.push_back(r);
    }
    
    // Sort (or simply display) the results.
    cout << "\n--- Correct Results ---" << endl;
    int n = min(numToShow, (int)correct.size());
    for (int i = 0; i < n; i++) {
        cout << "Buffer " << i << ": Orig=0x" << hex << setw(16) << setfill('0') << correct[i].originalBuffer
             << ", Offset=" << dec << correct[i].randomOffset
             << ", ExtraCycles=" << correct[i].extraCycles
             << ", Aligned=0x" << hex << setw(16) << setfill('0') << correct[i].alignedBuffer
             << ", Binary=" << correct[i].alignedBinary << endl;
    }
    
    cout << "\n--- Incorrect Results ---" << endl;
    n = min(numToShow, (int)incorrect.size());
    for (int i = 0; i < n; i++) {
        cout << "Buffer " << i << ": Orig=0x" << hex << setw(16) << setfill('0') << incorrect[i].originalBuffer
             << ", Offset=" << dec << incorrect[i].randomOffset
             << ", ExtraCycles=" << incorrect[i].extraCycles
             << ", Aligned=0x" << hex << setw(16) << setfill('0') << incorrect[i].alignedBuffer
             << ", Binary=" << incorrect[i].alignedBinary << endl;
    }
    
    return 0;
}
