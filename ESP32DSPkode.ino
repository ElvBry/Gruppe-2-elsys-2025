#include <Arduino.h>
#include "esp_timer.h"
#include <math.h>


//program that calculates 

struct point {
  double x, y, z;
};

// Structure to pass receiver information to the ISR.
struct ReceiverInfo {
  uint8_t index; // Index into the global arrays.
  uint8_t pin;   // The receiver pin number.
};

//------------------------
// Global Constants & Variables
//------------------------

const uint8_t receiverPins[] = {14, 27, 26, 25};

// Global volatile array to hold the timestamp (in microseconds) for each receiver.
volatile uint64_t receiverTimestamps[4] = {0, 0, 0, 0};

// Global array holding the known 3D positions of the receivers
point RECEIVER_ARRAY[4] = {
  {0.0, 0.0, 0.0},
  {1.0, 0.0, 0.0},
  {0.0, 1.0, 0.0},
  {0.0, 0.0, 1.0}
};

// Signal propagation speed (speed of sound in air m/s) change based on medium and chosen distance unit in RECEIVER_ARRAY.
double propagationSpeed = 340.0;

// An initial guess for the transmitter's position
point initialGuess = {0.5, 0.5, 0.5};

// Timeout for a measurement cycle (in microseconds)
const uint64_t measurementTimeout = 5000;  // 5 ms


uint64_t measurementStartTime = 0;

//------------------------
// ISR & Interrupt Attachment
//------------------------

// Interrupt Service Routine (ISR) for receiver pins.
// The ISR receives a pointer to a ReceiverInfo structure.
// It records the current time (in microseconds) into the corresponding entry
// in the global receiverTimestamps array and then detaches the interrupt for that pin.
void IRAM_ATTR isrHandler(void *arg) {
  ReceiverInfo *info = (ReceiverInfo *) arg;
  uint8_t idx = info->index;
  // If this receiver hasn't been triggered yet, record the timestamp.
  if (receiverTimestamps[idx] == 0) {
    receiverTimestamps[idx] = esp_timer_get_time();
    detachInterrupt(info->pin); // Detach the interrupt so that noise doesn't overwrite the value.
  }
}

// Templated helper function to attach interrupts on an array of receiver pins.
// This function uses a static array of ReceiverInfo structures (one per pin)
// and calls attachInterruptArg() for each pin.
template <size_t N>
void attachReceiverInterrupts(const uint8_t (&pins)[N], volatile uint64_t (&timestamps)[N]) {
  static ReceiverInfo receiverInfoArray[N]; // This array persists across calls.
  for (size_t i = 0; i < N; i++) {
    pinMode(pins[i], INPUT_PULLUP);  // Configure the pin with an internal pull-up.
    receiverInfoArray[i].index = i;
    receiverInfoArray[i].pin = pins[i];
    attachInterruptArg(pins[i], isrHandler, (void *)&receiverInfoArray[i], RISING); // Attach the ISR; the ISR will receive a pointer to the corresponding ReceiverInfo.
  }
}

//------------------------
// Utility Functions
//------------------------

double distance(point a, point b) {
  return sqrt((b.x - a.x) * (b.x - a.x) +
              (b.y - a.y) * (b.y - a.y) +
              (b.z - a.z) * (b.z - a.z));
}


void calculateTDOAposition3D(point RECEIVER_ARRAY[4], double TDOA_ARRAY[4], double propagationSpeed, point initialGuess) {// Perform a 3D TDOA calculation using the Newton–Raphson method and print the x,y,z coordinates in CSV format
    // Compute measured range differences using receiver 0 as the reference.
    double d12 = propagationSpeed * (TDOA_ARRAY[0] - TDOA_ARRAY[1]);
    double d13 = propagationSpeed * (TDOA_ARRAY[0] - TDOA_ARRAY[2]);
    double d14 = propagationSpeed * (TDOA_ARRAY[0] - TDOA_ARRAY[3]);

    // Initialize the guess.
    double x = initialGuess.x;
    double y = initialGuess.y;
    double z = initialGuess.z;

    const int maxIterations = 100;
    const double tol = 1e-6;  // Convergence tolerance.

    for (int iter = 0; iter < maxIterations; ++iter) {
        point guessPoint = {x, y, z};

        // Compute distances from the guess to each receiver.
        double d1 = distance(guessPoint, RECEIVER_ARRAY[0]);
        double d2 = distance(guessPoint, RECEIVER_ARRAY[1]);
        double d3 = distance(guessPoint, RECEIVER_ARRAY[2]);
        double d4 = distance(guessPoint, RECEIVER_ARRAY[3]);

        // Prevent division by zero.
        if (d1 < 1e-9 || d2 < 1e-9 || d3 < 1e-9 || d4 < 1e-9) {
            return;
        }
        // Define the nonlinear functions:
        // f = d1 - d2 - d12,  g = d1 - d3 - d13,  h = d1 - d4 - d14.
        double f_val = d1 - d2 - d12;
        double g_val = d1 - d3 - d13;
        double h_val = d1 - d4 - d14;

        // Compute partial derivatives for the Jacobian.
        double df_dx = ((x - RECEIVER_ARRAY[0].x) / d1) - ((x - RECEIVER_ARRAY[1].x) / d2);
        double df_dy = ((y - RECEIVER_ARRAY[0].y) / d1) - ((y - RECEIVER_ARRAY[1].y) / d2);
        double df_dz = ((z - RECEIVER_ARRAY[0].z) / d1) - ((z - RECEIVER_ARRAY[1].z) / d2);

        double dg_dx = ((x - RECEIVER_ARRAY[0].x) / d1) - ((x - RECEIVER_ARRAY[2].x) / d3);
        double dg_dy = ((y - RECEIVER_ARRAY[0].y) / d1) - ((y - RECEIVER_ARRAY[2].y) / d3);
        double dg_dz = ((z - RECEIVER_ARRAY[0].z) / d1) - ((z - RECEIVER_ARRAY[2].z) / d3);

        double dh_dx = ((x - RECEIVER_ARRAY[0].x) / d1) - ((x - RECEIVER_ARRAY[3].x) / d4);
        double dh_dy = ((y - RECEIVER_ARRAY[0].y) / d1) - ((y - RECEIVER_ARRAY[3].y) / d4);
        double dh_dz = ((z - RECEIVER_ARRAY[0].z) / d1) - ((z - RECEIVER_ARRAY[3].z) / d4);

        // Assemble the Jacobian matrix.
        double a11 = df_dx, a12 = df_dy, a13 = df_dz;
        double a21 = dg_dx, a22 = dg_dy, a23 = dg_dz;
        double a31 = dh_dx, a32 = dh_dy, a33 = dh_dz;

        // Compute determinant of the Jacobian.
        double det = a11 * (a22 * a33 - a23 * a32)
                   - a12 * (a21 * a33 - a23 * a31)
                   + a13 * (a21 * a32 - a22 * a31);
        if (fabs(det) < 1e-9) {
            break; // Singular Jacobian; abort iteration.
        }

        // Compute the adjugate of the Jacobian.
        double A11 =  (a22 * a33 - a23 * a32);
        double A12 = -(a21 * a33 - a23 * a31);
        double A13 =  (a21 * a32 - a22 * a31);
        double A21 = -(a12 * a33 - a13 * a32);
        double A22 =  (a11 * a33 - a13 * a31);
        double A23 = -(a11 * a32 - a12 * a31);
        double A31 =  (a12 * a23 - a13 * a22);
        double A32 = -(a11 * a23 - a13 * a21);
        double A33 =  (a11 * a22 - a12 * a21);

        // Newton–Raphson update.
        double dx = -(A11 * f_val + A12 * g_val + A13 * h_val) / det;
        double dy = -(A21 * f_val + A22 * g_val + A23 * h_val) / det;
        double dz = -(A31 * f_val + A32 * g_val + A33 * h_val) / det;

        x += dx;
        y += dy;
        z += dz;

        // Check for convergence.
        if (sqrt(dx * dx + dy * dy + dz * dz) < tol)
            break;
    }
    // For this example, print the result in CSV format.
    Serial.print(x, 6);
    Serial.print(",");
    Serial.print(y, 6);
    Serial.print(",");
    Serial.println(z, 6);
}

//------------------------
// Main Program
//------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("TDOA Measurement System Starting...");

  attachReceiverInterrupts(receiverPins, receiverTimestamps);
  measurementStartTime = esp_timer_get_time();
}

void loop() {
  bool allTimestampsSet = true;
  for (size_t i = 0; i < 4; i++) {
    if (receiverTimestamps[i] == 0) {
      allTimestampsSet = false;
      break;
    }
  }

  uint64_t currentTime = esp_timer_get_time();

  if (allTimestampsSet || (currentTime - measurementStartTime > measurementTimeout)) { // If all values are set OR if a timeout has elapsed...
    if (!allTimestampsSet) {  // Only proceed if at least one receiver was triggered.
      Serial.println("Measurement timeout reached; discarding incomplete data.");
    } else {
      // Convert the volatile timestamps into a double array for TDOA calculation.
      double TDOA_ARRAY[4];
      for (size_t i = 0; i < 4; i++) {
        TDOA_ARRAY[i] = static_cast<double>(receiverTimestamps[i]);
      }
      calculateTDOAposition3D(RECEIVER_ARRAY, TDOA_ARRAY, propagationSpeed, initialGuess); // Call the TDOA calculation function.
    }
    
    
    for (size_t i = 0; i < 4; i++) {// Reset timestamps for the next measurement cycle.
      receiverTimestamps[i] = 0;
    }
    attachReceiverInterrupts(receiverPins, receiverTimestamps); // Reattach interrupts to all receiver pins.
    measurementStartTime = esp_timer_get_time();  // Restart the measurement timer.
  }
  delay(10); // makes the program more stable, should be less than measurementTimeout for real applications
}

