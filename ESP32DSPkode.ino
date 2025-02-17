#include "esp_timer.h"
#include <math.h>

//------------------------
// Data Structures
//------------------------

struct point {
  double x, y, z;
};

struct ReceiverInfo {
  uint8_t index; // Index into the global arrays.
  uint8_t pin;   // The receiver pin number.
};

//------------------------
// Global Constants & Variables
//------------------------

const uint8_t receiverPins[] = {14, 27, 26, 25};

// Global volatile array to hold the timestamp (in microseconds) for each receiver.
volatile uint64_t RECEIVER_TIMESTAMPS[4] = {0, 0, 0, 0};

// Global array holding the known 3D positions of the receivers
// (Receiver 0 is the reference)
point RECEIVER_ARRAY[4] = {
  {0.0, 0.0, 0.0},
  {1.0, 1.0, 0.0},
  {0.0, 1.0, 0.0},
  {1.0, 0.6, 1.0}
};

double propagationSpeed = 0.00034; // Speed of sound in m/µs (340 m/s)
point initialGuess = {0.5, 0.5, 0.5};

const uint64_t measurementTimeout = 5;  // 5 ms timeout
uint64_t measurementStartTime = 0;

//------------------------
// ISR & Interrupt Attachment
//------------------------

void IRAM_ATTR isrHandler(void *arg) {
  ReceiverInfo *info = (ReceiverInfo *) arg;
  uint8_t idx = info->index;
  if (RECEIVER_TIMESTAMPS[idx] == 0) {
    RECEIVER_TIMESTAMPS[idx] = esp_timer_get_time();
    // Optionally detach interrupt here.
  }
}

template <size_t N>
void attachReceiverInterrupts(const uint8_t (&pins)[N], volatile uint64_t (&timestamps)[N]) {
  static ReceiverInfo receiverInfoArray[N]; // Persistent array
  for (size_t i = 0; i < N; i++) {
    pinMode(pins[i], INPUT_PULLUP);
    receiverInfoArray[i].index = i;
    receiverInfoArray[i].pin = pins[i];
    attachInterruptArg(pins[i], isrHandler, (void *)&receiverInfoArray[i], RISING);
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

double constrain_double(double value, double low, double high) {
  return fmin(fmax(value, low), high);
}

// Solve a 3x3 linear system J * delta = -F using Gaussian elimination with partial pivoting.
// J is a 3x3 matrix, F is a 3-element vector. The solution is written into delta.
// Returns true on success, false if the matrix is singular.
bool solve3x3(const double J[3][3], const double F[3], double delta[3]) {
  double A[3][4];
  // Build the augmented matrix A = [J | -F]
  for (int i = 0; i < 3; i++) {
    A[i][0] = J[i][0];
    A[i][1] = J[i][1];
    A[i][2] = J[i][2];
    A[i][3] = -F[i]; // because we want to solve for delta in J * delta = -F
  }
  // Gaussian elimination with partial pivoting.
  for (int i = 0; i < 3; i++) {
    // Find pivot row.
    int pivot = i;
    for (int j = i + 1; j < 3; j++) {
      if (fabs(A[j][i]) > fabs(A[pivot][i])) {
        pivot = j;
      }
    }
    if (fabs(A[pivot][i]) < 1e-12) {
      return false; // Singular matrix.
    }
    // Swap current row with pivot row.
    for (int k = i; k < 4; k++) {
      double temp = A[i][k];
      A[i][k] = A[pivot][k];
      A[pivot][k] = temp;
    }
    // Normalize pivot row.
    double pivotVal = A[i][i];
    for (int k = i; k < 4; k++) {
      A[i][k] /= pivotVal;
    }
    // Eliminate below.
    for (int j = i + 1; j < 3; j++) {
      double factor = A[j][i];
      for (int k = i; k < 4; k++) {
        A[j][k] -= factor * A[i][k];
      }
    }
  }
  // Back substitution.
  for (int i = 2; i >= 0; i--) {
    delta[i] = A[i][3];
    for (int j = i + 1; j < 3; j++) {
      delta[i] -= A[i][j] * delta[j];
    }
  }
  return true;
}

//------------------------
// Revised TDOA Calculation Function
//------------------------
//
// This function expects TDOA values (in microseconds) that are already zeroed
// with respect to Receiver 0 (which is treated as the reference).
//
void calculateTDOAposition3D(point RECEIVER_ARRAY[4], int32_t TDOA_ARRAY[4],
                             double propagationSpeed, point initialGuess) {
  // Compute measured range differences (in meters) using Receiver 0 as reference.
  double d12 = propagationSpeed * (static_cast<double>(TDOA_ARRAY[0] - TDOA_ARRAY[1]));
  double d13 = propagationSpeed * (static_cast<double>(TDOA_ARRAY[0] - TDOA_ARRAY[2]));
  double d14 = propagationSpeed * (static_cast<double>(TDOA_ARRAY[0] - TDOA_ARRAY[3]));
  d12 = constrain_double(d12, -1.0, 1.0);
  d13 = constrain_double(d13, -1.0, 1.0);
  d14 = constrain_double(d14, -1.0, 1.0);

  // Initialize transmitter guess.
  double x = initialGuess.x;
  double y = initialGuess.y;
  double z = initialGuess.z;

  const int maxIterations = 100;
  const double tol = 1e-6;  // Convergence tolerance.

  for (int iter = 0; iter < maxIterations; iter++) {
    point guess = { x, y, z };
    double d1 = distance(guess, RECEIVER_ARRAY[0]);
    double d2 = distance(guess, RECEIVER_ARRAY[1]);
    double d3 = distance(guess, RECEIVER_ARRAY[2]);
    double d4 = distance(guess, RECEIVER_ARRAY[3]);

    // Prevent division by zero.
    if (d1 < 1e-9 || d2 < 1e-9 || d3 < 1e-9 || d4 < 1e-9) {
      Serial.println("Error: Division by near zero.");
      return;
    }

    // Define the nonlinear functions:
    // f = d1 - d2 - d12, g = d1 - d3 - d13, h = d1 - d4 - d14.
    double f = d1 - d2 - d12;
    double g = d1 - d3 - d13;
    double h = d1 - d4 - d14;

    // Compute partial derivatives (Jacobian matrix entries).
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
    double J[3][3] = {
      { df_dx, df_dy, df_dz },
      { dg_dx, dg_dy, dg_dz },
      { dh_dx, dh_dy, dh_dz }
    };
    double F[3] = { f, g, h };
    double delta[3] = { 0, 0, 0 };

    // Solve J * delta = -F
    if (!solve3x3(J, F, delta)) {
      Serial.println("Error: Singular Jacobian encountered; aborting iteration.");
      return;
    }

    x += delta[0];
    y += delta[1];
    z += delta[2];

    double normDelta = sqrt(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]);
    if (normDelta < tol)
      break;
  }

  // Print estimated transmitter position.
  if (isnan(x) || isinf(x) || isnan(y) || isinf(y) || isnan(z) || isinf(z)) {
    Serial.println("Error: Calculation resulted in NaN or Infinity.");
  } else {
    Serial.print("Estimated Position: ");
    Serial.print(x, 6);
    Serial.print(", ");
    Serial.print(y, 6);
    Serial.print(", ");
    Serial.println(z, 6);
  }
}

//------------------------
// Main Program
//------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("TDOA Measurement System Starting...");
  attachReceiverInterrupts(receiverPins, RECEIVER_TIMESTAMPS);
  measurementStartTime = esp_timer_get_time();
}

void loop() {
  bool allTimestampsSet = true;
  for (size_t i = 0; i < 4; i++) {
    if (RECEIVER_TIMESTAMPS[i] == 0) {
      allTimestampsSet = false;
      break;
    }
  }

  // If all receivers have been triggered or timeout reached:
  if (allTimestampsSet || (esp_timer_get_time() - measurementStartTime > measurementTimeout)) {
    if (allTimestampsSet) {
      int32_t TDOA_ARRAY[4];
      uint64_t currentTime = min(
        min(RECEIVER_TIMESTAMPS[0], RECEIVER_TIMESTAMPS[1]),
        min(RECEIVER_TIMESTAMPS[2], RECEIVER_TIMESTAMPS[3])
      );
      for (size_t i = 0; i < 4; i++) {
        TDOA_ARRAY[i] = (int32_t)(RECEIVER_TIMESTAMPS[i] - currentTime);
      }
      // Call the revised TDOA calculation function.
      calculateTDOAposition3D(RECEIVER_ARRAY, TDOA_ARRAY, propagationSpeed, initialGuess);
      
      Serial.print("t1: ");
      Serial.println(TDOA_ARRAY[0]);
      Serial.print("t2: ");
      Serial.println(TDOA_ARRAY[1]);
      Serial.print("t3: ");
      Serial.println(TDOA_ARRAY[2]);
      Serial.print("t4: ");
      Serial.println(TDOA_ARRAY[3]);
    }
    // Reset timestamps for the next measurement cycle.
    for (size_t i = 0; i < 4; i++) {
      RECEIVER_TIMESTAMPS[i] = 0;
    }
    measurementStartTime = esp_timer_get_time();
  }
  delay(10);
}
