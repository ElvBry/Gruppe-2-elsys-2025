#include "esp_timer.h"
#include <cmath>
#include "soc/gpio_reg.h"
#include "soc/gpio_struct.h"


#define NUM_RECEIVERS 4

struct point {
  double x, y, z;
};


// Structure to hold a pin event with its delay in microseconds.
typedef struct {
  uint8_t pin;
  uint32_t delay_us;
} PinEvent;


// Set the given pin high quickly.
inline void setPinHigh(uint8_t pin) {
  if(pin < 32) {
    GPIO.out_w1ts = (1UL << pin);  // Write 1 to set the bit
  } else {
    GPIO.out1_w1ts.val = (1UL << (pin - 32));
  }
}

// Set the given pin low quickly.
inline void setPinLow(uint8_t pin) {
  if(pin < 32) {
    GPIO.out_w1tc = (1UL << pin);  // Write 1 to clear the bit
  } else {
    GPIO.out1_w1tc.val = (1UL << (pin - 32));
  }
}

double distance(point a, point b) {
  return sqrt((b.x - a.x) * (b.x - a.x) +
              (b.y - a.y) * (b.y - a.y) +
              (b.z - a.z) * (b.z - a.z));
}

// define receiver pins that send data to receivers
const uint8_t RECEIVER_PIN_ARRAY[NUM_RECEIVERS] = {16, 17, 5, 18};

// define the positions of the recievers
const point RECEIVER_ARRAY[NUM_RECEIVERS] = {
  {0.5, 0.3, 0.45},
  {0.2, 0.47, 0.45},
  {0.2, 0.13, 0.45},
  {0.33, 0.33, 0.12}
};

// insert positions to send pings from
const point TRANSMITTER_ARRAY[] = {
 {0.433, 0.346, 0.285},
 {0.294, 0.330, 0.188},
 {0.152, 0.184, 0.326},
 {0.365, 0.480, 0.315},
 {0.261, 0.262, 0.263},
 {0.315, 0.297, 0.153},
 {0.355, 0.124, 0.287},
 {0.356, 0.320, 0.393},
 {0.190, 0.293, 0.396},
 {0.448, 0.337, 0.193},
 {0.199, 0.409, 0.203},
 {0.173, 0.293, 0.152},
 {0.251, 0.131, 0.241},
 {0.424, 0.241, 0.198},
 {0.383, 0.353, 0.344},
 {0.175, 0.306, 0.260},
 {0.279, 0.178, 0.314},
 {0.375, 0.390, 0.359},
 {0.318, 0.374, 0.462},
 {0.256, 0.251, 0.160},
 {0.283, 0.146, 0.366},
 {0.201, 0.257, 0.179},
 {0.332, 0.302, 0.164},
 {0.244, 0.371, 0.128},
 {0.168, 0.315, 0.173},
 {0.449, 0.266, 0.398},
 {0.245, 0.397, 0.280},
 {0.344, 0.252, 0.224},
 {0.337, 0.348, 0.132},
 {0.186, 0.187, 0.256},
 {0.447, 0.388, 0.288},
 {0.416, 0.326, 0.202},
 {0.416, 0.394, 0.310},
 {0.233, 0.240, 0.446},
 {0.309, 0.436, 0.319},
 {0.190, 0.200, 0.350},
 {0.177, 0.268, 0.309},
 {0.274, 0.262, 0.423},
 {0.333, 0.186, 0.394},
 {0.231, 0.377, 0.153}
};
uint32_t TRANSMITTER_AMOUNT = sizeof(TRANSMITTER_ARRAY)/sizeof(TRANSMITTER_ARRAY[0]);

uint8_t transmitterPtr = 0;
const double propagationSpeed = 0.00034; // Speed of sound in m/µs

void setup() {
  //Serial.begin(115200);
  for (int i = 0; i < NUM_RECEIVERS; i++) {
    pinMode(RECEIVER_PIN_ARRAY[i], OUTPUT);
    setPinLow(RECEIVER_PIN_ARRAY[i]);
  }
}

void loop() {
  PinEvent events[NUM_RECEIVERS];

  for (int i = 0; i < NUM_RECEIVERS; i++) {
    events[i].pin = RECEIVER_PIN_ARRAY[i];
    double d = distance(TRANSMITTER_ARRAY[transmitterPtr], RECEIVER_ARRAY[i]);
    events[i].delay_us = static_cast<uint32_t>(d / propagationSpeed);
  }


  int32_t currentTime = min(
    min(events[0].delay_us,events[1].delay_us),
    min(events[2].delay_us,events[3].delay_us)
  );

  /*
  Serial.println("t1:" + String(events[0].delay_us-currentTime));
  Serial.println("t2:" + String(events[1].delay_us-currentTime));
  Serial.println("t3:" + String(events[2].delay_us-currentTime));
  Serial.println("t4:" + String(events[3].delay_us-currentTime));
  */


  for (int i = 0; i < NUM_RECEIVERS - 1; i++) {
    for (int j = i + 1; j < NUM_RECEIVERS; j++) {
      if (events[j].delay_us < events[i].delay_us) {
        PinEvent temp = events[i];
        events[i] = events[j];
        events[j] = temp;
      }
    }
  }

  uint64_t startTime = esp_timer_get_time();

  for (int i = 0; i < NUM_RECEIVERS; i++) {
    uint64_t targetTime = startTime + events[i].delay_us;
    while (esp_timer_get_time() < targetTime) {}
    setPinHigh(events[i].pin);
  }

  delayMicroseconds(100);
  for (int i = 0; i < NUM_RECEIVERS; i++) {
    setPinLow(RECEIVER_PIN_ARRAY[i]);
  }

  transmitterPtr = (transmitterPtr + 1) % TRANSMITTER_AMOUNT;

  delay(100);
}
