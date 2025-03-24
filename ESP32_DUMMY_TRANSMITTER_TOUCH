#include "esp_timer.h"
#include <cmath>
#include "soc/gpio_reg.h"
#include "soc/gpio_struct.h"


#define TOUCH_PIN_X 13
#define TOUCH_PIN_Y 12
#define TOUCH_PIN_Z 14
#define TOUCH_PIN_REVERSE 27

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

point TRANSMITTER_POSITION = {0.3,0.3,0.3};


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
    double d = distance(TRANSMITTER_POSITION, RECEIVER_ARRAY[i]);
    events[i].delay_us = static_cast<uint32_t>((d / propagationSpeed));
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
  TRANSMITTER_POSITION.x += (touchRead(TOUCH_PIN_REVERSE) < 20) ? -0.005 * (touchRead(TOUCH_PIN_X) < 20) : 0.005 * (touchRead(TOUCH_PIN_X) < 20);
  TRANSMITTER_POSITION.y += (touchRead(TOUCH_PIN_REVERSE) < 20) ? -0.005 * (touchRead(TOUCH_PIN_Y) < 20) : 0.005 * (touchRead(TOUCH_PIN_Y) < 20);
  TRANSMITTER_POSITION.z += (touchRead(TOUCH_PIN_REVERSE) < 20) ? -0.005 * (touchRead(TOUCH_PIN_Z) < 20) : 0.005 * (touchRead(TOUCH_PIN_Z) < 20);
  /*
  Serial.print(TRANSMITTER_POSITION.x, 3);
  Serial.print(",");
  Serial.print(TRANSMITTER_POSITION.y, 3);
  Serial.print(",");
  Serial.println(TRANSMITTER_POSITION.z, 3);
  */
  delay(50);
}
