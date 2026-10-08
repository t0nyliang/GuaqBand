// Week 3, Step 2: turn the ESP32's LED on when magnetometer 0 sees a big change.
//
// Wiring (already done for you):
//   ESP32 GPIO21 -> SDA, GPIO22 -> SCL -> PCA9548 mux (address 0x70)
//   One MLX90393 magnetometer (address 0x18) on one mux channel
//   The blue LED on the board is wired to GPIO2.
//
// setup() is written for you. It connects to the sensor and records a
// "baseline": what the sensor reads while nobody is touching the pad.
// Your job is in loop(): fill in the blanks marked ___ .
// Error "'___' was not declared" = a blank is still empty.
//
// Watch `change` in the serial monitor. Press EN to re-record the baseline.

#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <math.h>

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t MUX_ADDR = 0x70;
constexpr uint8_t MLX90393_ADDR = 0x18;
constexpr uint8_t SENSOR_CHANNEL = 0;  // set to the channel week3_scan found
constexpr uint8_t LED_PIN = 2;         // onboard blue LED

// How big a change (in microtesla) counts as "pressed". Tune this:
// pick a number above the resting wiggle and below a light press.
constexpr float THRESHOLD_UT = 100.0;

Adafruit_MLX90393 sensor;
float baseX = 0, baseY = 0, baseZ = 0;

void muxSelectChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);  // your Step 1 answer
  Wire.endTransmission();
}

// ---------------------------------------------------------------- given ---
void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_PIN, OUTPUT);
  Wire.begin(SDA_PIN, SCL_PIN);

  muxSelectChannel(SENSOR_CHANNEL);
  while (!sensor.begin_I2C(MLX90393_ADDR, &Wire)) {
    Serial.println("Sensor not found. Run week3_scan and check SENSOR_CHANNEL.");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // fast blink = error
    delay(250);
  }
  sensor.setResolution(MLX90393_X, MLX90393_RES_16);
  sensor.setResolution(MLX90393_Y, MLX90393_RES_16);
  sensor.setResolution(MLX90393_Z, MLX90393_RES_16);
  sensor.setOversampling(MLX90393_OSR_1);
  sensor.setFilter(MLX90393_FILTER_3);

  // Average 20 readings while the pad is untouched.
  Serial.println("Recording baseline - don't touch the pad...");
  const int samples = 20;
  for (int i = 0; i < samples; ++i) {
    float x, y, z;
    sensor.readData(&x, &y, &z);
    baseX += x;
    baseY += y;
    baseZ += z;
    delay(20);
  }
  baseX /= samples;
  baseY /= samples;
  baseZ /= samples;
  Serial.println("Baseline recorded. Press the pad!");
}

// ------------------------------------------------------------ your turn ---
void loop() {
  float x, y, z;
  if (!sensor.readData(&x, &y, &z)) {
    Serial.println("read failed");
    delay(100);
    return;
  }

  // TODO 1: how far has each axis moved from its baseline?
  //   Hint: new reading minus baseline. dx is done for you as an example.
  float dx = x - baseX;
  float dy = ___;
  float dz = ___;

  // TODO 2: one number for the total change.
  //   Hint: length of an arrow = sqrt(a*a + b*b + c*c). Use dx, dy, dz.
  float change = ___;

  // TODO 3: true when the change is big enough to count as a press.
  //   Hint: compare `change` with THRESHOLD_UT using >, < or ==.
  bool pressed = ___;

  // TODO 4: LED on when pressed, off when not.
  //   Hint: digitalWrite(pin, HIGH) turns it on, LOW turns it off.
  //   Format: if (pressed) digitalWrite(LED_PIN, ...); else digitalWrite(LED_PIN, ...);
  ___;

  Serial.print("change: ");
  Serial.print(change);
  Serial.print(" uT   threshold: ");
  Serial.println(THRESHOLD_UT);

  delay(50);  // about 20 readings per second
}
