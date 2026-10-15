// Week 4, Step 1: four magnetometers, four colors.
//
// Wiring (already done for you):
//   ESP32 GPIO21 -> SDA, GPIO22 -> SCL -> PCA9548 mux (address 0x70)
//   Four MLX90393 magnetometers (all address 0x18) on mux channels 0, 2, 5, 7
//   One NeoPixel (WS2812 RGB LED): DIN -> GPIO4
//
// Press a pad and the NeoPixel shows that sensor's color:
//   sensor 0 = red, sensor 1 = green, sensor 2 = blue, sensor 3 = yellow
//
// The helpers and setup() are written for you. Fill in the blanks marked ___ .
// Error "'___' was not declared" = a blank is still empty.
//
// Watch the serial monitor: s0..s3 are each sensor's change from its baseline.
// Press EN to restart and re-record every baseline.

#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t MUX_ADDR = 0x70;
constexpr uint8_t MLX90393_ADDR = 0x18;
constexpr uint8_t PIXEL_PIN = 4;

constexpr uint8_t NUM_SENSORS = 4;
// Which mux channel each sensor is plugged into.
// Sensor 0 is on CHANNELS[0] = 0, sensor 1 is on CHANNELS[1] = 2, and so on.
constexpr uint8_t CHANNELS[NUM_SENSORS] = {0, 2, 5, 7};

// Colors are written 0xRRGGBB: two hex digits each for red, green, blue.
constexpr uint32_t COLORS[NUM_SENSORS] = {
    0xFF0000,  // sensor 0: red
    0x00FF00,  // sensor 1: green
    0x0000FF,  // sensor 2: blue
    0xFFFF00,  // sensor 3: yellow (red + green)
};
constexpr uint32_t OFF = 0x000000;

// How big a change (in microtesla) counts as "pressed". Tune this:
// pick a number above the resting wiggle and below a light press.
constexpr float THRESHOLD_UT = 100.0;

// How often to re-record the baseline of sensors nobody is pressing.
constexpr unsigned long REBASELINE_MS = 10000;  // 10 seconds

Adafruit_MLX90393 sensors[NUM_SENSORS];  // one object per magnetometer
Adafruit_NeoPixel pixel(1, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool sensorOk[NUM_SENSORS];  // false if the sensor didn't answer at startup
float x[NUM_SENSORS], y[NUM_SENSORS], z[NUM_SENSORS];  // latest readings
float baseX[NUM_SENSORS], baseY[NUM_SENSORS], baseZ[NUM_SENSORS];
float change[NUM_SENSORS];
bool pressed[NUM_SENSORS];
unsigned long lastRebaselineMs = 0;

// ---------------------------------------------------------------- given ---
void muxSelectChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);  // your Week 3 answer
  Wire.endTransmission();
}

void showColor(uint32_t color) {
  pixel.setPixelColor(0, color);
  pixel.show();
}

// ------------------------------------------------------------ your turn ---
// Read sensor i into x[i], y[i], z[i]. Returns false if the read failed.
bool readSensor(uint8_t i) {
  // TODO 1: connect the mux to sensor i before talking to address 0x18.
  //   Hint: all four sensors answer at 0x18, so the mux decides which one
  //   hears us. Sensor i is plugged into channel CHANNELS[i] (not channel i!).
  //   Format: muxSelectChannel(<the channel sensor i is on>);
  muxSelectChannel(___);
  return sensors[i].readData(&x[i], &y[i], &z[i]);
}

// ---------------------------------------------------------------- given ---
// Average 20 readings of every sensor while nobody touches the pads.
void recordBaseline() {
  const int samples = 20;
  int good[NUM_SENSORS] = {0};
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) baseX[i] = baseY[i] = baseZ[i] = 0;

  for (int s = 0; s < samples; ++s) {
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
      if (sensorOk[i] && readSensor(i)) {
        baseX[i] += x[i];
        baseY[i] += y[i];
        baseZ[i] += z[i];
        ++good[i];
      }
    }
    delay(20);
  }
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    if (good[i] > 0) {
      baseX[i] /= good[i];
      baseY[i] /= good[i];
      baseZ[i] /= good[i];
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  pixel.begin();
  pixel.setBrightness(40);  // 0-255; full brightness is blinding
  showColor(OFF);

  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    muxSelectChannel(CHANNELS[i]);
    sensorOk[i] = sensors[i].begin_I2C(MLX90393_ADDR, &Wire);
    if (!sensorOk[i]) {
      Serial.printf("Sensor %d (channel %d) not found - it will be ignored.\n",
                    i, CHANNELS[i]);
      continue;
    }
    sensors[i].setResolution(MLX90393_X, MLX90393_RES_16);
    sensors[i].setResolution(MLX90393_Y, MLX90393_RES_16);
    sensors[i].setResolution(MLX90393_Z, MLX90393_RES_16);
    sensors[i].setOversampling(MLX90393_OSR_1);
    sensors[i].setFilter(MLX90393_FILTER_3);
  }

  Serial.println("Recording baseline - don't touch the pads...");
  recordBaseline();
  lastRebaselineMs = millis();
  Serial.println("Baseline recorded. Press the pads!");
}

// ------------------------------------------------------------ your turn ---
void loop() {
  // TODO 2: visit every sensor, one after the other.
  //   Hint: sensors are numbered 0, 1, 2, 3. There's a constant for "how many".
  //   Format: for (uint8_t i = 0; i < ...; ++i)
  for (uint8_t i = 0; i < ___; ++i) {
    if (!sensorOk[i] || !readSensor(i)) {
      pressed[i] = false;  // missing or failed sensor never counts as pressed
      continue;
    }
    float dx = x[i] - baseX[i];
    float dy = y[i] - baseY[i];
    float dz = z[i] - baseZ[i];
    change[i] = sqrt(dx * dx + dy * dy + dz * dz);  // same as Week 3

    // TODO 3: is sensor i pressed?
    //   Hint: same rule as Week 3, but use change[i] instead of change.
    pressed[i] = ___;
  }

  // TODO 4: pick the color of a pressed sensor.
  //   Hint: COLORS[i] is sensor i's color. If nothing is pressed, color stays OFF.
  //   (If two are pressed, the higher-numbered one wins. Step 2 fixes that.)
  uint32_t color = OFF;
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    if (pressed[i]) color = ___;
  }
  showColor(color);

  // Drift: the "resting" reading slowly wanders (temperature, the band shifting
  // on the arm, turning your arm in Earth's magnetic field). So every
  // REBASELINE_MS we take the current reading as the new resting value.
  //
  // TODO 5: has it been at least REBASELINE_MS since the last re-baseline?
  //   Hint: millis() is "milliseconds since power-on". Now minus last time
  //   = how long it's been.
  //   Format: if (millis() - lastRebaselineMs >= ...)
  if (millis() - lastRebaselineMs >= ___) {
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
      if (!sensorOk[i]) continue;
      // TODO 6: only re-baseline a sensor that is NOT pressed right now.
      //   Hint: if you reset a sensor while someone presses it, the press
      //   becomes the new "normal" and the LED turns off mid-press.
      //   `!` means "not".  Format: if (... pressed[i])
      if (___) {
        baseX[i] = x[i];
        baseY[i] = y[i];
        baseZ[i] = z[i];
      }
    }
    lastRebaselineMs = millis();
    Serial.println("--- re-baselined ---");
  }

  // Print one line per loop: s0:12.3 s1:4.5 ... (works in Serial Plotter too)
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    Serial.printf("s%d:%.1f ", i, change[i]);
  }
  Serial.printf("threshold:%.1f\n", THRESHOLD_UT);

  delay(20);
}
