// Week 4, Step 2: press two or more pads at once -> the LED flashes white.
//
// Wiring (already done for you):
//   ESP32 GPIO21 -> SDA, GPIO22 -> SCL -> PCA9548 mux (address 0x70)
//   Four MLX90393 magnetometers (all address 0x18) on mux channels 0, 2, 5, 7
//   One NeoPixel (WS2812 RGB LED): DIN -> GPIO4
//
// This is the finished Step 1 code plus a new part in loop(): counting how
// many pads are pressed. Fill in the blanks marked ___ .
// Error "'___' was not declared" = a blank is still empty.
//
//   0 pressed -> off
//   1 pressed -> that sensor's color
//   2+ pressed -> flashing white
//
// This is also the file you extend with Cursor in Step 3.

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
constexpr uint8_t CHANNELS[NUM_SENSORS] = {0, 2, 5, 7};

// Colors are written 0xRRGGBB: two hex digits each for red, green, blue.
constexpr uint32_t COLORS[NUM_SENSORS] = {
    0xFF0000,  // sensor 0: red
    0x00FF00,  // sensor 1: green
    0x0000FF,  // sensor 2: blue
    0xFFFF00,  // sensor 3: yellow (red + green)
};
constexpr uint32_t WHITE = 0xFFFFFF;
constexpr uint32_t OFF = 0x000000;

// How big a change (in microtesla) counts as "pressed". Use your Step 1 value.
constexpr float THRESHOLD_UT = 100.0;

// How often to re-record the baseline of sensors nobody is pressing.
constexpr unsigned long REBASELINE_MS = 10000;  // 10 seconds

// How long the white flash stays on, then off, then on...
constexpr unsigned long FLASH_MS = 150;

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
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void showColor(uint32_t color) {
  pixel.setPixelColor(0, color);
  pixel.show();
}

// Read sensor i into x[i], y[i], z[i]. Returns false if the read failed.
bool readSensor(uint8_t i) {
  muxSelectChannel(CHANNELS[i]);  // the mux picks which 0x18 we talk to
  return sensors[i].readData(&x[i], &y[i], &z[i]);
}

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

// Re-record the baseline of every sensor that isn't pressed (drift fix).
void rebaselineIdleSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    if (sensorOk[i] && !pressed[i]) {
      baseX[i] = x[i];
      baseY[i] = y[i];
      baseZ[i] = z[i];
    }
  }
  Serial.println("--- re-baselined ---");
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

void loop() {
  // Read every sensor through the mux (your Step 1 code).
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    if (!sensorOk[i] || !readSensor(i)) {
      pressed[i] = false;
      continue;
    }
    float dx = x[i] - baseX[i];
    float dy = y[i] - baseY[i];
    float dz = z[i] - baseZ[i];
    change[i] = sqrt(dx * dx + dy * dy + dz * dz);
    pressed[i] = change[i] > THRESHOLD_UT;
  }

  // ---------------------------------------------------------- your turn ---
  // TODO 1: count how many sensors are pressed.
  //   Hint: start at 0 and add one for every pressed sensor.
  //   `++numPressed` adds one to numPressed.
  //   Format: if (pressed[i]) ...;
  uint8_t numPressed = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    if (pressed[i]) ___;
  }

  // TODO 2: "two or more" pressed?
  //   Hint: use >= with the smallest count that should flash.
  if (numPressed >= ___) {
    // TODO 3: flash without delay().
    //   delay() would freeze the whole program, so no sensors get read while
    //   it waits. Instead, use the clock: millis() / FLASH_MS counts up by one
    //   every FLASH_MS ms (0, 1, 2, 3, ...). The remainder after dividing by
    //   some number goes 0, 1, 0, 1, ... and that's our on/off.
    //   Format: (millis() / FLASH_MS) % ... == 0   (which number gives 0,1,0,1?)
    bool flashOn = (millis() / FLASH_MS) % ___ == 0;

    // TODO 4: white when flashOn, off when not.
    //   Hint: `a ? b : c` means "if a then b, otherwise c".
    showColor(flashOn ? WHITE : ___);
  } else if (numPressed == 1) {
    // Exactly one pressed: show its color.
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
      if (pressed[i]) showColor(COLORS[i]);
    }
  } else {
    showColor(OFF);
  }
  // ---------------------------------------------------------------------

  // Drift fix (your Step 1 code, now moved into a helper).
  if (millis() - lastRebaselineMs >= REBASELINE_MS) {
    rebaselineIdleSensors();
    lastRebaselineMs = millis();
  }

  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    Serial.printf("s%d:%.1f ", i, change[i]);
  }
  Serial.printf("threshold:%.1f pressed:%d\n", THRESHOLD_UT, numPressed);

  delay(20);
}
