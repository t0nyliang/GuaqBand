// Week 3, Step 1: find the four magnetometers through the I2C mux.
//
// Wiring (already done for you):
//   ESP32 GPIO21 -> SDA, GPIO22 -> SCL -> PCA9548 mux (address 0x70)
//   One MLX90393 magnetometer (address 0x18) on one mux channel
//
// Fill in the two blanks marked ___ . Everything else is written for you.
// Error "'___' was not declared" = a blank is still empty.

#include <Wire.h>

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t MUX_ADDR = 0x70;
constexpr uint8_t MLX90393_ADDR = 0x18;

// Turn on exactly one mux channel (0-7). The mux reads one byte where each
// bit is a channel: bit 0 = channel 0, bit 5 = channel 5, and so on.
void muxSelectChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  // TODO 1: send a byte with only bit `channel` turned on.
  //   Hint: 1 << 3 is 0b00001000 (only bit 3 on). Use `channel` instead of 3.
  //   Format: Wire.write(1 << ...);
  Wire.write(___);
  Wire.endTransmission();
}

// Ask "is anyone at this address?" Wire.endTransmission() returns 0 when a
// device answered (an ACK) and a non-zero error code when nobody did.
bool devicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  // TODO 2: return true only when the device answered.
  //   Hint: Wire.endTransmission() gives back 0 when someone answered.
  //   Format: return Wire.endTransmission() ... 0;   (which comparison?)
  return ___;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println();
  Serial.println("Week 3 mux scan");
  Serial.print("Mux at 0x70: ");
  Serial.println(devicePresent(MUX_ADDR) ? "found" : "MISSING - check the cable to the mux");
}

void loop() {
  uint8_t found = 0;
  for (uint8_t channel = 0; channel < 8; ++channel) {
    muxSelectChannel(channel);
    if (devicePresent(MLX90393_ADDR)) {
      Serial.print("  channel ");
      Serial.print(channel);
      Serial.println(": MLX90393 at 0x18");
      ++found;
    }
  }
  Serial.print("Found ");
  Serial.print(found);
  Serial.println(" sensor(s). Expected 1: put its channel in SENSOR_CHANNEL in week3_led.ino");
  Serial.println();
  delay(2000);
}
