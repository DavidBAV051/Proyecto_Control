#pragma once
#include <Arduino.h>

// =====================================================================
// Centralized GPIO pin map (ESP32, mhetesp32devkit ~30 usable GPIOs)
//
// Budget:
//   Motors (4x IN1/IN2 + encoder A/B)........ 16 pins
//   HuskyLens UART2 (RX2/TX2)................. 2 pins
//   Shared I2C bus (BNO055 IMU + ToF array).... 2 pins
//   Proximity sensor XSHUT (address select).... 4 pins
//   ------------------------------------------------
//   Total...................................... 24 pins
// =====================================================================

// ---- Motor 1 ----
#define M1_IN1_PIN   18
#define M1_IN2_PIN   19
#define M1_ENC_A_PIN 34   // input-only, needs external 10k pull-up to 3.3V
#define M1_ENC_B_PIN 35   // input-only, needs external 10k pull-up to 3.3V

// ---- Motor 2 ----
#define M2_IN1_PIN   26
#define M2_IN2_PIN   27
#define M2_ENC_A_PIN 36   // VP/SENSOR_VP, input-only, needs external 10k pull-up
#define M2_ENC_B_PIN 39   // VN/SENSOR_VN, input-only, needs external 10k pull-up

// ---- Motor 3 ----
#define M3_IN1_PIN   23
#define M3_IN2_PIN   25
#define M3_ENC_A_PIN 13
#define M3_ENC_B_PIN 14

// ---- Motor 4 ----
#define M4_IN1_PIN   32
#define M4_IN2_PIN   33
#define M4_ENC_A_PIN 4
#define M4_ENC_B_PIN 5    // strapping pin (must float/HIGH at boot); OK as encoder input w/ pull-up

// ---- HuskyLens (UART2) ----
// Kept on the ESP32's default UART2 pins; do not reuse for encoders/GPIO.
#define HUSKYLENS_RX2_PIN 16
#define HUSKYLENS_TX2_PIN 17

// ---- Shared I2C bus: BNO055 IMU + proximity (ToF) sensor array ----
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// ---- Proximity sensor shutdown (XSHUT) pins ----
// All sensors share the I2C bus above and boot with the same default address.
// Each XSHUT line is held LOW at startup so sensors can be brought up one at
// a time and reassigned a unique I2C address.
#define PROX_SENSOR_COUNT 4
#define PROX1_XSHUT_PIN 2   // strapping pin, must be LOW at boot; drive only after setup()
#define PROX2_XSHUT_PIN 12  // strapping pin, must be LOW at boot (flash voltage select)
#define PROX3_XSHUT_PIN 15  // strapping pin, must be LOW at boot (boot log enable)
#define PROX4_XSHUT_PIN 0   // BOOT button pin; avoid pressing BOOT while running

static const uint8_t PROX_XSHUT_PINS[PROX_SENSOR_COUNT] = {
    PROX1_XSHUT_PIN, PROX2_XSHUT_PIN, PROX3_XSHUT_PIN, PROX4_XSHUT_PIN
};
