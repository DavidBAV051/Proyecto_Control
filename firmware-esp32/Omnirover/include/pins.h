#pragma once
#include <Arduino.h>

// =====================================================================
// Centralized GPIO pin map (ESP32, mhetesp32devkit ~30 usable GPIOs)
//
// Proximity (ToF) sensors were dropped from the design. Their former XSHUT
// pins (2, 12, 15, 0) are free/unused for now; encoders stayed on the
// original input-only pins (34/35/36/39), which need external 10k
// pull-ups to 3.3V since those pins have no internal pull resistors.
//
// Budget:
//   Motors (4x IN1/IN2 + encoder A/B)........ 16 pins
//   HuskyLens UART2 (RX2/TX2)................. 2 pins
//   Shared I2C bus (BNO08x IMU)................ 2 pins
//   ------------------------------------------------
//   Total...................................... 20 pins
//   Free (unused)............................. 2, 12, 15, 0
// =====================================================================

// ---- Motor 1 ----
#define M1_IN1_PIN   26
#define M1_IN2_PIN   27
#define M1_ENC_A_PIN 13
#define M1_ENC_B_PIN 14   // strapping pin (must float/HIGH at boot); OK as encoder input w/ pull-up

// ---- Motor 2 ----
#define M2_IN1_PIN   18
#define M2_IN2_PIN   19
#define M2_ENC_A_PIN 36   // VP/SENSOR_VP, input-only, needs external 10k pull-up
#define M2_ENC_B_PIN 39   // VN/SENSOR_VN, input-only, needs external 10k pull-up

// ---- Motor 3 ----
#define M3_IN1_PIN   23
#define M3_IN2_PIN   25
#define M3_ENC_A_PIN 34 // input-only, needs external 10k pull-up to 3.3V
#define M3_ENC_B_PIN 35 // input-only, needs external 10k pull-up to 3.3V

// ---- Motor 4 ----
#define M4_IN1_PIN   32
#define M4_IN2_PIN   33
#define M4_ENC_A_PIN 4
#define M4_ENC_B_PIN 5

// ---- HuskyLens (UART2) ----
// Kept on the ESP32's default UART2 pins; do not reuse for encoders/GPIO.
#define HUSKYLENS_RX2_PIN 16
#define HUSKYLENS_TX2_PIN 17

// ---- Shared I2C bus: GY-BNO08X IMU ----
// GY-BNO08X wiring for I2C mode (no extra GPIOs needed):
//   PS1, PS0 -> GND (selects I2C protocol; module default)
//   CS, RST  -> regulated 3V3 rail only, NEVER board 5V (direct logic pins on the die)
//   ADO      -> GND (I2C address 0x4A; drive HIGH instead for 0x4B if it collides)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
