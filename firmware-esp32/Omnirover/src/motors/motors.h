#pragma once
#include <Arduino.h>
#include "pins.h"

// DRV8871 driver + quadrature encoder pins are defined centrally in pins.h.
// Motors 2 & 4 encoders use input-only GPIOs (34/35/36/39): require 10 kOhm
// pull-ups to 3.3 V on each line (no internal pull-up available).
// Motors 1 & 3 encoders use regular GPIOs: configure with INPUT_PULLUP in firmware.

#define NUM_MOTORS 4

// Free-run RPM at 100% duty cycle; M2 was swapped to a 140 RPM motor to fix
// its encoder issue, so every motor's duty is scaled to cap at FLEET_RPM_CAP.
static const uint16_t MOTOR_RATED_RPM[NUM_MOTORS] = {150, 140, 150, 150};
static const uint16_t FLEET_RPM_CAP = 140;

// Encoder pulse counting: PLACEHOLDER base PPR, tune once the true output-shaft
// counts-per-revolution is measured per motor. Quadrature decoding on both
// channels (A+B, every edge) multiplies this base PPR by ENCODER_QUAD_MULT.
// Motor index 1 is the swapped 140 RPM unit with a different encoder: at the
// same commanded duty its raw pulse rate reads ~1.9x the other three, so it
// gets its own (still placeholder) PPR until measured directly.
static const float   ENCODER_PPR[NUM_MOTORS] = {11.0f, 20.5f, 11.0f, 11.0f};
static const uint8_t ENCODER_QUAD_MULT = 4;

class MotorController {
public:
    void init();
    void setMotorSpeed(uint8_t motorIndex, int16_t speed);
    void stopMotor(uint8_t motorIndex);
    void stopAll();

    // Reads the latest quadrature pulse counts and pushes position/RPM to
    // telemetry for all motors. Call periodically (e.g. from the telemetry task).
    void updateEncoderTelemetry();

private:
    static const uint32_t PWM_FREQ = 20000; // 20 kHz ultrasonic frequency
    static const uint8_t  PWM_RES  = 8;     // 8-bit resolution (0 - 255)
};

extern MotorController motors;
