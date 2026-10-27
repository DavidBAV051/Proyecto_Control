#pragma once
#include <Arduino.h>
#include "pins.h"

// See README.md for pin wiring, encoder decoding, and duty-cap rationale.

#define NUM_MOTORS 4

// Free-run RPM at 100% duty cycle; duty is scaled per motor to cap at FLEET_RPM_CAP.
static const uint16_t MOTOR_RATED_RPM[NUM_MOTORS] = {150, 140, 150, 150};
static const uint16_t FLEET_RPM_CAP = 140;

// Measured quadrature pulse counts per output-shaft revolution, per motor.
static const float ENCODER_COUNTS_PER_REV[NUM_MOTORS] = {1047.0f, 2046.0f, 1028.0f, 1000.0f};

// Normalizes raw quadrature sign so +RPM always means "the way moveForward() drives it".
static const int8_t ENCODER_DIR[NUM_MOTORS] = {-1, 1, -1, 1};

class MotorController {
public:
    void init();
    void setMotorSpeed(uint8_t motorIndex, int16_t speed);
    void stopMotor(uint8_t motorIndex);
    void stopAll();

    // Ramps toward targetSpeed instead of jumping instantly; call update() every loop().
    void setMotorSpeedRamped(uint8_t motorIndex, int16_t targetSpeed);
    void update();

    // Reads encoder counts, pushes RPM/position to telemetry, and updates odometry.
    void updateEncoderTelemetry();

private:
    static const uint32_t PWM_FREQ = 20000; // 20 kHz ultrasonic frequency
    static const uint8_t  PWM_RES  = 8;     // 8-bit resolution (0 - 255)
    static const uint32_t RAMP_TIME_MS = 300; // time to ramp 0 -> full duty

    int16_t  currentSpeed_[NUM_MOTORS] = {0, 0, 0, 0};
    int16_t  targetSpeed_[NUM_MOTORS]  = {0, 0, 0, 0};
    uint32_t lastRampMs_ = 0;
};

extern MotorController motors;

