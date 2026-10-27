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

// Measured quadrature pulse counts for one full output-shaft revolution,
// per motor (M1 is the swapped 140 RPM unit, hence the different count).
static const float ENCODER_COUNTS_PER_REV[NUM_MOTORS] = {1047.0f, 2046.0f, 1028.0f, 1000.0f};

// Motors 0 & 2 have their encoder A/B leads wired with the opposite phase
// relationship vs motors 1 & 3, so raw quadrature counts come out negated
// relative to commanded forward direction. Flips sign so +RPM always means
// "spinning the way moveForward() drives it", matching motors 1/3.
static const int8_t ENCODER_DIR[NUM_MOTORS] = {-1, 1, -1, 1};

class MotorController {
public:
    void init();
    void setMotorSpeed(uint8_t motorIndex, int16_t speed);
    void stopMotor(uint8_t motorIndex);
    void stopAll();

    // Sets a target speed that update() ramps toward gradually (instead of an instant
    // jump), so 4 motors starting together don't all spike current at once and brown
    // out / trip the shared battery. Call update() every loop() iteration.
    void setMotorSpeedRamped(uint8_t motorIndex, int16_t targetSpeed);
    void update();

    // Reads the latest quadrature pulse counts and pushes position/RPM to
    // telemetry for all motors. Call periodically (e.g. from the telemetry task).
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
