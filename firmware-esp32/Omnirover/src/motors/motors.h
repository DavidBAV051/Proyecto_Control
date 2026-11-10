#pragma once
#include <Arduino.h>
#include "pins.h"
#include "pid.h"

// See README.md for pin wiring, encoder decoding, duty-cap and PID rationale.

#define NUM_MOTORS 4

// Max wheel speed without saturating the actuators (MATLAB); maps to 100% PWM.
static constexpr float MAX_WHEEL_RAD_S = 7.2f;
static constexpr float RAD_S_TO_RPM = 60.0f / (2.0f * (float)PI);

// Datasheet free-run RPM at rated voltage (reference only; real bus voltage is lower).
static const uint16_t MOTOR_RATED_RPM[NUM_MOTORS] = {150, 140, 150, 150};

// Measured quadrature pulse counts per output-shaft revolution, per motor.
static const float ENCODER_COUNTS_PER_REV[NUM_MOTORS] = {1047.0f, 2046.0f, 1028.0f, 1000.0f};

// Normalizes raw quadrature sign so +RPM always means "the way moveForward() drives it".
static const int8_t ENCODER_DIR[NUM_MOTORS] = {-1, 1, -1, 1};

// Duty sign that drives each wheel forward (motor 3's driver is wired inverted).
static const int8_t MOTOR_CMD_DIR[NUM_MOTORS] = {1, 1, 1, -1};

// Per-wheel velocity PID gains (MATLAB design). M1's Ki lowered from 4.35 to match
// the others: with Kp=0 the integrator ramp rate IS the start-up delay, so a higher
// Ki made M1 break away well before the rest.
static const float PID_KP[NUM_MOTORS] = {0.0f,   0.0f,   0.0f,   0.0f};
static const float PID_KI[NUM_MOTORS] = {3.0f,   3.0f,   3.0f,   3.0f};
static const float PID_KD[NUM_MOTORS] = {0.005f, 0.005f, 0.005f, 0.005f};

class MotorController {
public:
    MotorController();

    void init();
    void setMotorSpeed(uint8_t motorIndex, int16_t speed);
    void stopMotor(uint8_t motorIndex);
    void stopAll();

    // Closed-loop wheel speed in rad/s (+ = forward for every wheel), clamped to
    // +-MAX_WHEEL_RAD_S. Call updateSpeedControl() every loop() to run the PIDs.
    void setWheelSpeed(uint8_t motorIndex, float radPerSec);
    void updateSpeedControl();

    // Ramps toward targetSpeed instead of jumping instantly; call update() every loop().
    void setMotorSpeedRamped(uint8_t motorIndex, int16_t targetSpeed);
    void update();

    // Reads encoder counts, pushes RPM/position to telemetry, and updates odometry.
    void updateEncoderTelemetry();

private:
    static const uint32_t PWM_FREQ = 20000; // 20 kHz ultrasonic frequency
    static const uint8_t  PWM_RES  = 8;     // 8-bit resolution (0 - 255)
    static const uint32_t RAMP_TIME_MS = 300; // time to ramp 0 -> full duty
    static const uint32_t PID_PERIOD_MS = 20; // velocity loop sample time

    int16_t  currentSpeed_[NUM_MOTORS] = {0, 0, 0, 0};
    int16_t  targetSpeed_[NUM_MOTORS]  = {0, 0, 0, 0};
    uint32_t lastRampMs_ = 0;

    PID      speedPid_[NUM_MOTORS];
    float    targetRadS_[NUM_MOTORS] = {0, 0, 0, 0};
    bool     pidActive_[NUM_MOTORS]  = {false, false, false, false};
    long     lastPidCount_[NUM_MOTORS] = {0, 0, 0, 0};
    uint32_t lastPidMs_ = 0;
};

extern MotorController motors;

