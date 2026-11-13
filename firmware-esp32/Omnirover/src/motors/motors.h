#pragma once
#include <Arduino.h>
#include "pins.h"
#include "pid.h"

// See README.md for pin wiring, encoder decoding, duty-cap and PID rationale.

#define NUM_MOTORS 4

static constexpr float RAD_S_TO_RPM = 60.0f / (2.0f * (float)PI);

// Wheel setpoint cap. Measured 2026-11-12 on the floor: at 68.75 rpm (7.2 rad/s) the
// slowest wheel (M1, 140 rpm nominal) needed 77% duty, the rest ~72%. 80 rpm puts M1 at
// ~90% / others ~84%, leaving ~10% authority for the loop plus battery sag. Do not raise
// without re-measuring duty; 85 rpm already pushes M1 to ~95% and it saturates as the
// LiPo discharges.
static constexpr float MAX_WHEEL_RPM   = 75.0f;
static constexpr float MAX_WHEEL_RAD_S = MAX_WHEEL_RPM / RAD_S_TO_RPM; // ~8.38 rad/s
// Controller output scale: this many "rad/s-equivalent" = 100% duty. Kept at the
// MATLAB value so PID_KP/KI/KD stay valid; raising the setpoint cap must not retune the loop.
static constexpr float PID_OUTPUT_FULL_RAD_S = 7.2f;

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
static const float PID_KP[NUM_MOTORS] = {0.22f,   0.18f,   0.25f,   0.25f};
static const float PID_KI[NUM_MOTORS] = {3.0f,   3.0f,   3.0f,   3.0f};
static const float PID_KD[NUM_MOTORS] = {0.005f, 0.005f, 0.005f, 0.005f};
// First-order low-pass on measured wheel speed (PID measurement + telemetry). Tames
// encoder count jitter (worst on M3: lowest CPR, so each spurious count is more rad/s)
// before the D term amplifies it. Lag sits inside the loop: 0.10 s made the integrator
// wind past the setpoint (visible overshoot), so keep it to a couple of 20 ms ticks.
static constexpr float SPEED_FILTER_TAU_S = 0.035f;

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
    float    filteredRadS_[NUM_MOTORS] = {0, 0, 0, 0}; // low-passed measurement, see SPEED_FILTER_TAU_S
    bool     pidActive_[NUM_MOTORS]  = {false, false, false, false};
    long     lastPidCount_[NUM_MOTORS] = {0, 0, 0, 0};
    uint32_t lastPidMs_ = 0;
};

extern MotorController motors;

