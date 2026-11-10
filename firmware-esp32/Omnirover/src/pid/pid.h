#pragma once
#include <Arduino.h>

// Discrete PID with derivative-on-measurement and clamped integrator. See README.md.
class PID {
public:
    PID(float kp, float ki, float kd, float outMin, float outMax);

    // dt in seconds. Returns the saturated control output.
    float update(float setpoint, float measurement, float dt);
    void reset();

private:
    float kp_, ki_, kd_;
    float outMin_, outMax_;
    float integral_ = 0.0f;
    float prevMeasurement_ = 0.0f;
    bool  firstRun_ = true;
};
