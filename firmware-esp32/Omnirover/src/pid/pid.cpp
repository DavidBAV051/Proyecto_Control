#include "pid.h"

PID::PID(float kp, float ki, float kd, float outMin, float outMax)
    : kp_(kp), ki_(ki), kd_(kd), outMin_(outMin), outMax_(outMax) {}

float PID::update(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return constrain(integral_, outMin_, outMax_);

    float error = setpoint - measurement;

    // Clamp the integrator to the output range so it can't wind up past saturation.
    integral_ += ki_ * error * dt;
    integral_ = constrain(integral_, outMin_, outMax_);

    // Derivative on measurement avoids a kick when the setpoint steps.
    float derivative = firstRun_ ? 0.0f : (measurement - prevMeasurement_) / dt;
    prevMeasurement_ = measurement;
    firstRun_ = false;

    float output = kp_ * error + integral_ - kd_ * derivative;
    return constrain(output, outMin_, outMax_);
}

void PID::reset() {
    integral_ = 0.0f;
    prevMeasurement_ = 0.0f;
    firstRun_ = true;
}
