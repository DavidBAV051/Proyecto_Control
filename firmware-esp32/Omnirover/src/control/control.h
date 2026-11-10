#pragma once
#include <Arduino.h>

// Rover run/stop state + selected trajectory, shared between the web server
// task and main.cpp's loop(). See README.md.

enum RoverTrajectory {
    TRAJ_SQUARE = 0,
    TRAJ_CIRCLE = 1,
    TRAJ_FORWARD = 2,
    TRAJ_MANUAL = 3, // dashboard joystick
};

// Latest joystick command, body frame, each axis normalized to [-1, 1].
// vx + = forward, vy + = right, wz + = clockwise. timestampMs = millis() when received.
struct ManualCmd {
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    uint32_t timestampMs = 0;
};

void control_init();

void control_start();
void control_stop();
bool control_isRunning();

void control_setTrajectory(RoverTrajectory t);
RoverTrajectory control_getTrajectory();

void control_setManual(float vx, float vy, float wz);
ManualCmd control_getManual();
