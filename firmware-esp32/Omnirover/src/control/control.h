#pragma once
#include <Arduino.h>

// Rover run/stop state + selected trajectory, shared between the web server
// task and main.cpp's loop(). See README.md.

enum RoverTrajectory {
    TRAJ_SQUARE = 0,
    TRAJ_CIRCLE = 1,
    TRAJ_FORWARD = 2,
};

void control_init();

void control_start();
void control_stop();
bool control_isRunning();

void control_setTrajectory(RoverTrajectory t);
RoverTrajectory control_getTrajectory();
