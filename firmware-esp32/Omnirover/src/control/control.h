#pragma once
#include <Arduino.h>

// Rover run/stop state + selected trajectory, shared between the web server
// task and main.cpp's loop(). See README.md.

enum RoverTrajectory {
    TRAJ_SQUARE   = 0, // kept in firmware, hidden from the dashboard during PID validation
    TRAJ_CIRCLE   = 1, // idem
    TRAJ_FORWARD  = 2,
    TRAJ_BACKWARD = 3,
    TRAJ_RIGHT    = 4, // lateral slide
    TRAJ_LEFT     = 5,
};

void control_init();

void control_start();
void control_stop();
bool control_isRunning();

void control_setTrajectory(RoverTrajectory t);
RoverTrajectory control_getTrajectory();
