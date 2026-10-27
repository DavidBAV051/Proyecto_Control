#pragma once
#include <Arduino.h>

// Rover run/stop state + selected trajectory, set from the dashboard's Start/Stop
// buttons and trajectory selector (web_server.cpp's /control endpoint) and read
// from main.cpp's loop(). Thread-safe: the web server and main loop run as
// separate FreeRTOS tasks.

enum RoverTrajectory {
    TRAJ_SQUARE = 0,
    TRAJ_CIRCLE = 1,
};

void control_init();

void control_start();
void control_stop();
bool control_isRunning();

void control_setTrajectory(RoverTrajectory t);
RoverTrajectory control_getTrajectory();
