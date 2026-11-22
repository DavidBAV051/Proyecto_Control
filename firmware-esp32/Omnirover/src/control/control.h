#pragma once
#include <Arduino.h>

enum RoverOperationMode : uint8_t {
    OPERATION_MODE_CAMERA = 0,
    OPERATION_MODE_LANDMARK = 1,
};

enum RoverLandmark : uint8_t {
    LANDMARK_NONE = 0,
    LANDMARK_TOP_RIGHT,
    LANDMARK_TOP_LEFT,
    LANDMARK_DOWN_RIGHT,
    LANDMARK_DOWN_LEFT,
    LANDMARK_CENTER,
};

struct RoverControlState {
    RoverOperationMode mode;
    RoverLandmark landmark;
    uint32_t modeRevision;
    uint32_t idleRevision;
    bool running;
};

void control_init();

void control_selectMode(RoverOperationMode mode, RoverLandmark landmark = LANDMARK_NONE);
void control_start();
void control_idle();
RoverControlState control_getState();
