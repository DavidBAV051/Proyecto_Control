#pragma once
#include <Arduino.h>
#include "control.h"

enum EspNowControlCommand : uint8_t {
    ESPNOW_CONTROL_MODE = 1,
    ESPNOW_CONTROL_START,
    ESPNOW_CONTROL_IDLE,
};

void espNow_init();
void espNow_sendControl(EspNowControlCommand command,
                        RoverOperationMode mode = OPERATION_MODE_CAMERA,
                        RoverLandmark landmark = LANDMARK_NONE);
