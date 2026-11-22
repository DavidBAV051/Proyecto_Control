#pragma once
#include <Arduino.h>
#include "control.h"

struct PiLinkStatus {
    bool ready = false;
    bool started = false;
    bool telReceived = false;
    bool fresh = false;
    bool wheelsAccepted = false;
    uint32_t telAgeMs = 0;
    float wheelsRadS[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    char state[12] = "NONE";
    char issue[24] = "WAIT_MODE";
};

void cam_init();
void cam_poll();

void cam_requestMode(RoverOperationMode mode, RoverLandmark landmark);
bool cam_hasReady(RoverOperationMode mode, RoverLandmark landmark);
bool cam_sendStart();
void cam_sendIdle();
void cam_sendPose(float x, float y, float theta);

bool cam_started();
bool cam_wheelsFresh();
void cam_copyWheelSpeeds(float wheelRadS[4]);
PiLinkStatus cam_getLinkStatus();