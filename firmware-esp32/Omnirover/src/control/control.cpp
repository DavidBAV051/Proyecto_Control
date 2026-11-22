#include "control.h"

static SemaphoreHandle_t g_mutex = nullptr;
static RoverControlState g_state = {
    OPERATION_MODE_CAMERA,
    LANDMARK_NONE,
    0,
    0,
    false,
};

void control_init() {
    if (!g_mutex) {
        g_mutex = xSemaphoreCreateMutex();
    }
}

void control_selectMode(RoverOperationMode mode, RoverLandmark landmark) {
    if (mode == OPERATION_MODE_CAMERA) {
        landmark = LANDMARK_NONE;
    } else if (landmark == LANDMARK_NONE) {
        landmark = LANDMARK_CENTER;
    }

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_state.mode = mode;
    g_state.landmark = landmark;
    g_state.modeRevision++;
    g_state.running = false;
    xSemaphoreGive(g_mutex);
}

void control_start() {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_state.running = true;
    xSemaphoreGive(g_mutex);
}

void control_idle() {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_state.running = false;
    g_state.idleRevision++;
    xSemaphoreGive(g_mutex);
}

RoverControlState control_getState() {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    RoverControlState state = g_state;
    xSemaphoreGive(g_mutex);
    return state;
}
