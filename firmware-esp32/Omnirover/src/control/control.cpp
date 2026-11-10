#include "control.h"

static SemaphoreHandle_t g_mutex = nullptr;
static bool g_running = false;
static RoverTrajectory g_trajectory = TRAJ_SQUARE;
static ManualCmd g_manual;

void control_init() {
    if (!g_mutex) {
        g_mutex = xSemaphoreCreateMutex();
    }
}

void control_start() {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_running = true;
    xSemaphoreGive(g_mutex);
}

void control_stop() {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_running = false;
    xSemaphoreGive(g_mutex);
}

bool control_isRunning() {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    bool running = g_running;
    xSemaphoreGive(g_mutex);
    return running;
}

void control_setTrajectory(RoverTrajectory t) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_trajectory = t;
    xSemaphoreGive(g_mutex);
}

RoverTrajectory control_getTrajectory() {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    RoverTrajectory t = g_trajectory;
    xSemaphoreGive(g_mutex);
    return t;
}

void control_setManual(float vx, float vy, float wz) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_manual.vx = constrain(vx, -1.0f, 1.0f);
    g_manual.vy = constrain(vy, -1.0f, 1.0f);
    g_manual.wz = constrain(wz, -1.0f, 1.0f);
    g_manual.timestampMs = millis();
    xSemaphoreGive(g_mutex);
}

ManualCmd control_getManual() {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    ManualCmd c = g_manual;
    xSemaphoreGive(g_mutex);
    return c;
}
