#include "telemetry.h"
#include <ArduinoJson.h>

static RoverTelemetry g_telemetry;
static SemaphoreHandle_t g_mutex = nullptr;

void telemetry_init() {
    if (!g_mutex) {
        g_mutex = xSemaphoreCreateMutex();
    }
}

void telemetry_setIMU(const IMU_Data &data) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_telemetry.imu = data;
    xSemaphoreGive(g_mutex);
}

void telemetry_setCam(const CamData &data) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_telemetry.cam = data;
    xSemaphoreGive(g_mutex);
}

void telemetry_setEncoder(uint8_t motorIndex, const EncoderData &data) {
    if (motorIndex >= TELEMETRY_NUM_MOTORS) return;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_telemetry.encoders[motorIndex] = data;
    xSemaphoreGive(g_mutex);
}

void telemetry_setPose(float x, float y) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_telemetry.pose.x = x;
    g_telemetry.pose.y = y;
    xSemaphoreGive(g_mutex);
}

RoverTelemetry telemetry_get() {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    RoverTelemetry snapshot = g_telemetry;
    xSemaphoreGive(g_mutex);
    return snapshot;
}

void telemetry_simulateCam() {
    uint32_t t = millis();
    float phase = t / 1000.0f;

    CamData cam;
    cam.valid  = true;
    cam.id     = 1;
    cam.x      = 160 + (int16_t)(100 * sinf(phase * 0.8f));
    cam.y      = 120 + (int16_t)(60 * cosf(phase * 0.8f));
    cam.width  = 40;
    cam.height = 40;
    telemetry_setCam(cam);

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_telemetry.uptimeMs = t;
    xSemaphoreGive(g_mutex);
}

String telemetry_toJson() {
    RoverTelemetry snap = telemetry_get();

    JsonDocument doc;
    doc["t"] = snap.uptimeMs;

    JsonObject imu = doc["imu"].to<JsonObject>();
    imu["ax"]   = snap.imu.accelX;
    imu["ay"]   = snap.imu.accelY;
    imu["az"]   = snap.imu.accelZ;
    imu["gx"]   = snap.imu.gyroX;
    imu["gy"]   = snap.imu.gyroY;
    imu["gz"]   = snap.imu.gyroZ;
    imu["temp"] = snap.imu.temp;
    imu["yaw"]  = snap.imu.yaw;

    JsonObject cam = doc["cam"].to<JsonObject>();
    cam["valid"] = snap.cam.valid;
    cam["id"]    = snap.cam.id;
    cam["x"]     = snap.cam.x;
    cam["y"]     = snap.cam.y;
    cam["w"]     = snap.cam.width;
    cam["h"]     = snap.cam.height;

    JsonArray enc = doc["enc"].to<JsonArray>();
    for (uint8_t m = 0; m < TELEMETRY_NUM_MOTORS; m++) {
        JsonObject e = enc.add<JsonObject>();
        e["pos"] = snap.encoders[m].position;
        e["vel"] = snap.encoders[m].velocityRpm;
    }

    JsonObject pose = doc["pose"].to<JsonObject>();
    pose["x"] = snap.pose.x;
    pose["y"] = snap.pose.y;

    String out;
    serializeJson(doc, out);
    return out;
}
