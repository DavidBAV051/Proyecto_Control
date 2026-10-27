#pragma once
#include <Arduino.h>
#include "imu.h"

// Latest HuskyLens detection; decoupled from HUSKYLENSResult so telemetry.h
// doesn't need the HuskyLens library. See README.md.
struct CamData {
    bool    valid  = false;
    int16_t id     = 0;
    int16_t x      = 0;
    int16_t y      = 0;
    int16_t width  = 0;
    int16_t height = 0;
};

struct EncoderData {
    long  position    = 0; // pulse count
    float velocityRpm = 0;
};

// Dead-reckoned world-frame position in meters; see src/odometry/README.md.
struct PoseData {
    float x = 0;
    float y = 0;
};

#define TELEMETRY_NUM_MOTORS 4

struct RoverTelemetry {
    uint32_t     uptimeMs                          = 0;
    IMU_Data     imu                                = {};
    CamData      cam                                = {};
    EncoderData  encoders[TELEMETRY_NUM_MOTORS]     = {};
    PoseData     pose                               = {};
};

void telemetry_init();

// Thread-safe setters; see README.md for who calls each one.
void telemetry_setIMU(const IMU_Data &data);
void telemetry_setCam(const CamData &data);
void telemetry_setEncoder(uint8_t motorIndex, const EncoderData &data);
void telemetry_setPose(float x, float y);

// Thread-safe snapshot of the current telemetry state.
RoverTelemetry telemetry_get();

// Placeholder cam detection until src/cam/ is wired in; see README.md.
void telemetry_simulateCam();

// Serializes the current snapshot to a compact JSON string for the dashboard.
String telemetry_toJson();
