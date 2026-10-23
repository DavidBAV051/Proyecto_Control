#pragma once
#include <Arduino.h>
#include "imu.h"

// Latest HuskyLens detection (mirrors HUSKYLENSResult, decoupled so telemetry.h
// doesn't need to pull in the HuskyLens library).
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

#define TELEMETRY_NUM_MOTORS 4

struct RoverTelemetry {
    uint32_t     uptimeMs                          = 0;
    IMU_Data     imu                                = {};
    CamData      cam                                = {};
    EncoderData  encoders[TELEMETRY_NUM_MOTORS]     = {};
};

void telemetry_init();

// Thread-safe setters: call these from imu.cpp/cam.cpp/motors.cpp once the
// real BNO/HuskyLens reads are wired in (encoders are already live, fed by
// MotorController::updateEncoderTelemetry()), to replace the placeholders.
void telemetry_setIMU(const IMU_Data &data);
void telemetry_setCam(const CamData &data);
void telemetry_setEncoder(uint8_t motorIndex, const EncoderData &data);

// Thread-safe snapshot of the current telemetry state.
RoverTelemetry telemetry_get();

// No IMU/cam sensors are wired in yet: fills telemetry with smoothly moving
// placeholder values so the dashboard has something real-time to show.
// Encoders are excluded here since they're now fed live, see telemetry_setEncoder().
// TODO: remove once init_IMU()/updateHuskyLens() feed telemetry_set*().
void telemetry_simulate();

// Serializes the current snapshot to a compact JSON string for the dashboard.
String telemetry_toJson();
