#pragma once
#include <Arduino.h>

// Mecanum-wheel dead-reckoning: combines the 4 wheel RPMs (motors.cpp) with the
// IMU yaw (imu.cpp) to track the rover's position in meters, world frame.
// Wheel RPM alone can't give heading (no slip-free way to derive yaw from 4
// independent mecanum wheels reliably), so yaw always comes from the IMU.

struct OdometryPose {
    float x = 0; // meters, world frame (matches dashboard: +x when yaw=0 strafing right)
    float y = 0; // meters, world frame (matches dashboard: +y when yaw=0 driving forward)
};

void odometry_init();
void odometry_reset();

// wheelRpm order matches motor index: 0 = front left, 1 = front right,
// 2 = back left, 3 = back right (see main.cpp). yawDeg: 0 = forward,
// positive = clockwise (same convention as IMU_Data.yaw). elapsedMs: time
// since the previous call (e.g. MotorController's encoder sample interval).
void odometry_update(const float wheelRpm[4], float yawDeg, uint32_t elapsedMs);

OdometryPose odometry_getPose();
