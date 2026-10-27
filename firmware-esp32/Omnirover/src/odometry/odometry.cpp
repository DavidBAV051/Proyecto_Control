#include "odometry.h"

// Chassis geometry, measured 2026-10-27 (MATLAB control model):
static const float WHEEL_RADIUS_M    = 0.04f;  // Rw: wheel radius (80 mm diameter)
static const float HALF_WHEELBASE_M  = 0.08f;  // L: center-of-mass to front/back axle
static const float HALF_TRACK_M      = 0.105f; // l: center-of-mass to left/right axle (incl. couplers)
// HALF_TRACK_M + HALF_WHEELBASE_M would feed the wheels->yaw-rate term of the mecanum
// Jacobian; unused here since yaw comes from the IMU instead (see odometry.h).

static OdometryPose g_pose;

void odometry_init() { g_pose = OdometryPose(); }
void odometry_reset() { g_pose = OdometryPose(); }

void odometry_update(const float wheelRpm[4], float yawDeg, uint32_t elapsedMs) {
    if (elapsedMs == 0) return;
    float dt = elapsedMs / 1000.0f;

    // Wheel surface speed, m/s (already sign-normalized by ENCODER_DIR in motors.cpp
    // so all 4 are directly comparable: + means "the way moveForward() drives it").
    float v[4];
    for (uint8_t i = 0; i < 4; i++) {
        v[i] = (wheelRpm[i] / 60.0f) * TWO_PI * WHEEL_RADIUS_M;
    }

    // Standard mecanum forward kinematics (body frame), derived from this rover's
    // motor index order (0=back right, 1=back left, 2=front right, 3=front left)
    // and verified against moveForward()/slideRight()'s commanded sign patterns.
    float vxBody = (v[0] + v[1] + v[2] + v[3]) / 4.0f;        // forward
    float vyBody = (v[0] - v[1] - v[2] + v[3]) / 4.0f;        // lateral, + = right

    // Rotate body-frame velocity into world frame using IMU yaw (0 deg = forward,
    // + = clockwise), matching the dashboard's x=sin(yaw)/y=cos(yaw) convention.
    float yawRad = yawDeg * DEG_TO_RAD;
    float sinY = sinf(yawRad), cosY = cosf(yawRad);
    g_pose.x += (vxBody * sinY + vyBody * cosY) * dt;
    g_pose.y += (vxBody * cosY - vyBody * sinY) * dt;
}

OdometryPose odometry_getPose() { return g_pose; }
