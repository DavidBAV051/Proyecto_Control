#include <Arduino.h>
#include <esp_system.h>
#include "cam.h"
#include "imu.h"
#include "motors.h"
#include "web_server.h"
#include "espnow_link.h"
#include "telemetry.h"
#include "control.h"

// Wheel speeds are closed-loop setpoints in rad/s; + = forward for every wheel
// (MotorController applies the per-driver sign). PIDs run in motors.updateSpeedControl().

// Motor index reference (physical, verified on the rover 2026-11-02):
//   0 = front left, 1 = front right, 2 = back left, 3 = back right
// Left side = {0, 2}, right side = {1, 3}.

static const float DEFAULT_DRIVE_RAD_S  = MAX_WHEEL_RAD_S; // 7.2 rad/s (~69 rpm)
static const float DEFAULT_SLIDE_RAD_S  = MAX_WHEEL_RAD_S;
static const float DEFAULT_CIRCLE_RAD_S = MAX_WHEEL_RAD_S;
static const float CIRCLE_INNER_RATIO   = 0.54f; // left/inner wheels {0,2} vs. right/outer -> arcs left
static const uint32_t SAFE_START_DELAY_MS = 2000; // wait before first motor command
static const uint32_t MOVE_DURATION_MS = 1000;
static const uint32_t STOP_PAUSE_MS = 500;
static const uint32_t MANUAL_CMD_TIMEOUT_MS = 500; // joystick deadman: stop if no fresh command

// Heading hold: PI controller on IMU yaw that trims right/left wheel setpoints so
// straight segments stay straight despite per-wheel encoder/tire mismatch. The I term
// removes the steady-state offset a P-only loop leaves against a constant asymmetry.
static const float HEADING_KP = 0.08f;  // rad/s of wheel differential per degree of yaw error
static const float HEADING_KI = 0.04f;  // rad/s per degree-second
static const float HEADING_MAX_CORR_RAD_S = 1.5f;
static PID   headingPid(HEADING_KP, HEADING_KI, 0.0f, -HEADING_MAX_CORR_RAD_S, HEADING_MAX_CORR_RAD_S);
static float headingTargetDeg = 0.0f;
static bool  headingLocked = false;
static uint32_t headingLastMs = 0;
static float headingLastCorr = 0.0f;

// Call whenever a new motion segment starts so the next straight move re-captures its heading.
void releaseHeadingHold()
{
    headingLocked = false;
    headingPid.reset();
    headingLastCorr = 0.0f;
}

// Yaw is compass-style (0 = forward, + = clockwise). Returns the trim in rad/s.
// Drifting clockwise (+err) -> slow the left side {0,2}, speed up the right {1,3}.
static float headingHoldCorrection()
{
    float yaw = get_IMU_Data().yaw;
    uint32_t now = millis();
    if (!headingLocked) {
        headingTargetDeg = yaw;
        headingLocked = true;
        headingLastMs = now;
    }
    uint32_t elapsedMs = now - headingLastMs;
    if (elapsedMs == 0) return headingLastCorr;
    headingLastMs = now;

    float err = yaw - headingTargetDeg;
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    // PID computes setpoint - measurement, so feed the wrapped error as "setpoint" with 0 measurement.
    headingLastCorr = headingPid.update(err, 0.0f, elapsedMs / 1000.0f);
    return headingLastCorr;
}

// Straight-line setpoints (FL, FR, BL, BR) with heading-hold trim applied. Target heading
// is whatever yaw the rover had when the segment began, so any start angle works.
static void driveStraight(float fl, float fr, float bl, float br)
{
    float c = headingHoldCorrection();
    motors.setWheelSpeed(0, fl - c);
    motors.setWheelSpeed(1, fr + c);
    motors.setWheelSpeed(2, bl - c);
    motors.setWheelSpeed(3, br + c);
}

// Inverse mecanum kinematics for joystick input (body frame, all normalized -1..1):
// vx + = forward, vy + = right, wz + = clockwise. Scaled so the fastest wheel hits
// MAX_WHEEL_RAD_S without distorting the direction.
static void driveManual()
{
    ManualCmd cmd = control_getManual();
    if (millis() - cmd.timestampMs > MANUAL_CMD_TIMEOUT_MS) {
        motors.stopAll();
        return;
    }
    float w[NUM_MOTORS] = {
        cmd.vx + cmd.vy + cmd.wz, // FL
        cmd.vx - cmd.vy - cmd.wz, // FR
        cmd.vx - cmd.vy + cmd.wz, // BL
        cmd.vx + cmd.vy - cmd.wz, // BR
    };
    float peak = 0.0f;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) peak = max(peak, fabsf(w[i]));
    if (peak < 0.02f) {
        motors.stopAll(); // centered stick: hard stop instead of letting the integrators unwind
        return;
    }
    float scale = (peak > 1.0f) ? (MAX_WHEEL_RAD_S / peak) : MAX_WHEEL_RAD_S;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors.setWheelSpeed(i, w[i] * scale);
    }
}

void moveCircle(float speed = DEFAULT_CIRCLE_RAD_S)
{
    motors.setWheelSpeed(0, speed * CIRCLE_INNER_RATIO);
    motors.setWheelSpeed(1, speed);
    motors.setWheelSpeed(2, speed * CIRCLE_INNER_RATIO);
    motors.setWheelSpeed(3, speed);
}

void moveForward(float speed = DEFAULT_DRIVE_RAD_S)
{
    driveStraight(speed, speed, speed, speed);
}

void moveBackward(float speed = DEFAULT_DRIVE_RAD_S)
{
    driveStraight(-speed, -speed, -speed, -speed);
}

void slideRight(float speed = DEFAULT_SLIDE_RAD_S)
{
    driveStraight(speed, -speed, -speed, speed);
}

void slideLeft(float speed = DEFAULT_SLIDE_RAD_S)
{
    driveStraight(-speed, speed, speed, -speed);
}

// Diagonals: only one wheel pair drives (FL+BR or FR+BL), the other pair idles.
// "Right"/"Left" names the forward diagonal's direction; the backward variants
// reverse the same pair, so diagonalRightBackward physically moves back-left.
void diagonalRightForward(float speed = DEFAULT_DRIVE_RAD_S)
{
    driveStraight(speed, 0.0f, 0.0f, speed);
}

void diagonalLeftForward(float speed = DEFAULT_DRIVE_RAD_S)
{
    driveStraight(0.0f, speed, speed, 0.0f);
}

void diagonalRightBackward(float speed = DEFAULT_DRIVE_RAD_S)
{
    driveStraight(-speed, 0.0f, 0.0f, -speed);
}

void diagonalLeftBackward(float speed = DEFAULT_DRIVE_RAD_S)
{
    driveStraight(0.0f, -speed, -speed, 0.0f);
}

void stopAllMotors()
{
    motors.stopAll();
}

// Non-blocking (no delay()) square trajectory: loops forward->slideRight->
// backward->slideLeft forever while selected and running, so the Stop button
// (control_isRunning()) and dashboard telemetry both stay responsive.
struct TrajectoryStep {
    void (*action)();
    uint32_t durationMs;
};
static const TrajectoryStep SQUARE_TRAJECTORY[] = {
    { []() { moveForward(); },  MOVE_DURATION_MS },
    { stopAllMotors,            STOP_PAUSE_MS },
    { []() { slideRight(); },   MOVE_DURATION_MS },
    { stopAllMotors,            STOP_PAUSE_MS },
    { []() { moveBackward(); }, MOVE_DURATION_MS },
    { stopAllMotors,            STOP_PAUSE_MS },
    { []() { slideLeft(); },    MOVE_DURATION_MS },
    { stopAllMotors,            STOP_PAUSE_MS },
};
static const uint8_t SQUARE_STEP_COUNT = sizeof(SQUARE_TRAJECTORY) / sizeof(SQUARE_TRAJECTORY[0]);
static int8_t squareStepIndex = -1; // -1 = not started yet
static uint32_t squareStepStartMs = 0;

void resetSquareTrajectory()
{
    squareStepIndex = -1;
    releaseHeadingHold();
}

// Call every loop() iteration while running the square trajectory. The active
// step's action is re-applied every iteration (actions are idempotent setpoint
// writes) so the heading-hold trim keeps updating during a segment.
void runSquareTrajectory()
{
    uint32_t now = millis();
    if (squareStepIndex < 0) {
        squareStepIndex = 0;
        squareStepStartMs = now;
    } else if (now - squareStepStartMs >= SQUARE_TRAJECTORY[squareStepIndex].durationMs) {
        squareStepIndex = (squareStepIndex + 1) % SQUARE_STEP_COUNT;
        squareStepStartMs = now;
        releaseHeadingHold();
    }
    SQUARE_TRAJECTORY[squareStepIndex].action();
}

// Reads + broadcasts IMU data and throttles the serial print to ~5Hz; call this
// every loop() iteration so the dashboard/serial output never goes stale.
static uint32_t lastImuPrintMs = 0;
void pollIMU()
{
    update_IMU();
    telemetry_setIMU(get_IMU_Data());
    if (millis() - lastImuPrintMs >= 200) {
        lastImuPrintMs = millis();
        print_IMU_Data();
    }
}

// Blocks until the safe-start window has elapsed since boot.
void waitForSafeStart(uint32_t delayMs = SAFE_START_DELAY_MS)
{
    motors.stopAll();
    Serial.print("Safe start: waiting ");
    Serial.print(delayMs);
    Serial.println(" ms before enabling motors...");
    uint32_t start = millis();
    while (millis() - start < delayMs) {
        delay(50);
    }
    Serial.println("Safe start complete. Motors enabled.");
}

void setup()
{
    Serial.begin(115200);

    // A BROWNOUT here right after a "lost signal" episode means the shared LiPo
    // sagged under motor load and reset the ESP32 (the WiFi AP drops with it).
    esp_reset_reason_t rr = esp_reset_reason();
    const char *rrName =
        rr == ESP_RST_BROWNOUT ? "BROWNOUT" :
        rr == ESP_RST_TASK_WDT ? "TASK_WDT" :
        rr == ESP_RST_PANIC    ? "PANIC" :
        rr == ESP_RST_POWERON  ? "POWERON" :
        rr == ESP_RST_SW       ? "SW" : "OTHER";
    Serial.printf("Reset reason: %s (%d)\n", rrName, (int)rr);

    // Real-time dashboard (WiFi AP + web server, or ESP-NOW bridge depending
    // on ROVER_LINK_MODE in include/net_config.h). Cam telemetry is still
    // placeholder data until HuskyLens is wired in; IMU and encoders are live.
    webServer_init();
    espNow_init();

    // initHuskyLens();
    if (init_IMU()) {
        Serial.println("BNO08x online.");
    } else {
        Serial.println("Error: BNO08x not detected.");
        while (1) { delay(1000); }
    }
    motors.init();
    motors.stopAll();
    Serial.println("DRV8871 Motor Controller Ready.");

    waitForSafeStart();
}

void loop()
{
    pollIMU();

    static bool wasRunning = false;
    static RoverTrajectory lastTrajectory = TRAJ_SQUARE;
    bool running = control_isRunning();
    RoverTrajectory trajectory = control_getTrajectory();

    if (!running) {
        if (wasRunning) {
            stopAllMotors();
            resetSquareTrajectory();
        }
    } else {
        if (!wasRunning || trajectory != lastTrajectory) {
            resetSquareTrajectory(); // fresh start when (re)starting or switching trajectory
        }
        if (trajectory == TRAJ_SQUARE) {
            runSquareTrajectory();
        } else if (trajectory == TRAJ_FORWARD) {
            moveForward(); // continuous straight-line drive, no steps to advance
        } else if (trajectory == TRAJ_MANUAL) {
            driveManual(); // joystick from the dashboard, deadman-protected
        } else {
            moveCircle(); // continuous arc, no discrete steps to advance
        }
    }
    wasRunning = running;
    lastTrajectory = trajectory;

    motors.updateSpeedControl(); // per-wheel velocity PIDs, ticks every PID_PERIOD_MS

    // Without this, loop() never blocks/yields, starving the IDLE task and tripping the
    // Task Watchdog (periodic reboot -> dashboard WiFi/WebSocket disconnects + lag).
    delay(1);
}