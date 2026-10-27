#include <Arduino.h>
#include "cam.h"
#include "imu.h"
#include "motors.h"
#include "web_server.h"
#include "espnow_link.h"
#include "telemetry.h"
#include "control.h"

// for setMotorSpeed 0, 1, and 2, positive values move forward, negative values move backward
// for setMotorSpeed 3, negative values move forwards, positive values move backwards

// Motor index reference:
//   0 = back right, 1 = back left, 2 = front right, 3 = front left

static const int16_t DEFAULT_DRIVE_SPEED = 190;
static const int16_t DEFAULT_SLIDE_SPEED = 190;
static const int16_t DEFAULT_CIRCLE_SPEED = 190;
static const uint32_t SAFE_START_DELAY_MS = 2000; // wait before first motor command
static const uint32_t MOVE_DURATION_MS = 1000;
static const uint32_t STOP_PAUSE_MS = 500;

void moveCircle(int16_t speed = DEFAULT_CIRCLE_SPEED)
{
    motors.setMotorSpeed(0, speed-100);
    motors.setMotorSpeed(1, speed);
    motors.setMotorSpeed(2, speed-100);
    motors.setMotorSpeed(3, -speed);
}

void moveForward(int16_t speed = DEFAULT_DRIVE_SPEED)
{
    motors.setMotorSpeed(0, speed);
    motors.setMotorSpeed(1, speed);
    motors.setMotorSpeed(2, speed);
    motors.setMotorSpeed(3, -speed);
}

void moveBackward(int16_t speed = DEFAULT_DRIVE_SPEED)
{
    motors.setMotorSpeed(0, -speed);
    motors.setMotorSpeed(1, -speed);
    motors.setMotorSpeed(2, -speed);
    motors.setMotorSpeed(3, speed);
}

void slideRight(int16_t speed = DEFAULT_SLIDE_SPEED)
{
    motors.setMotorSpeed(0, speed);
    motors.setMotorSpeed(1, -speed);
    motors.setMotorSpeed(2, -speed);
    motors.setMotorSpeed(3, -speed);
}

void slideLeft(int16_t speed = DEFAULT_SLIDE_SPEED)
{
    motors.setMotorSpeed(0, -speed);
    motors.setMotorSpeed(1, speed);
    motors.setMotorSpeed(2, speed);
    motors.setMotorSpeed(3, speed);
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
}

// Call every loop() iteration while running the square trajectory; advances to
// the next step once the current one's durationMs has elapsed, looping forever.
void runSquareTrajectory()
{
    uint32_t now = millis();
    if (squareStepIndex < 0) {
        squareStepIndex = 0;
        squareStepStartMs = now;
        SQUARE_TRAJECTORY[0].action();
        return;
    }
    if (now - squareStepStartMs >= SQUARE_TRAJECTORY[squareStepIndex].durationMs) {
        squareStepIndex = (squareStepIndex + 1) % SQUARE_STEP_COUNT;
        squareStepStartMs = now;
        SQUARE_TRAJECTORY[squareStepIndex].action();
    }
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
        } else {
            moveCircle(); // continuous arc, no discrete steps to advance
        }
    }
    wasRunning = running;
    lastTrajectory = trajectory;

    // Without this, loop() never blocks/yields, starving the IDLE task and tripping the
    // Task Watchdog (periodic reboot -> dashboard WiFi/WebSocket disconnects + lag).
    delay(1);
}