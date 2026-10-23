#include <Arduino.h>
#include "cam.h"
#include "imu.h"
#include "motors.h"
#include "web_server.h"
#include "espnow_link.h"
#include "telemetry.h"

// for setMotorSpeed 0, 1, and 2, positive values move forward, negative values move backward
// for setMotorSpeed 3, negative values move forwards, positive values move backwards

// Motor index reference:
//   0 = back right, 1 = back left, 2 = front right, 3 = front left

static const int16_t DEFAULT_DRIVE_SPEED = 190;
static const int16_t DEFAULT_SLIDE_SPEED = 190;
static const uint32_t SAFE_START_DELAY_MS = 2000; // wait before first motor command
static const uint32_t MOVE_DURATION_MS = 1000;
static const uint32_t STOP_PAUSE_MS = 500;

void moveForward(int16_t speed = DEFAULT_DRIVE_SPEED)
{
    motors.setMotorSpeedRamped(0, speed);
    motors.setMotorSpeedRamped(1, speed);
    motors.setMotorSpeedRamped(2, speed);
    motors.setMotorSpeedRamped(3, -speed);
}

void moveBackward(int16_t speed = DEFAULT_DRIVE_SPEED)
{
    motors.setMotorSpeedRamped(0, -speed);
    motors.setMotorSpeedRamped(1, -speed);
    motors.setMotorSpeedRamped(2, -speed);
    motors.setMotorSpeedRamped(3, speed);
}

void slideRight(int16_t speed = DEFAULT_SLIDE_SPEED)
{
    motors.setMotorSpeedRamped(0, speed);
    motors.setMotorSpeedRamped(1, -speed);
    motors.setMotorSpeedRamped(2, -speed);
    motors.setMotorSpeedRamped(3, -speed);
}

void slideLeft(int16_t speed = DEFAULT_SLIDE_SPEED)
{
    motors.setMotorSpeedRamped(0, -speed);
    motors.setMotorSpeedRamped(1, speed);
    motors.setMotorSpeedRamped(2, speed);
    motors.setMotorSpeedRamped(3, speed);
}

void stopAllMotors()
{
    motors.stopAll();
}

// Non-blocking (no delay()) step sequences: update_IMU()/telemetry_setIMU() need to
// keep running every loop() iteration during a maneuver so the dashboard pose trail
// doesn't freeze while the rover is driving a leg.
struct TrajectoryStep {
    void (*action)();
    uint32_t durationMs;
};

// Traces a square while keeping the chassis heading fixed (strafe instead of turning
// at the corners), using only the drive primitives already defined above.
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

static int8_t trajectoryStepIndex = -1; // -1 = not started yet
static uint32_t trajectoryStepStartMs = 0;

// Call every loop() iteration; advances one step of SQUARE_TRAJECTORY once its
// durationMs has elapsed, then stops (does not repeat).
void runSquareTrajectory()
{
    if (trajectoryStepIndex >= (int8_t)SQUARE_STEP_COUNT) return; // finished

    uint32_t now = millis();
    if (trajectoryStepIndex < 0) {
        trajectoryStepIndex = 0;
        trajectoryStepStartMs = now;
        Serial.println("Square trajectory: step 1/8");
        SQUARE_TRAJECTORY[0].action();
        return;
    }

    if (now - trajectoryStepStartMs >= SQUARE_TRAJECTORY[trajectoryStepIndex].durationMs) {
        trajectoryStepIndex++;
        if (trajectoryStepIndex < (int8_t)SQUARE_STEP_COUNT) {
            trajectoryStepStartMs = now;
            Serial.printf("Square trajectory: step %d/%d\n", trajectoryStepIndex + 1, SQUARE_STEP_COUNT);
            SQUARE_TRAJECTORY[trajectoryStepIndex].action();
        } else {
            Serial.println("Square trajectory: complete.");
        }
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
    motors.init();
    motors.stopAll();

    // IMU bring-up: motors stay idle below while this is being validated.
    if (init_IMU()) {
        Serial.println("BNO08x online.");
    } else {
        Serial.println("Error: BNO08x not detected.");
        while (1) { delay(1000); }
    }

    waitForSafeStart();
}

void loop()
{
    static uint32_t lastPrint = 0;
    update_IMU();
    motors.update(); // advance any in-progress speed ramp
    telemetry_setIMU(get_IMU_Data());
    if (millis() - lastPrint >= 200) {
        lastPrint = millis();
        print_IMU_Data();
    }
    runSquareTrajectory();
    // Without this, loop() never blocks/yields, starving the IDLE task and tripping the
    // Task Watchdog (periodic reboot -> dashboard WiFi/WebSocket disconnects + lag).
    delay(1);
}