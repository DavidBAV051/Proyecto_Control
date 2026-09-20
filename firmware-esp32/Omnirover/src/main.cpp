#include <Arduino.h>
#include "cam.h"
#include "imu.h"
#include "motors.h"

// for setMotorSpeed 0 and 1, negative values move forward, positive values move backward
// for setMotorSpeed 2 and 3, negative values move backward, positive values move forward

// Motor index reference:
//   0 = back right, 1 = back left, 2 = front right, 3 = front left

static const int16_t DEFAULT_DRIVE_SPEED = 190;
static const int16_t DEFAULT_SLIDE_SPEED = 190;
static const uint32_t SAFE_START_DELAY_MS = 2000; // wait before first motor command
static const uint32_t MOVE_DURATION_MS = 1000;
static const uint32_t STOP_PAUSE_MS = 500;

void moveForward(int16_t speed = DEFAULT_DRIVE_SPEED)
{
    motors.setMotorSpeed(0, -speed);
    motors.setMotorSpeed(1, -speed);
    motors.setMotorSpeed(2,  speed);
    motors.setMotorSpeed(3,  speed);
}

void moveBackward(int16_t speed = DEFAULT_DRIVE_SPEED)
{
    motors.setMotorSpeed(0,  speed);
    motors.setMotorSpeed(1,  speed);
    motors.setMotorSpeed(2, -speed);
    motors.setMotorSpeed(3, -speed);
}

void slideRight(int16_t speed = DEFAULT_SLIDE_SPEED)
{
    motors.setMotorSpeed(0, -speed);
    motors.setMotorSpeed(1,  speed);
    motors.setMotorSpeed(2, -speed);
    motors.setMotorSpeed(3,  speed);
}

void slideLeft(int16_t speed = DEFAULT_SLIDE_SPEED)
{
    motors.setMotorSpeed(0,  speed);
    motors.setMotorSpeed(1, -speed);
    motors.setMotorSpeed(2,  speed);
    motors.setMotorSpeed(3, -speed);
}

void stopAllMotors()
{
    motors.stopAll();
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
    // initHuskyLens();
    // if (init_IMU()) {
    //     Serial.println("¡IMU online y filtrada a 10Hz!");
    // } else {
    //     Serial.println("Error: No se detecto la MPU6050");
    //     while(1);
    // }
    motors.init();
    motors.stopAll();
    Serial.println("DRV8871 Motor Controller Ready.");

    waitForSafeStart();
}

void loop()
{
    moveForward();
    delay(MOVE_DURATION_MS);
    stopAllMotors();
    delay(STOP_PAUSE_MS);

    slideRight();
    delay(MOVE_DURATION_MS);
    stopAllMotors();
    delay(STOP_PAUSE_MS);

    moveBackward();
    delay(MOVE_DURATION_MS);
    stopAllMotors();
    delay(STOP_PAUSE_MS);

    slideLeft();
    delay(MOVE_DURATION_MS);
    stopAllMotors();
    delay(STOP_PAUSE_MS);
}