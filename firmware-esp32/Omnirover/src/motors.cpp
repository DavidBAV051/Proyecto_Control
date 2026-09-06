#include "motors.h"

MotorController motors;

// Pin mapping table: [MotorIndex][IN1, IN2]
static const uint8_t driverPins[NUM_MOTORS][2] = {
    {M1_IN1_PIN, M1_IN2_PIN},
    {M2_IN1_PIN, M2_IN2_PIN},
    {M3_IN1_PIN, M3_IN2_PIN},
    {M4_IN1_PIN, M4_IN2_PIN}
};

// ESP32 LEDC PWM channel mapping (2 channels per DRV8871)
static const uint8_t pwmChannels[NUM_MOTORS][2] = {
    {0, 1},
    {2, 3},
    {4, 5},
    {6, 7}
};

void MotorController::init() {
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        for (uint8_t ch = 0; ch < 2; ch++) {
            ledcSetup(pwmChannels[m][ch], PWM_FREQ, PWM_RES);
            ledcAttachPin(driverPins[m][ch], pwmChannels[m][ch]);
            ledcWrite(pwmChannels[m][ch], 0);
        }
    }
}

void MotorController::setMotorSpeed(uint8_t motorIndex, int16_t speed) {
    if (motorIndex >= NUM_MOTORS) return;

    speed = constrain(speed, -255, 255);
    uint8_t ch1 = pwmChannels[motorIndex][0];
    uint8_t ch2 = pwmChannels[motorIndex][1];

    if (speed > 0) {
        // Forward rotation
        ledcWrite(ch1, (uint32_t)speed);
        ledcWrite(ch2, 0);
    } else if (speed < 0) {
        // Reverse rotation
        ledcWrite(ch1, 0);
        ledcWrite(ch2, (uint32_t)(-speed));
    } else {
        // Coast / Stop
        ledcWrite(ch1, 0);
        ledcWrite(ch2, 0);
    }
}

void MotorController::stopMotor(uint8_t motorIndex) {
    setMotorSpeed(motorIndex, 0);
}

void MotorController::stopAll() {
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        stopMotor(m);
    }
}
