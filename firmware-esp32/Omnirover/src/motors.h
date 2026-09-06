#pragma once
#include <Arduino.h>

// DRV8871 Driver Pins
#define M1_IN1_PIN 18
#define M1_IN2_PIN 19

#define M2_IN1_PIN 26
#define M2_IN2_PIN 27

#define M3_IN1_PIN 23
#define M3_IN2_PIN 25

#define M4_IN1_PIN 4
#define M4_IN2_PIN 5

#define NUM_MOTORS 4

class MotorController {
public:
    void init();
    void setMotorSpeed(uint8_t motorIndex, int16_t speed);
    void stopMotor(uint8_t motorIndex);
    void stopAll();

private:
    static const uint32_t PWM_FREQ = 20000; // 20 kHz ultrasonic frequency
    static const uint8_t  PWM_RES  = 8;     // 8-bit resolution (0 - 255)
};

extern MotorController motors;
