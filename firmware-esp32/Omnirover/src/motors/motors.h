#pragma once
#include <Arduino.h>
#include "pins.h"

// DRV8871 driver + quadrature encoder pins are defined centrally in pins.h.
// Motors 1 & 2 encoders use input-only GPIOs (34/35/36/39): require 10 kOhm
// pull-ups to 3.3 V on each line (no internal pull-up available).
// Motors 3 & 4 encoders use regular GPIOs: configure with INPUT_PULLUP in firmware.

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
