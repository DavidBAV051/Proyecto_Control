#include "motors.h"
#include "telemetry.h"

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

// Per-motor duty cap so a 100% speed request yields FLEET_RPM_CAP on every wheel
static const uint8_t motorMaxDuty[NUM_MOTORS] = {
    (uint8_t)(255UL * FLEET_RPM_CAP / MOTOR_RATED_RPM[0]),
    (uint8_t)(255UL * FLEET_RPM_CAP / MOTOR_RATED_RPM[1]),
    (uint8_t)(255UL * FLEET_RPM_CAP / MOTOR_RATED_RPM[2]),
    (uint8_t)(255UL * FLEET_RPM_CAP / MOTOR_RATED_RPM[3])
};

// ---- Quadrature encoder reading ----
static const uint8_t encoderPinA[NUM_MOTORS] = {M1_ENC_A_PIN, M2_ENC_A_PIN, M3_ENC_A_PIN, M4_ENC_A_PIN};
static const uint8_t encoderPinB[NUM_MOTORS] = {M1_ENC_B_PIN, M2_ENC_B_PIN, M3_ENC_B_PIN, M4_ENC_B_PIN};

static volatile long    encoderCount[NUM_MOTORS]     = {0, 0, 0, 0};
static volatile uint8_t encoderPrevState[NUM_MOTORS] = {0, 0, 0, 0};
static long             lastEncoderCount[NUM_MOTORS]  = {0, 0, 0, 0};
static uint32_t         lastEncoderUpdateMs = 0;

// Full quadrature transition table, indexed by (prevState<<2 | newState) where each
// 2-bit state is (A<<1 | B). Valid single-step transitions map to +-1, everything
// else (no change / both-edges-at-once glitches) maps to 0.
static const int8_t QUAD_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static void IRAM_ATTR onEncoderChange(void *arg) {
    uint8_t m = (uint8_t)(uintptr_t)arg;
    uint8_t state = (digitalRead(encoderPinA[m]) << 1) | digitalRead(encoderPinB[m]);
    uint8_t transition = (encoderPrevState[m] << 2) | state;
    encoderCount[m] += QUAD_TABLE[transition];
    encoderPrevState[m] = state;
}

static void initEncoders() {
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        // M2/M3 sit on input-only GPIOs with no internal pull resistor: they need
        // external 10k pull-ups to 3.3V on the hardware (see pins.h). M1/M4 use
        // regular GPIOs, so the internal pull-up is enough.
        bool inputOnly = (encoderPinA[m] == M2_ENC_A_PIN) || (encoderPinA[m] == M3_ENC_A_PIN);
        pinMode(encoderPinA[m], inputOnly ? INPUT : INPUT_PULLUP);
        pinMode(encoderPinB[m], inputOnly ? INPUT : INPUT_PULLUP);
        encoderPrevState[m] = (digitalRead(encoderPinA[m]) << 1) | digitalRead(encoderPinB[m]);
        attachInterruptArg(digitalPinToInterrupt(encoderPinA[m]), onEncoderChange, (void *)(uintptr_t)m, CHANGE);
        attachInterruptArg(digitalPinToInterrupt(encoderPinB[m]), onEncoderChange, (void *)(uintptr_t)m, CHANGE);
    }
    lastEncoderUpdateMs = millis();
}

void MotorController::init() {
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        for (uint8_t ch = 0; ch < 2; ch++) {
            ledcSetup(pwmChannels[m][ch], PWM_FREQ, PWM_RES);
            ledcAttachPin(driverPins[m][ch], pwmChannels[m][ch]);
            ledcWrite(pwmChannels[m][ch], 0);
        }
    }
    initEncoders();
}

void MotorController::updateEncoderTelemetry() {
    uint32_t now = millis();
    uint32_t elapsedMs = now - lastEncoderUpdateMs;
    if (elapsedMs == 0) return;
    lastEncoderUpdateMs = now;

    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        long count = encoderCount[m]; // 32-bit aligned read, atomic on this core
        long delta = count - lastEncoderCount[m];
        lastEncoderCount[m] = count;

        float countsPerRev = ENCODER_PPR[m] * ENCODER_QUAD_MULT;
        EncoderData enc;
        enc.position    = count;
        enc.velocityRpm = (delta / countsPerRev) * (60000.0f / elapsedMs);
        telemetry_setEncoder(m, enc);
        // Raw A/B levels help tell a disconnected/dead encoder (pins never change)
        // apart from a decode bug (pins toggle but count doesn't move).
        Serial.printf("Enc M%u: %ld pulses (%.0f rpm) | A=%d B=%d\n", m, enc.position,
                      enc.velocityRpm, digitalRead(encoderPinA[m]), digitalRead(encoderPinB[m]));
    }
}

void MotorController::setMotorSpeed(uint8_t motorIndex, int16_t speed) {
    if (motorIndex >= NUM_MOTORS) return;

    speed = constrain(speed, -255, 255);
    uint8_t ch1 = pwmChannels[motorIndex][0];
    uint8_t ch2 = pwmChannels[motorIndex][1];
    uint32_t duty = ((uint32_t)abs(speed) * motorMaxDuty[motorIndex]) / 255;

    if (speed > 0) {
        // Forward rotation
        ledcWrite(ch1, duty);
        ledcWrite(ch2, 0);
    } else if (speed < 0) {
        // Reverse rotation
        ledcWrite(ch1, 0);
        ledcWrite(ch2, duty);
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
