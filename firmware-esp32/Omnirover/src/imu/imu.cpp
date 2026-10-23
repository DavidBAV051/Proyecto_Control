#include "imu.h"
#include "pins.h"

// GY-BNO08X breakout: PS1/PS0 are hardwired low on the module (I2C mode is
// the default and only wiring supported by our pin budget). This breakout has
// no CS pin. RST must be tied to 3.3V (active-low; GND would hold the chip in
// permanent reset), so only SDA/SCL are needed -> reuses the shared bus.
static Adafruit_BNO08x bno08x(-1);
static IMU_Data current_data;
static TwoWire I2C_IMU = TwoWire(0);

// SH2_ACCELEROMETER is calibrated (bias/scale corrected) but not low-pass filtered,
// so raw samples are still jittery with vibration noise; smooth with an EMA filter.
// Lower alpha = smoother but more lag.
static const float ACCEL_EMA_ALPHA = 0.2f;
static bool accelFilterInit = false;

// Re-issue the reports we depend on; needed after init and after any
// sensor-triggered reset (BNO08x reports are not persisted across resets).
static void enable_reports() {
    bno08x.enableReport(SH2_ACCELEROMETER, 10000);         // 100Hz, m/s^2 incl. gravity
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);   // 100Hz, rad/s
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);   // 100Hz, gyro+accel fused heading (no magnetometer drift ref)
    // SH2_TEMPERATURE dropped: this unit's SH-2 firmware never emits it (enableReport
    // succeeds but no event ever arrives), so current_data.temp stays telemetry's simulated value.
}

// atan2-based yaw extraction from a unit quaternion (same convention Adafruit's
// own quaternion_yaw_pitch_roll example uses).
static float quaternionToYawDeg(float qr, float qi, float qj, float qk) {
    float sqr = qr * qr, sqi = qi * qi, sqj = qj * qj, sqk = qk * qk;
    float yaw = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    return yaw * RAD_TO_DEG;
}

// Keeps a yaw angle in (-180, 180] after subtracting the zero offset below.
static float wrapYawDeg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

// SH2_GAME_ROTATION_VECTOR has no absolute reference (no magnetometer), so it zeroes
// at whatever orientation the chip was in when it booted/reset. We want yaw=0 to mean
// "current heading at our own boot", so capture the first reading as an offset and
// report everything relative to it instead of the chip's own arbitrary reference.
static bool yawZeroed = false;
static float yawOffsetDeg = 0.0f;

bool init_IMU() {
    // Shared I2C bus at 400kHz (Fast Mode)
    if (!I2C_IMU.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000)) {
        return false;
    }

    if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &I2C_IMU)) {
        return false;
    }

    enable_reports();
    return true;
}

void update_IMU() {
    if (bno08x.wasReset()) {
        enable_reports();
        yawZeroed = false; // chip picks a new arbitrary reference on reset, re-zero to it
    }

    sh2_SensorValue_t event;
    while (bno08x.getSensorEvent(&event)) {
        switch (event.sensorId) {
            case SH2_ACCELEROMETER: {
                float ax = event.un.accelerometer.x;
                float ay = event.un.accelerometer.y;
                float az = event.un.accelerometer.z;
                if (!accelFilterInit) {
                    current_data.accelX = ax;
                    current_data.accelY = ay;
                    current_data.accelZ = az;
                    accelFilterInit = true;
                } else {
                    current_data.accelX += ACCEL_EMA_ALPHA * (ax - current_data.accelX);
                    current_data.accelY += ACCEL_EMA_ALPHA * (ay - current_data.accelY);
                    current_data.accelZ += ACCEL_EMA_ALPHA * (az - current_data.accelZ);
                }
                break;
            }
            case SH2_GYROSCOPE_CALIBRATED:
                current_data.gyroX = event.un.gyroscope.x;
                current_data.gyroY = event.un.gyroscope.y;
                current_data.gyroZ = event.un.gyroscope.z;
                break;
            case SH2_GAME_ROTATION_VECTOR: {
                float rawYaw = quaternionToYawDeg(
                    event.un.gameRotationVector.real,
                    event.un.gameRotationVector.i,
                    event.un.gameRotationVector.j,
                    event.un.gameRotationVector.k);
                if (!yawZeroed) {
                    yawOffsetDeg = rawYaw;
                    yawZeroed = true;
                }
                current_data.yaw = wrapYawDeg(rawYaw - yawOffsetDeg);
                break;
            }
        }
    }
}

IMU_Data get_IMU_Data() {
    return current_data;
}

void print_IMU_Data() {
    Serial.printf(
        "accel[%.2f, %.2f, %.2f] m/s^2  gyro[%.2f, %.2f, %.2f] rad/s  yaw=%.1fdeg\n",
        current_data.accelX, current_data.accelY, current_data.accelZ,
        current_data.gyroX, current_data.gyroY, current_data.gyroZ,
        current_data.yaw);
}
