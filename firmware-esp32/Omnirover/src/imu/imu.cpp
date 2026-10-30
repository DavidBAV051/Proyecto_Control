#include "imu.h"
#include "pins.h"

// GY-BNO08X breakout, I2C mode, no CS pin; see README.md and pins.h for wiring.
static Adafruit_BNO08x bno08x(-1);
static IMU_Data current_data;
static TwoWire I2C_IMU = TwoWire(0);

// Raw accel is jittery with vibration; EMA-smoothed (lower alpha = smoother, more lag).
static const float ACCEL_EMA_ALPHA = 0.2f;
static bool accelFilterInit = false;

// Re-issue the reports we depend on; BNO08x doesn't persist them across resets.
static void enable_reports() {
    bno08x.enableReport(SH2_ACCELEROMETER, 10000);         // 100Hz, m/s^2 incl. gravity
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);   // 100Hz, rad/s
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);   // 100Hz, fused heading, no magnetometer
    // SH2_TEMPERATURE dropped: this unit's firmware never emits it.
}

// atan2-based yaw extraction from a unit quaternion. BNO08x's quaternion frame
// is counterclockwise-positive (right-hand rule about +Z); negated here so the
// output matches this project's clockwise-positive/compass convention (see README.md).
static float quaternionToYawDeg(float qr, float qi, float qj, float qk) {
    float sqr = qr * qr, sqi = qi * qi, sqj = qj * qj, sqk = qk * qk;
    float yaw = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    return -yaw * RAD_TO_DEG;
}

// Keeps a yaw angle in (-180, 180] after subtracting the zero offset below.
static float wrapYawDeg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

// Game rotation vector has no absolute reference; zero it to our own boot heading
// (see README.md). Capture the first reading as an offset and report relative to it.
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
