#include "imu.h"
#include "pins.h"

// GY-BNO08X breakout: PS1/PS0 are hardwired low on the module (I2C mode is
// the default and only wiring supported by our pin budget). CS is tied to
// VCC and RST to VCC (no MCU-controlled reset pin, per Adafruit's own I2C
// reference wiring), so only SDA/SCL are needed -> reuses the shared bus.
static Adafruit_BNO08x bno08x(-1);
static IMU_Data current_data;
static TwoWire I2C_IMU = TwoWire(0);

// Re-issue the reports we depend on; needed after init and after any
// sensor-triggered reset (BNO08x reports are not persisted across resets).
static void enable_reports() {
    bno08x.enableReport(SH2_ACCELEROMETER, 10000);         // 100Hz, m/s^2 incl. gravity
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);   // 100Hz, rad/s
    bno08x.enableReport(SH2_TEMPERATURE, 1000000);          // 1Hz, deg C
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);   // 100Hz, gyro+accel fused heading (no magnetometer drift ref)
}

// atan2-based yaw extraction from a unit quaternion (same convention Adafruit's
// own quaternion_yaw_pitch_roll example uses).
static float quaternionToYawDeg(float qr, float qi, float qj, float qk) {
    float sqr = qr * qr, sqi = qi * qi, sqj = qj * qj, sqk = qk * qk;
    float yaw = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    return yaw * RAD_TO_DEG;
}

bool init_IMU() {
    // Shared I2C bus (also used by the proximity sensor array) at 400kHz (Fast Mode)
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
    }

    sh2_SensorValue_t event;
    while (bno08x.getSensorEvent(&event)) {
        switch (event.sensorId) {
            case SH2_ACCELEROMETER:
                current_data.accelX = event.un.accelerometer.x;
                current_data.accelY = event.un.accelerometer.y;
                current_data.accelZ = event.un.accelerometer.z;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                current_data.gyroX = event.un.gyroscope.x;
                current_data.gyroY = event.un.gyroscope.y;
                current_data.gyroZ = event.un.gyroscope.z;
                break;
            case SH2_TEMPERATURE:
                current_data.temp = event.un.temperature.value;
                break;
            case SH2_GAME_ROTATION_VECTOR:
                current_data.yaw = quaternionToYawDeg(
                    event.un.gameRotationVector.real,
                    event.un.gameRotationVector.i,
                    event.un.gameRotationVector.j,
                    event.un.gameRotationVector.k);
                break;
        }
    }
}

IMU_Data get_IMU_Data() {
    return current_data;
}
