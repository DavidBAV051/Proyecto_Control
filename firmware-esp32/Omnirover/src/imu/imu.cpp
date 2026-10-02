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
    bno08x.enableReport(SH2_ACCELEROMETER, 10000);              // 100Hz, m/s^2 incl. gravity
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);        // 100Hz, rad/s
    bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000);   // 100Hz, uT
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);         // 100Hz, m/s^2, gravity removed
    bno08x.enableReport(SH2_GRAVITY, 10000);                     // 100Hz, m/s^2
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);             // 100Hz, absolute quaternion (uses magnetometer)
    bno08x.enableReport(SH2_TEMPERATURE, 1000000);                // 1Hz, deg C
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
            case SH2_MAGNETIC_FIELD_CALIBRATED:
                current_data.magX = event.un.magneticField.x;
                current_data.magY = event.un.magneticField.y;
                current_data.magZ = event.un.magneticField.z;
                break;
            case SH2_LINEAR_ACCELERATION:
                current_data.linAccelX = event.un.linearAcceleration.x;
                current_data.linAccelY = event.un.linearAcceleration.y;
                current_data.linAccelZ = event.un.linearAcceleration.z;
                break;
            case SH2_GRAVITY:
                current_data.gravX = event.un.gravity.x;
                current_data.gravY = event.un.gravity.y;
                current_data.gravZ = event.un.gravity.z;
                break;
            case SH2_ROTATION_VECTOR:
                current_data.quatReal        = event.un.rotationVector.real;
                current_data.quatI           = event.un.rotationVector.i;
                current_data.quatJ           = event.un.rotationVector.j;
                current_data.quatK           = event.un.rotationVector.k;
                current_data.quatAccuracyRad = event.un.rotationVector.accuracy;
                break;
            case SH2_TEMPERATURE:
                current_data.temp = event.un.temperature.value;
                break;
        }
    }
}

IMU_Data get_IMU_Data() {
    return current_data;
}

void print_IMU_Data() {
    const IMU_Data &d = current_data;
    Serial.println("---- BNO08x ----");
    Serial.printf("Accel      (m/s^2): %8.3f %8.3f %8.3f\n", d.accelX, d.accelY, d.accelZ);
    Serial.printf("Gyro        (rad/s): %8.3f %8.3f %8.3f\n", d.gyroX, d.gyroY, d.gyroZ);
    Serial.printf("Mag           (uT): %8.3f %8.3f %8.3f\n", d.magX, d.magY, d.magZ);
    Serial.printf("Lin. Accel (m/s^2): %8.3f %8.3f %8.3f\n", d.linAccelX, d.linAccelY, d.linAccelZ);
    Serial.printf("Gravity    (m/s^2): %8.3f %8.3f %8.3f\n", d.gravX, d.gravY, d.gravZ);
    Serial.printf("Quaternion (r,i,j,k): %7.4f %7.4f %7.4f %7.4f (acc %.3f rad)\n",
                  d.quatReal, d.quatI, d.quatJ, d.quatK, d.quatAccuracyRad);
    Serial.printf("Temp            (C): %6.2f\n", d.temp);
}
