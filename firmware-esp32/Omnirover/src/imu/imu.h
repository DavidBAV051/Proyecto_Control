#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Estructura para los datos de la IMU
struct IMU_Data {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float temp;
    float yaw; // degrees, from BNO08x game rotation vector, 0 = heading at boot/last reset
};

// Funciones de la interfaz de la IMU
bool init_IMU();
void update_IMU();
IMU_Data get_IMU_Data();