#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Estructura para los datos de la IMU
struct IMU_Data {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float temp;
};

// Funciones de la interfaz de la IMU
bool init_IMU();
void update_IMU();
IMU_Data get_IMU_Data();