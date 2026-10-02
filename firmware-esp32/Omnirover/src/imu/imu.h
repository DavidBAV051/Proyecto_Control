#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Estructura para los datos de la IMU (todos los reports habilitados para debug)
struct IMU_Data {
    float accelX, accelY, accelZ;       // m/s^2, incl. gravity
    float gyroX, gyroY, gyroZ;          // rad/s
    float magX, magY, magZ;             // uT
    float linAccelX, linAccelY, linAccelZ; // m/s^2, gravity removed
    float gravX, gravY, gravZ;          // m/s^2
    float quatReal, quatI, quatJ, quatK; // absolute rotation vector (fused w/ magnetometer)
    float quatAccuracyRad;              // estimated heading error, radians
    float temp;                         // deg C
};

// Funciones de la interfaz de la IMU
bool init_IMU();
void update_IMU();
IMU_Data get_IMU_Data();

// Bring-up/debug helper: dumps every current IMU_Data field to Serial.
void print_IMU_Data();