#include <Arduino.h>
#include "cam.h"
#include "imu.h"

void setup()
{
    Serial.begin(115200);
    initHuskyLens();
    if (init_IMU()) {
        Serial.println("¡IMU online y filtrada a 10Hz!");
    } else {
        Serial.println("Error: No se detecto la MPU6050");
        while(1);
    }
}

void loop()
{
    updateHuskyLens();
}