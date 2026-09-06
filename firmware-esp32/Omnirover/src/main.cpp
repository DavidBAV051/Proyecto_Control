#include <Arduino.h>
#include "cam.h"
#include "imu.h"
#include "motors.h"

void setup()
{
    Serial.begin(115200);
    // initHuskyLens();
    // if (init_IMU()) {
    //     Serial.println("¡IMU online y filtrada a 10Hz!");
    // } else {
    //     Serial.println("Error: No se detecto la MPU6050");
    //     while(1);
    // }
    motors.init();
    Serial.println("DRV8871 Motor Controller Ready.");
}

void loop()
{
    // updateHuskyLens();
    // Spin all 4 motors forward at ~75% duty cycle (speed 190)
    // for (int i = 0; i < NUM_MOTORS; i++) {
    //     motors.setMotorSpeed(i, 190);
    // }
    motors.setMotorSpeed(0, -190);
    motors.setMotorSpeed(1, -190);
    motors.setMotorSpeed(2, 190);
    motors.setMotorSpeed(3, 190);
    delay(3000);

    // Stop all motors
    motors.stopAll();
    delay(1500);

    // Spin all 4 motors in reverse at speed -190
    // for (int i = 0; i < NUM_MOTORS; i++) {
    //     motors.setMotorSpeed(i, -190);
    // }
    // motors.setMotorSpeed(0, -190);
    // motors.setMotorSpeed(1, 190);
    // motors.setMotorSpeed(2, -190);
    // motors.setMotorSpeed(3, 190);
    // delay(3000);

    // // Stop all motors
    // motors.stopAll();
    // delay(2000);
}