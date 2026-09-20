#include <Arduino.h>
#include "HUSKYLENS.h"
#include "pins.h"

HUSKYLENS huskylens;
HardwareSerial HuskySerial(2);

void initHuskyLens()
{
    HuskySerial.begin(9600, SERIAL_8N1, HUSKYLENS_RX2_PIN, HUSKYLENS_TX2_PIN);

    while (!huskylens.begin(HuskySerial))
    {
        Serial.println("HuskyLens not connected!");
        delay(1000);
    }

    Serial.println("HuskyLens connected");
}

void updateHuskyLens()
{
    if (!huskylens.request())
    {
        Serial.println("Request failed");
    }
    else if (!huskylens.available())
    {
        Serial.println("No object");
    }
    else
    {
        HUSKYLENSResult result = huskylens.read();

        Serial.print("ID=");
        Serial.print(result.ID);

        Serial.print(" X=");
        Serial.print(result.xCenter);

        Serial.print(" Y=");
        Serial.println(result.yCenter);

        Serial.print("Width=");
        Serial.print(result.width);

        Serial.print(" Height=");
        Serial.println(result.height);
    }

    delay(100);
}