#include <Arduino.h>
#include "cam.h"

void setup()
{
    Serial.begin(115200);
    initHuskyLens();
}

void loop()
{
    updateHuskyLens();
}