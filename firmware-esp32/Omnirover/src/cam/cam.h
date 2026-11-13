#pragma once
#include <Arduino.h>

// Raspberry Pi 4 vision link over UART2; see README.md.

void cam_init();

// Non-blocking: drains UART2 and echoes each complete line to the USB serial
// monitor. Call every loop() iteration.
void cam_poll();