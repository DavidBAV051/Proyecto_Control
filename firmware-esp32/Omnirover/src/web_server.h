#pragma once
#include <Arduino.h>

// Starts the WiFi Access Point and the async HTTP/WebSocket server that
// hosts the real-time dashboard (index page + periodic telemetry pushes).
void webServer_init();
