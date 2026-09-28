#pragma once
#include <Arduino.h>

// =====================================================================
// Rover GUI link configuration
//
// The same dashboard (HTML/JS served from web_server.cpp) can be fed by
// two different transports, selected at build time with ROVER_LINK_MODE:
//
//   LINK_MODE_LOCAL        This board reads/simulates the sensors itself
//                           and hosts the WiFi AP + web dashboard directly.
//                           (default, matches a single-board rover)
//   LINK_MODE_ESPNOW_ROVER This board reads the sensors and only pushes
//                           telemetry over ESP-NOW; it does not run a web
//                           server (saves power, no AP needed on the rover).
//   LINK_MODE_ESPNOW_BASE  This board has no sensors; it receives ESP-NOW
//                           telemetry packets from the rover and re-serves
//                           the exact same dashboard over its own WiFi AP.
// =====================================================================
#define LINK_MODE_LOCAL        0
#define LINK_MODE_ESPNOW_ROVER 1
#define LINK_MODE_ESPNOW_BASE  2

#ifndef ROVER_LINK_MODE
#define ROVER_LINK_MODE LINK_MODE_LOCAL
#endif

// ---- WiFi Access Point (dashboard host) ----
#define WIFI_AP_SSID     "Omnirover"
#define WIFI_AP_PASSWORD "omnirover123" // >=8 chars, use "" for an open network
#define WIFI_AP_CHANNEL  1
#define WEB_SERVER_PORT  80

// ---- Telemetry ----
#define TELEMETRY_BROADCAST_INTERVAL_MS 200 // 5 Hz push rate over WebSocket/ESP-NOW

// ---- ESP-NOW pairing ----
// Replace with the peer's real MAC address once the base/rover pair is known.
// Broadcast address (all 0xFF) reaches every ESP-NOW listener on the channel.
static const uint8_t ESPNOW_PEER_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
