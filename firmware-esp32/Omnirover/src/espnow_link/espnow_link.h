#pragma once
#include <Arduino.h>

// Initializes ESP-NOW according to ROVER_LINK_MODE (net_config.h):
//  - LINK_MODE_LOCAL: no-op, telemetry is produced/served on this same board.
//  - LINK_MODE_ESPNOW_ROVER: periodically sends the local telemetry snapshot
//    to ESPNOW_PEER_MAC (a base-station board).
//  - LINK_MODE_ESPNOW_BASE: registers a receive callback that feeds incoming
//    telemetry packets into the shared telemetry store used by web_server.cpp.
void espNow_init();
