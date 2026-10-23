#include "espnow_link.h"
#include "net_config.h"

#if ROVER_LINK_MODE != LINK_MODE_LOCAL
#include <WiFi.h>
#include <esp_now.h>
#include "telemetry.h"

// Flat, fixed-size mirror of RoverTelemetry (ESP-NOW payloads are capped at 250 bytes).
struct EspNowPacket {
    uint32_t    uptimeMs;
    IMU_Data    imu;
    CamData     cam;
    EncoderData encoders[TELEMETRY_NUM_MOTORS];
};

#if ROVER_LINK_MODE == LINK_MODE_ESPNOW_BASE
// arduino-esp32 core 3.x (IDF5) passes esp_now_recv_info_t*; core 2.x passes a raw MAC pointer.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    (void)info;
#else
static void onDataRecv(const uint8_t *macAddr, const uint8_t *data, int len) {
    (void)macAddr;
#endif
    if (len != sizeof(EspNowPacket)) return;
    EspNowPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    telemetry_setIMU(pkt.imu);
    telemetry_setCam(pkt.cam);
    for (uint8_t m = 0; m < TELEMETRY_NUM_MOTORS; m++) {
        telemetry_setEncoder(m, pkt.encoders[m]);
    }
}
#else
static void espNowSendTask(void *param) {
    (void)param;
    for (;;) {
        RoverTelemetry snap = telemetry_get();
        EspNowPacket pkt;
        pkt.uptimeMs = snap.uptimeMs;
        pkt.imu = snap.imu;
        pkt.cam = snap.cam;
        memcpy(pkt.encoders, snap.encoders, sizeof(pkt.encoders));
        esp_now_send(ESPNOW_PEER_MAC, reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_BROADCAST_INTERVAL_MS));
    }
}
#endif

void espNow_init() {
    WiFi.mode(WIFI_STA); // ESP-NOW needs the WiFi driver up even without an AP/router connection

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

#if ROVER_LINK_MODE == LINK_MODE_ESPNOW_BASE
    telemetry_init();
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("ESP-NOW base station: waiting for rover telemetry...");
#else
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, ESPNOW_PEER_MAC, 6);
    peer.channel = WIFI_AP_CHANNEL;
    peer.encrypt = false;
    if (!esp_now_is_peer_exist(ESPNOW_PEER_MAC)) {
        esp_now_add_peer(&peer);
    }
    xTaskCreatePinnedToCore(espNowSendTask, "espnow_tx", 4096, nullptr, 1, nullptr, 0);
    Serial.println("ESP-NOW rover link: streaming telemetry to base station");
#endif
}

#else
void espNow_init() {
    // LINK_MODE_LOCAL: telemetry is produced and served on this same board, nothing to do.
}
#endif
