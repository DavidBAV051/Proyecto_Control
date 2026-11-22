#include "espnow_link.h"
#include "net_config.h"

#if ROVER_LINK_MODE != LINK_MODE_LOCAL
#include <WiFi.h>
#include <esp_now.h>
#include "telemetry.h"

static const uint8_t ESPNOW_TELEMETRY_TYPE = 0xA1;
static const uint8_t ESPNOW_COMMAND_TYPE = 0xC1;

struct EspNowPacket {
    uint8_t     type = ESPNOW_TELEMETRY_TYPE;
    uint32_t    uptimeMs = 0;
    IMU_Data    imu = {};
    CamData     cam = {};
    EncoderData  encoders[TELEMETRY_NUM_MOTORS] = {};
    PoseData     pose = {};
    PiLinkStatus piLink = {};
};

static_assert(sizeof(EspNowPacket) <= 250, "ESP-NOW telemetry packet exceeds protocol limit");

struct EspNowCommandPacket {
    uint8_t              type = ESPNOW_COMMAND_TYPE;
    EspNowControlCommand command = ESPNOW_CONTROL_IDLE;
    RoverOperationMode   mode = OPERATION_MODE_CAMERA;
    RoverLandmark        landmark = LANDMARK_NONE;
};

static void addPeer() {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, ESPNOW_PEER_MAC, 6);
    peer.channel = WIFI_AP_CHANNEL;
    peer.encrypt = false;
    if (!esp_now_is_peer_exist(ESPNOW_PEER_MAC)) {
        esp_now_add_peer(&peer);
    }
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    (void)info;
#else
static void onDataRecv(const uint8_t *macAddr, const uint8_t *data, int len) {
    (void)macAddr;
#endif
#if ROVER_LINK_MODE == LINK_MODE_ESPNOW_BASE
    if (len != sizeof(EspNowPacket)) return;
    EspNowPacket packet;
    memcpy(&packet, data, sizeof(packet));
    if (packet.type != ESPNOW_TELEMETRY_TYPE) return;
    telemetry_setUptime(packet.uptimeMs);
    telemetry_setIMU(packet.imu);
    telemetry_setCam(packet.cam);
    telemetry_setPose(packet.pose.x, packet.pose.y);
    telemetry_setPiLink(packet.piLink);
    for (uint8_t motor = 0; motor < TELEMETRY_NUM_MOTORS; motor++) {
        telemetry_setEncoder(motor, packet.encoders[motor]);
    }
#else
    if (len != sizeof(EspNowCommandPacket)) return;
    EspNowCommandPacket packet;
    memcpy(&packet, data, sizeof(packet));
    if (packet.type != ESPNOW_COMMAND_TYPE) return;
    if (packet.command == ESPNOW_CONTROL_MODE) {
        control_selectMode(packet.mode, packet.landmark);
    } else if (packet.command == ESPNOW_CONTROL_START) {
        control_start();
    } else if (packet.command == ESPNOW_CONTROL_IDLE) {
        control_idle();
    }
#endif
}

#if ROVER_LINK_MODE == LINK_MODE_ESPNOW_ROVER
static void espNowSendTask(void *param) {
    (void)param;
    for (;;) {
        RoverTelemetry snap = telemetry_get();
        EspNowPacket packet;
        packet.uptimeMs = snap.uptimeMs;
        packet.imu = snap.imu;
        packet.cam = snap.cam;
        packet.pose = snap.pose;
        packet.piLink = snap.piLink;
        memcpy(packet.encoders, snap.encoders, sizeof(packet.encoders));
        esp_now_send(ESPNOW_PEER_MAC, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_BROADCAST_INTERVAL_MS));
    }
}
#endif

void espNow_init() {
#if ROVER_LINK_MODE == LINK_MODE_ESPNOW_BASE
    WiFi.mode(WIFI_AP_STA);
#else
    WiFi.mode(WIFI_STA);
#endif

    if (esp_now_init() != ESP_OK) return;
    addPeer();
    esp_now_register_recv_cb(onDataRecv);

#if ROVER_LINK_MODE == LINK_MODE_ESPNOW_ROVER
    xTaskCreatePinnedToCore(espNowSendTask, "espnow_tx", 4096, nullptr, 1, nullptr, 0);
#endif
}

void espNow_sendControl(EspNowControlCommand command,
                        RoverOperationMode mode,
                        RoverLandmark landmark) {
#if ROVER_LINK_MODE == LINK_MODE_ESPNOW_BASE
    EspNowCommandPacket packet;
    packet.command = command;
    packet.mode = mode;
    packet.landmark = landmark;
    esp_now_send(ESPNOW_PEER_MAC, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
#else
    (void)command;
    (void)mode;
    (void)landmark;
#endif
}

#else
void espNow_init() {}

void espNow_sendControl(EspNowControlCommand command,
                        RoverOperationMode mode,
                        RoverLandmark landmark) {
    (void)command;
    (void)mode;
    (void)landmark;
}
#endif
