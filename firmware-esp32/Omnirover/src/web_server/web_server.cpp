#include "web_server.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include "net_config.h"
#include "telemetry.h"
#include "web_dashboard_html.h"

static AsyncWebServer server(WEB_SERVER_PORT);
static AsyncWebSocket ws("/ws");

// User-uploaded photo of the rover, stored in flash (LittleFS) so it
// persists across reboots. Capped well under the filesystem partition size.
static const char *ROBOT_IMAGE_PATH = "/robot.jpg";
static const size_t ROBOT_IMAGE_MAX_BYTES = 400 * 1024;

static void handleRobotImageUpload(AsyncWebServerRequest *request, String filename, size_t index,
                                    uint8_t *data, size_t len, bool final) {
    (void)request; (void)filename;
    static File uploadFile;
    static bool tooLarge = false;

    if (index == 0) {
        tooLarge = false;
        uploadFile = LittleFS.open(ROBOT_IMAGE_PATH, FILE_WRITE);
    }

    if (index + len > ROBOT_IMAGE_MAX_BYTES) {
        tooLarge = true;
    } else if (uploadFile) {
        uploadFile.write(data, len);
    }

    if (final) {
        if (uploadFile) uploadFile.close();
        if (tooLarge) {
            LittleFS.remove(ROBOT_IMAGE_PATH);
            Serial.println("Robot photo upload rejected: file too large");
        } else {
            Serial.printf("Robot photo upload saved: %u bytes\n", (unsigned)(index + len));
        }
    }
}

static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                       AwsEventType type, void *arg, uint8_t *data, size_t len) {
    (void)server; (void)arg; (void)data; (void)len;
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WS client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WS client #%u disconnected\n", client->id());
    }
}

// Runs independently of loop() so the dashboard keeps updating in real time
// even while main.cpp is busy running blocking motor demo sequences.
static void telemetryTask(void *param) {
    (void)param;
    for (;;) {
#if ROVER_LINK_MODE != LINK_MODE_ESPNOW_BASE
        telemetry_simulate(); // TODO: drop once real sensors feed telemetry_set*()
#endif
        if (ws.count() > 0) {
            ws.textAll(telemetry_toJson());
        }
        ws.cleanupClients();
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_BROADCAST_INTERVAL_MS));
    }
}

void webServer_init() {
    telemetry_init();

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed, rover photo upload unavailable");
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);
    Serial.print("Dashboard AP SSID: ");
    Serial.println(WIFI_AP_SSID);
    Serial.print("Dashboard AP IP: ");
    Serial.println(WiFi.softAPIP());

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", DASHBOARD_HTML);
    });

    server.on("/robot.jpg", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists(ROBOT_IMAGE_PATH)) {
            request->send(LittleFS, ROBOT_IMAGE_PATH, "image/jpeg");
        } else {
            request->send(404, "text/plain", "no photo uploaded yet");
        }
    });

    server.on(
        "/upload", HTTP_POST,
        [](AsyncWebServerRequest *request) { request->send(200, "text/plain", "ok"); },
        handleRobotImageUpload);

    server.begin();

    xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 1, nullptr, 0);
}
