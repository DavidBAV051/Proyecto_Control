#include "web_server.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "net_config.h"
#include "telemetry.h"
#include "web_dashboard_html.h"

static AsyncWebServer server(WEB_SERVER_PORT);
static AsyncWebSocket ws("/ws");

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

    server.begin();

    xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 1, nullptr, 0);
}
