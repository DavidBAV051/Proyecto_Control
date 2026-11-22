#include "web_server.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include "net_config.h"
#include "telemetry.h"
#include "motors.h"
#include "control.h"
#include "espnow_link.h"
#include "odometry.h"
#include "imu.h"
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
        if (tooLarge) LittleFS.remove(ROBOT_IMAGE_PATH);
    }
}

static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                       AwsEventType type, void *arg, uint8_t *data, size_t len) {
    (void)server; (void)client; (void)type; (void)arg; (void)data; (void)len;
}

// Runs independently of loop() so the dashboard stays live during blocking
// motor sequences; see README.md.
static void telemetryTask(void *param) {
    (void)param;
    for (;;) {
        // Skip the frame if a client's TX queue is still backed up: on a weak link,
        // queuing more only grows the backlog and the GUI shows ever-staler data.
        if (ws.count() > 0 && ws.availableForWriteAll()) {
            ws.textAll(telemetry_toJson());
        }
        ws.cleanupClients();
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_BROADCAST_INTERVAL_MS));
    }
}

void webServer_init() {
    telemetry_init();
    control_init();

    LittleFS.begin(true); // rover photo storage; upload is simply unavailable if this fails

    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);
    WiFi.setTxPower(WIFI_AP_TX_POWER);
    WiFi.setSleep(false);

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse(200, "text/html", DASHBOARD_HTML);
        // Prevent the browser from caching a stale dashboard across reflashes.
        response->addHeader("Cache-Control", "no-store");
        request->send(response);
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

    server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
        RoverOperationMode mode = OPERATION_MODE_CAMERA;
        RoverLandmark landmark = LANDMARK_NONE;

        if (request->hasParam("mode")) {
            String modeName = request->getParam("mode")->value();
            mode = modeName == "LANDMARK" ? OPERATION_MODE_LANDMARK : OPERATION_MODE_CAMERA;
            if (request->hasParam("landmark")) {
                String landmarkName = request->getParam("landmark")->value();
                if (landmarkName == "TOP_RIGHT") landmark = LANDMARK_TOP_RIGHT;
                else if (landmarkName == "TOP_LEFT") landmark = LANDMARK_TOP_LEFT;
                else if (landmarkName == "DOWN_RIGHT") landmark = LANDMARK_DOWN_RIGHT;
                else if (landmarkName == "DOWN_LEFT") landmark = LANDMARK_DOWN_LEFT;
                else if (landmarkName == "CENTER") landmark = LANDMARK_CENTER;
            }
            control_selectMode(mode, landmark);
            espNow_sendControl(ESPNOW_CONTROL_MODE, mode, landmark);
        }

        if (request->hasParam("cmd")) {
            String cmd = request->getParam("cmd")->value();
            if (cmd == "start") {
                control_start();
                espNow_sendControl(ESPNOW_CONTROL_START);
            } else if (cmd == "idle" || cmd == "stop") {
                control_idle();
                espNow_sendControl(ESPNOW_CONTROL_IDLE);
            } else if (cmd == "resetPose") {
                odometry_reset();
                OdometryPose pose = odometry_getPose();
                telemetry_setPose(pose.x, pose.y);
            } else if (cmd == "resetYaw") {
                imu_zeroYaw();
            }
        }
        request->send(200, "text/plain", "ok");
    });

    server.begin();

    xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 1, nullptr, 0);
}
