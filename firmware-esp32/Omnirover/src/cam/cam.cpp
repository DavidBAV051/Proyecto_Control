#include "cam.h"
#include "pins.h"
#include "telemetry.h"
#include <ArduinoJson.h>
#include <math.h>

static HardwareSerial piSerial(2);
static String lineBuf;
static bool discardingLine = false;
static uint32_t lastByteMs = 0;
static const size_t PI_LINE_MAX = 512;
static const uint32_t PARTIAL_FLUSH_MS = 250;
static const uint32_t TEL_TIMEOUT_MS = 500;

static RoverOperationMode requestedMode = OPERATION_MODE_CAMERA;
static RoverLandmark requestedLandmark = LANDMARK_NONE;
static bool ready = false;
static bool started = false;
static bool telReceived = false;
static bool telAllowsMotion = false;
static uint32_t lastTelMs = 0;
static float wheelSpeeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static char lastTelState[12] = "NONE";
static char lastIssue[24] = "WAIT_MODE";

static const char *modeName(RoverOperationMode mode) {
    return mode == OPERATION_MODE_LANDMARK ? "LANDMARK" : "CAMERA";
}

static const char *landmarkName(RoverLandmark landmark) {
    switch (landmark) {
        case LANDMARK_TOP_RIGHT: return "TOP_RIGHT";
        case LANDMARK_TOP_LEFT: return "TOP_LEFT";
        case LANDMARK_DOWN_RIGHT: return "DOWN_RIGHT";
        case LANDMARK_DOWN_LEFT: return "DOWN_LEFT";
        case LANDMARK_CENTER: return "CENTER";
        default: return "NONE";
    }
}

static void sendDocument(const JsonDocument &doc) {
    serializeJson(doc, piSerial);
    piSerial.write('\n');
}

static RoverOperationMode parseMode(const char *name) {
    return strcmp(name, "LANDMARK") == 0 ? OPERATION_MODE_LANDMARK : OPERATION_MODE_CAMERA;
}

static RoverLandmark parseLandmark(const char *name) {
    if (strcmp(name, "TOP_RIGHT") == 0) return LANDMARK_TOP_RIGHT;
    if (strcmp(name, "TOP_LEFT") == 0) return LANDMARK_TOP_LEFT;
    if (strcmp(name, "DOWN_RIGHT") == 0) return LANDMARK_DOWN_RIGHT;
    if (strcmp(name, "DOWN_LEFT") == 0) return LANDMARK_DOWN_LEFT;
    if (strcmp(name, "CENTER") == 0) return LANDMARK_CENTER;
    return LANDMARK_NONE;
}

static void clearMotionState() {
    ready = false;
    started = false;
    telReceived = false;
    telAllowsMotion = false;
    memset(wheelSpeeds, 0, sizeof(wheelSpeeds));
    strncpy(lastTelState, "NONE", sizeof(lastTelState) - 1);
    strncpy(lastIssue, "WAIT_READY", sizeof(lastIssue) - 1);

    CamData cam;
    strncpy(cam.target, "NONE", sizeof(cam.target) - 1);
    telemetry_setCam(cam);
}

static void processTel(JsonDocument &doc) {
    CamData cam;
    cam.valid = doc["tracking"] | false;
    cam.x = doc["tag_x"] | 0;
    cam.y = doc["tag_y"] | 0;
    cam.width = static_cast<int16_t>(roundf(doc["width_px"] | 0.0f));
    const char *target = doc["target"] | "NONE";
    strncpy(cam.target, target, sizeof(cam.target) - 1);
    cam.target[sizeof(cam.target) - 1] = '\0';
    cam.arrived = doc["arrived"] | false;
    telemetry_setCam(cam);

    const char *state = doc["state"] | "IDLE";
    strncpy(lastTelState, state, sizeof(lastTelState) - 1);
    lastTelState[sizeof(lastTelState) - 1] = '\0';
    telAllowsMotion = strcmp(state, "STOP") != 0 && strcmp(state, "IDLE") != 0;
    memset(wheelSpeeds, 0, sizeof(wheelSpeeds));
    telReceived = true;
    lastTelMs = millis();
    if (!telAllowsMotion) {
        strncpy(lastIssue, lastTelState, sizeof(lastIssue) - 1);
        return;
    }

    JsonArrayConst incoming = doc["wheels"].as<JsonArrayConst>();
    if (incoming.size() < 4) {
        telAllowsMotion = false;
        strncpy(lastIssue, "WHEELS_MISSING", sizeof(lastIssue) - 1);
        return;
    }

    for (size_t index = 0; index < 4; index++) {
        JsonVariantConst value = incoming[index];
        if (!value.is<float>() && !value.is<int>()) {
            telAllowsMotion = false;
            strncpy(lastIssue, "WHEELS_TYPE", sizeof(lastIssue) - 1);
            return;
        }
        float speed = value.as<float>();
        if (!isfinite(speed)) {
            telAllowsMotion = false;
            strncpy(lastIssue, "WHEELS_NONFINITE", sizeof(lastIssue) - 1);
            return;
        }
        wheelSpeeds[index] = speed;
    }
    strncpy(lastIssue, "WHEELS_OK", sizeof(lastIssue) - 1);
}

static void processLine(const String &line) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, line);
    if (error) {
        strncpy(lastIssue, "JSON_ERROR", sizeof(lastIssue) - 1);
        return;
    }

    const char *type = doc["type"] | "";
    if (strcmp(type, "READY") == 0) {
        const char *proto = doc["proto"] | "";
        const char *mode = doc["mode"] | "";
        bool validMode = strcmp(mode, "CAMERA") == 0 || strcmp(mode, "LANDMARK") == 0;
        RoverOperationMode readyMode = parseMode(mode);
        RoverLandmark readyLandmark = parseLandmark(doc["landmark"] | "NONE");
        bool motion = doc["motion"] | false;
        bool matchingMode = readyMode == requestedMode;
        bool matchingLandmark = requestedMode == OPERATION_MODE_CAMERA || readyLandmark == requestedLandmark;
        ready = validMode && strcmp(proto, "SAOAH_UART_V2") == 0 && motion && matchingMode && matchingLandmark;
        strncpy(lastIssue, ready ? "READY_OK" : "READY_REJECTED", sizeof(lastIssue) - 1);
        return;
    }

    if (strcmp(type, "TEL") == 0) {
        processTel(doc);
    }
}

void cam_init() {
    piSerial.setRxBufferSize(1024);
    piSerial.begin(PI_UART_BAUD, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
    lineBuf.reserve(PI_LINE_MAX);
    clearMotionState();
}

void cam_poll() {
    while (piSerial.available() > 0) {
        char c = static_cast<char>(piSerial.read());
        lastByteMs = millis();
        if (c == '\n' || c == '\r') {
            if (discardingLine) {
                discardingLine = false;
            } else if (lineBuf.length() > 0) {
                processLine(lineBuf);
            }
            lineBuf = "";
        } else if (discardingLine) {
            continue;
        } else if (lineBuf.length() < PI_LINE_MAX) {
            lineBuf += c;
        } else {
            lineBuf = "";
            discardingLine = true;
        }
    }

    if (lineBuf.length() > 0 && millis() - lastByteMs > PARTIAL_FLUSH_MS) {
        lineBuf = "";
    }
}

void cam_requestMode(RoverOperationMode mode, RoverLandmark landmark) {
    clearMotionState();
    strncpy(lastIssue, "WAIT_READY", sizeof(lastIssue) - 1);
    requestedMode = mode;
    requestedLandmark = mode == OPERATION_MODE_LANDMARK ? landmark : LANDMARK_NONE;

    JsonDocument doc;
    doc["type"] = "MODE";
    doc["mode"] = modeName(requestedMode);
    if (requestedMode == OPERATION_MODE_LANDMARK) {
        doc["landmark"] = landmarkName(requestedLandmark);
    }
    sendDocument(doc);
}

bool cam_hasReady(RoverOperationMode mode, RoverLandmark landmark) {
    bool expectedLandmark = mode == OPERATION_MODE_CAMERA || landmark == requestedLandmark;
    return ready && mode == requestedMode && expectedLandmark;
}

bool cam_sendStart() {
    if (!ready || started) return started;
    JsonDocument doc;
    doc["type"] = "START";
    sendDocument(doc);
    started = true;
    telReceived = false;
    telAllowsMotion = false;
    memset(wheelSpeeds, 0, sizeof(wheelSpeeds));
    strncpy(lastIssue, "START_SENT", sizeof(lastIssue) - 1);
    return true;
}

void cam_sendIdle() {
    JsonDocument doc;
    doc["type"] = "IDLE";
    sendDocument(doc);
    clearMotionState();
}

void cam_sendPose(float x, float y, float theta) {
    if (!started) return;
    JsonDocument doc;
    doc["type"] = "POSE";
    doc["x"] = x;
    doc["y"] = y;
    doc["theta"] = theta;
    sendDocument(doc);
}

bool cam_started() {
    return started;
}

bool cam_wheelsFresh() {
    return started && telReceived && telAllowsMotion && millis() - lastTelMs <= TEL_TIMEOUT_MS;
}

void cam_copyWheelSpeeds(float wheelRadS[4]) {
    if (cam_wheelsFresh()) {
        memcpy(wheelRadS, wheelSpeeds, sizeof(wheelSpeeds));
    } else {
        memset(wheelRadS, 0, sizeof(wheelSpeeds));
    }
}

PiLinkStatus cam_getLinkStatus() {
    PiLinkStatus status;
    status.ready = ready;
    status.started = started;
    status.telReceived = telReceived;
    status.fresh = cam_wheelsFresh();
    status.wheelsAccepted = telAllowsMotion;
    status.telAgeMs = telReceived ? millis() - lastTelMs : 0;
    memcpy(status.wheelsRadS, wheelSpeeds, sizeof(wheelSpeeds));
    strncpy(status.state, lastTelState, sizeof(status.state) - 1);
    strncpy(status.issue, lastIssue, sizeof(status.issue) - 1);
    return status;
}