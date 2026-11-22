#include <Arduino.h>
#include "cam.h"
#include "imu.h"
#include "motors.h"
#include "web_server.h"
#include "espnow_link.h"
#include "telemetry.h"
#include "control.h"
#include "odometry.h"
#include "net_config.h"

static const uint32_t SAFE_START_DELAY_MS = 2000;

static void pollIMU() {
    update_IMU();
    telemetry_setIMU(get_IMU_Data());
}

static void waitForSafeStart() {
    motors.stopAll();
    uint32_t start = millis();
    while (millis() - start < SAFE_START_DELAY_MS) {
        delay(50);
    }
}

void setup()
{
    Serial.begin(115200);

#if ROVER_LINK_MODE == LINK_MODE_ESPNOW_BASE
    webServer_init();
    espNow_init();
#else
#if ROVER_LINK_MODE == LINK_MODE_LOCAL
    webServer_init();
#else
    telemetry_init();
    control_init();
#endif
    espNow_init();
    cam_init();

    if (!init_IMU()) {
        Serial.println("[boot] BNO08x not detected - halted");
        while (1) { delay(1000); }
    }

    motors.init();
    motors.stopAll();
    waitForSafeStart();
    Serial.println("[boot] SAOAH_UART_V2 ready on GPIO16/17 @115200");
#endif
}

void loop()
{
#if ROVER_LINK_MODE == LINK_MODE_ESPNOW_BASE
    delay(10);
#else
    static uint32_t handledModeRevision = 0;
    static uint32_t handledIdleRevision = 0;
    static bool wasRunning = false;
    static uint32_t lastPoseSendMs = 0;
    static const uint32_t POSE_SEND_PERIOD_MS = 40;

    cam_poll();
    pollIMU();
    motors.updateEncoderTelemetry();
    telemetry_setUptime(millis());

    RoverControlState controlState = control_getState();
    if (controlState.modeRevision != handledModeRevision) {
        handledModeRevision = controlState.modeRevision;
        motors.stopAll();
        cam_requestMode(controlState.mode, controlState.landmark);
        wasRunning = false;
    }

    if (controlState.idleRevision != handledIdleRevision) {
        handledIdleRevision = controlState.idleRevision;
        motors.stopAll();
        cam_sendIdle();
    }

    if (controlState.running) {
        if (!cam_started() && cam_hasReady(controlState.mode, controlState.landmark)) {
            cam_sendStart();
        }

        if (cam_started()) {
            float wheelSpeeds[4];
            cam_copyWheelSpeeds(wheelSpeeds);
            for (uint8_t motor = 0; motor < NUM_MOTORS; motor++) {
                motors.setWheelSpeed(motor, wheelSpeeds[motor]);
            }

            uint32_t now = millis();
            if (now - lastPoseSendMs >= POSE_SEND_PERIOD_MS) {
                lastPoseSendMs = now;
                OdometryPose pose = odometry_getPose();
                cam_sendPose(pose.x, pose.y, get_IMU_Data().yaw * DEG_TO_RAD);
            }
        } else {
            motors.stopAll();
        }
    } else if (wasRunning || cam_started()) {
        motors.stopAll();
    }

    wasRunning = controlState.running;
    telemetry_setPiLink(cam_getLinkStatus());
    motors.updateSpeedControl();
    delay(1);
#endif
}