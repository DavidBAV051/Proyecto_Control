# Web server (WiFi AP + dashboard + control HTTP)

Hospeda la GUI embebida y publica el snapshot de telemetría por WebSocket. La
lectura de encoders y sensores vive exclusivamente en `main.cpp`; la tarea web
solo retransmite el snapshot.

## Control HTTP

| Ruta | Acción |
|---|---|
| `/control?mode=CAMERA` | Selecciona seguimiento de AprilTag. |
| `/control?mode=LANDMARK&landmark=TOP_RIGHT` | Selecciona un destino válido. |
| `/control?cmd=start` | Solicita START después del handshake READY. |
| `/control?cmd=idle` | Detiene y vuelve a estado IDLE. |
| `/control?cmd=resetPose` | Reinicia odometría local. |
| `/control?cmd=resetYaw` | Re-captura el cero de yaw. |
| `/ws` | Push periódico de telemetría al navegador. |

En modo `LINK_MODE_ESPNOW_BASE`, los comandos de modo, START e IDLE se transmiten
a la placa rover por ESP-NOW. En local se aplican directamente al estado compartido.

## Dashboard

La tarjeta de control ofrece `CAMERA` o `LANDMARK`, los cinco landmarks exactos y
botones `START`/`IDLE`. La vista de visión muestra `tracking`, `target`, `tag_x`,
`tag_y` y `arrived`; no usa detecciones simuladas. Las RPM medidas/setpoint y el
trail de odometría continúan actualizándose por el mismo WebSocket.
