# Web server (WiFi AP + dashboard + control HTTP)

Hospeda el access point WiFi y el dashboard en tiempo real (HTML/CSS/JS
servido desde flash, sin dependencias externas/CDN), y expone los endpoints
HTTP que el dashboard usa para subir una foto y arrancar/detener al rover.

Archivos: [`web_server.h`](web_server.h), [`web_server.cpp`](web_server.cpp),
[`web_dashboard_html.h`](web_dashboard_html.h) (HTML/CSS/JS como `PROGMEM`,
un solo string `DASHBOARD_HTML`).

## Qué hace `webServer_init()`

1. Inicializa `telemetry_init()` / `control_init()`.
2. Monta LittleFS (para la foto subida del rover).
3. Levanta el WiFi en modo AP (`WIFI_AP_SSID`/`WIFI_AP_PASSWORD` en
   `include/net_config.h`).
4. Registra rutas HTTP y el WebSocket (`/ws`).
5. Arranca `telemetryTask` (tarea FreeRTOS separada) que, cada
   `TELEMETRY_BROADCAST_INTERVAL_MS`, actualiza encoders y transmite
   `telemetry_toJson()` a todos los clientes WebSocket conectados.

`telemetryTask` corre en su propio núcleo/tarea para que el dashboard siga
respondiendo aunque `main.cpp::loop()` esté ocupado corriendo una secuencia
de motores bloqueante.

## Rutas HTTP

| Ruta | Método | Qué hace |
|---|---|---|
| `/` | GET | Sirve `DASHBOARD_HTML` (con `Cache-Control: no-store` para no servir una versión vieja tras reflashear). |
| `/robot.jpg` | GET | Sirve la foto subida desde LittleFS, o 404 si no hay ninguna. |
| `/upload` | POST (multipart) | Guarda una foto nueva en `/robot.jpg` (tope de 400 KB, `handleRobotImageUpload()`). |
| `/control` | GET | `?cmd=start\|stop\|resetPose\|resetYaw` y/o `?trajectory=forward\|backward\|right\|left` (también acepta `square\|circle`, ocultas del dashboard mientras se valida el PID) -> `control.cpp`/`odometry.cpp`/`imu.cpp`. GET en vez de POST a propósito: el cliente solo hace `fetch()` sin armar body. |
| `/ws` | WebSocket | Servidor->cliente: empuje periódico de telemetría (JSON), ver `telemetry.cpp`. El cliente no envía datos. |

## Dashboard (`web_dashboard_html.h`)

Una sola página en dos columnas. Izquierda (ancha): gráfica de velocidad de
rueda vs. tiempo (canvas `speedCanvas`: RPM medida en línea sólida y setpoint
del PID en línea punteada por motor, útil para ver el escalón/sobreimpulso/
error estacionario al revalidar las ganancias) y debajo la tarjeta de
orientación/trayectoria (canvas `poseCanvas`, dibuja el rover y el trail real
en metros que manda `telemetry.pose`). Derecha: tarjeta de seguimiento de
persona (placeholder hasta que `src/cam/` esté integrado) y controles de
start/stop/trayectoria. La tarjeta de foto del rover se quitó del dashboard
(las rutas `/robot.jpg` y `/upload` siguen en el servidor por si se vuelve a
agregar).
Todo en un solo WebSocket (`applyTelemetry()` despacha cada mensaje a las
funciones `render*` correspondientes).

## Multi-placa (ver `src/espnow_link/README.md`)

Este mismo `web_server.cpp` también sirve el dashboard cuando
`ROVER_LINK_MODE == LINK_MODE_ESPNOW_BASE` — en ese modo `telemetryTask` no
lee sensores locales (los datos llegan por ESP-NOW a `telemetry_set*()` desde
otra placa), solo retransmite lo que ya está en `telemetry_get()`.
