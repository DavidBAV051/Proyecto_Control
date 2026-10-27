# Telemetría (estado compartido + serialización JSON)

Único punto de verdad para "qué está pasando en el rover ahora mismo" (IMU,
cámara, encoders, pose), leído/escrito de forma segura entre tareas FreeRTOS
(IMU y encoders se actualizan desde `loop()`/`telemetryTask`, el servidor web
y ESP-NOW leen desde sus propias tareas).

Archivos: [`telemetry.h`](telemetry.h), [`telemetry.cpp`](telemetry.cpp)

## Qué guarda (`RoverTelemetry`)

- `imu` (`IMU_Data`, ver `src/imu/`)
- `cam` (`CamData`, placeholder simulado hasta que `src/cam/` esté integrado)
- `encoders[4]` (`EncoderData`: posición en pulsos + RPM, ver `src/motors/`)
- `pose` (`PoseData`: x/y en metros, ver `src/odometry/`)

Todo protegido por un único mutex (`g_mutex`); `telemetry_get()` devuelve una
copia completa (snapshot), no una referencia, para que el llamador no tenga
que mantener el lock mientras la usa.

## API

- `telemetry_init()`: crea el mutex (llamar una vez, antes de usar lo demás).
- `telemetry_setIMU/Cam/Encoder/Pose(...)`: setters thread-safe, llamados
  desde donde se produce cada dato real (`imu.cpp`, `motors.cpp`,
  `odometry.cpp`) o desde `espnow_link.cpp` en modo `LINK_MODE_ESPNOW_BASE`.
- `telemetry_get()`: snapshot completo, usado por el dashboard y por el envío
  ESP-NOW.
- `telemetry_toJson()`: serializa el snapshot a un JSON compacto
  (`ArduinoJson`) que el dashboard consume por WebSocket.
- `telemetry_simulateCam()`: genera una detección falsa (seno/coseno) porque
  el HuskyLens (`src/cam/`) todavía no está cableado — quitar cuando lo esté.

## Formato JSON (ver `telemetry_toJson()`)

```json
{ "t": 12345, "imu": {...}, "cam": {...}, "enc": [{"pos":.., "vel":..}, ...], "pose": {"x":.., "y":..} }
```
