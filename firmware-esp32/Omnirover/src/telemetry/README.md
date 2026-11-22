# Telemetría (estado compartido + JSON de dashboard)

Único snapshot thread-safe de IMU, cámara, encoders y pose. El snapshot se usa
por el servidor WebSocket y por el emisor ESP-NOW.

`CamData` contiene datos reales de `TEL`: `tracking` (`valid`), `target`, `x`,
`y`, ancho y `arrived`. Los encoders almacenan posición, RPM medida y setpoint
activo; la pose usa metros calculados por odometría.

Los productores escriben desde `main.cpp` (`uptime`, IMU, encoders, pose) y
`cam.cpp` (TEL). En la placa base, los mismos setters reciben los structs de
ESP-NOW. No existe telemetría de cámara simulada.

`telemetry_toJson()` publica, entre otros campos:

```json
{
  "t": 12345,
  "imu": {},
  "cam": {"valid": true, "target": "DAVID", "x": 1500, "y": 400, "arrived": false},
  "enc": [{"pos": 0, "vel": 0, "sp": 0}],
  "pose": {"x": 0.512, "y": -0.23}
}
```
