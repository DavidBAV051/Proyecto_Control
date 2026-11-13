# Cámara (Raspberry Pi 4 por UART)

Enlace serie con una Raspberry Pi 4 que corre la visión por computadora
(seguimiento de persona) y manda las detecciones al ESP32 por UART2.
Reemplaza al HuskyLens (librería y código retirados el 2026-11-12).

Archivos: [`cam.h`](cam.h), [`cam.cpp`](cam.cpp)

## Cableado

| Pi 4 | ESP32 | Nota |
|---|---|---|
| GPIO14 (TXD) | GPIO16 (RX2) | `PI_UART_RX_PIN` en `include/pins.h` |
| GPIO15 (RXD) | GPIO17 (TX2) | `PI_UART_TX_PIN` |
| GND | GND | común en el punto estrella de la alimentación |

Ambos lados son lógica de 3.3 V: sin level shifter. Baud `PI_UART_BAUD`
(115200). En la Pi usar el PL011 (`dtoverlay=disable-bt` → `/dev/ttyAMA0`),
no el mini-UART: su baud deriva con el reloj del CPU y una Pi haciendo CV
está throttleando todo el tiempo.

## Estado actual: eco crudo por USB

- `cam_init()` abre UART2; `cam_poll()` (llamado cada `loop()`) drena el
  puerto sin bloquear, junta bytes hasta `\n`/`\r` y escribe cada línea
  completa tal cual en el monitor serie USB. Líneas de más de 128 bytes sin
  terminador se descartan.
- **El monitor USB está reservado para este eco**: todos los demás
  `Serial.print` del firmware (IMU, reset reason, WiFi, WS, safe-start) se
  quitaron para que lo único que aparezca sea lo que manda la Pi. Si el
  BNO08x no responde en `setup()` el firmware se queda en un `while(1)`
  silencioso.
- Todavía no se parsea nada: el dashboard sigue mostrando
  `telemetry_simulateCam()`.

## Siguiente paso

Definir el formato de línea con la Pi (p. ej. `T,x,y,w,h` / `N`) y en
`cam_poll()` convertirlo a `CamData` (`telemetry.h`) → `telemetry_setCam()`,
quitando `telemetry_simulateCam()` de `web_server.cpp`.
