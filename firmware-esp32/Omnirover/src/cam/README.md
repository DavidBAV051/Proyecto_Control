# Cámara y control (Raspberry Pi 4 por UART)

Implementa `SAOAH_UART_V2` entre la Raspberry Pi 4 y el ESP32 por UART2. Cada
mensaje es un JSON compacto terminado en `\n`, a 115200 baud, 8N1. El parser
usa ArduinoJson y drena el UART sin bloqueo desde `loop()`.

## Cableado

| Pi 4 | ESP32 | Nota |
|---|---|---|
| GPIO14 (TXD) | GPIO16 (RX2) | `PI_UART_RX_PIN` |
| GPIO15 (RXD) | GPIO17 (TX2) | `PI_UART_TX_PIN` |
| GND | GND | común en el punto estrella de la alimentación |

## Secuencia ESP32 -> Pi

1. Al elegir modo: `{"type":"MODE","mode":"CAMERA"}` o
   `{"type":"MODE","mode":"LANDMARK","landmark":"TOP_RIGHT"}`.
2. Se espera `READY` con protocolo, modo, landmark y `motion:true` coincidentes.
3. Tras `READY` y START del usuario: `{"type":"START"}`.
4. Con 25 Hz: `{"type":"POSE","x":0.512,"y":-0.230,"theta":1.5708}`.
5. IDLE envía `{"type":"IDLE"}`.

## Secuencia Pi -> ESP32

`TEL` aporta `tracking`, `target`, coordenadas, `arrived` y `wheels` en rad/s.
Los cuatro valores se copian directamente a los setpoints de los PIDs. Cualquier
TEL con estado `STOP`/`IDLE`, `wheels` inválido o ausencia de TEL durante más de
500 ms deja los cuatro setpoints en cero.

El parser descarta líneas de más de 512 bytes y limpia fragmentos incompletos.
`tracking`, `target`, `tag_x`, `tag_y` y `arrived` se publican en `CamData` para
la GUI local o por ESP-NOW.
