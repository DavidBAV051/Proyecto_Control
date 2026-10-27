# ESP-NOW link (transporte opcional rover <-> base)

Permite separar "quién lee los sensores" de "quién sirve el dashboard",
usando ESP-NOW (protocolo peer-to-peer de Espressif, sin necesidad de router)
como enlace entre dos placas ESP32.

Archivos: [`espnow_link.h`](espnow_link.h), [`espnow_link.cpp`](espnow_link.cpp)

## 3 modos (elegidos en build time con `ROVER_LINK_MODE`, `include/net_config.h`)

| Modo | Qué hace esta placa |
|---|---|
| `LINK_MODE_LOCAL` (default) | Lee sensores y sirve el dashboard, todo en una sola placa. `espNow_init()` es un no-op. |
| `LINK_MODE_ESPNOW_ROVER` | Lee sensores reales, NO sirve dashboard; empaqueta `RoverTelemetry` en un `EspNowPacket` fijo (<250 bytes, límite de ESP-NOW) y lo envía por radio cada `TELEMETRY_BROADCAST_INTERVAL_MS` a `ESPNOW_PEER_MAC`. |
| `LINK_MODE_ESPNOW_BASE` | No tiene sensores; registra un callback que recibe paquetes y los vuelca a `telemetry_setIMU/Cam/Encoder()`, y sirve el mismo dashboard HTML de siempre sobre su propio WiFi AP. |

Para probar un modo sin editar archivos:
`$env:PLATFORMIO_BUILD_FLAGS="-D ROVER_LINK_MODE=1"; pio run` (1 = ESPNOW_ROVER, 2 = ESPNOW_BASE).

## Por qué un struct "plano" (`EspNowPacket`)

ESP-NOW limita el payload a 250 bytes y no serializa structs con punteros o
tamaño variable — por eso `EspNowPacket` es una copia fija de los campos de
`RoverTelemetry` (sin JSON, sin `String`), armada con `memcpy`.

## Detalles no obvios

- `WiFi.mode(WIFI_STA)` se necesita aunque no haya AP/router — ESP-NOW
  requiere que el driver WiFi esté levantado.
- La firma del callback de recepción (`onDataRecv`) cambió entre
  arduino-esp32 core 2.x y 3.x (IDF5); el `#if ESP_ARDUINO_VERSION_MAJOR >= 3`
  cubre ambas.
- `ESPNOW_PEER_MAC` en `net_config.h` todavía es la dirección broadcast
  (`FF:FF:FF:FF:FF:FF`) — reemplazar por la MAC real del par una vez
  conocida, para no transmitir a todos los receptores ESP-NOW del canal.
