# ESP-NOW link (rover <-> base)

Separa el rover que ejecuta sensores, UART y PIDs de la placa que hospeda la GUI.
El enlace es bidireccional:

- Rover -> base: telemetría fija con IMU, encoders, pose y estado de TEL
  (`tracking`, `target`, coordenadas y `arrived`).
- Base -> rover: comandos `MODE`, `START` e `IDLE` con el landmark seleccionado.

## Perfiles

| Modo | Responsabilidad |
|---|---|
| `LINK_MODE_LOCAL` (0) | Rover y GUI en una sola placa. |
| `LINK_MODE_ESPNOW_ROVER` (1) | Sin servidor web; transmite telemetría y recibe comandos. |
| `LINK_MODE_ESPNOW_BASE` (2) | Sin sensores ni motores; sirve GUI, transmite comandos y recibe telemetría. |

Compilar rover/base sin editar el proyecto:

```powershell
$env:PLATFORMIO_BUILD_FLAGS="-DROVER_LINK_MODE=1"; pio run
$env:PLATFORMIO_BUILD_FLAGS="-DROVER_LINK_MODE=2"; pio run
```

Los payloads siguen siendo structs planos menores de 250 bytes. La base usa
`WIFI_AP_STA` para conservar simultáneamente el access point y ESP-NOW; el rover
usa modo STA. `ESPNOW_PEER_MAC` debe cambiarse de broadcast a las MAC reales del
par en `include/net_config.h` cuando se cierre la instalación.
