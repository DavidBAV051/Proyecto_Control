# Cámara (HuskyLens)

Interfaz mínima para un sensor HuskyLens (reconocimiento de objetos/personas
por UART) conectado al UART2 del ESP32.

Archivos: [`cam.h`](cam.h), [`cam.cpp`](cam.cpp)

## Estado actual: NO está integrado

- `initHuskyLens()` / `updateHuskyLens()` existen pero **no se llaman desde
  `main.cpp`** (están comentadas) ni alimentan `telemetry.h` — el dashboard
  muestra datos de cámara simulados (`telemetry_simulateCam()` en
  `telemetry.cpp`), no detecciones reales.
- `initHuskyLens()` bloquea en un `while` hasta que el sensor responda — no
  debe llamarse hasta que el HuskyLens esté físicamente conectado, o el
  `setup()` se quedará colgado ahí para siempre.
- `updateHuskyLens()` solo imprime por Serial (`ID`, `X`, `Y`, `Width`,
  `Height`) y tiene un `delay(100)` bloqueante — si se usa debe adaptarse a
  no bloquear `loop()` (ver la nota de "loop() must always yield" en la
  memoria del repo) y a llamar `telemetry_setCam()` con los datos reales.

## Para integrarlo

1. Llamar `initHuskyLens()` en `setup()` (después de que el hardware esté
   conectado) en vez de dejarlo comentado.
2. Reemplazar los `Serial.print` de `updateHuskyLens()` por
   `telemetry_setCam({...})` con los campos de `CamData` (`telemetry.h`).
3. Quitar `telemetry_simulateCam()` de `web_server.cpp` una vez esto esté
   feeding datos reales.

Pines UART2 (RX2/TX2) están centralizados en `include/pins.h`.
