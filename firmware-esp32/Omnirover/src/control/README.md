# Control (modo, landmark y estado START/IDLE)

Guarda, entre tareas FreeRTOS, la solicitud que llega desde el dashboard local
o desde ESP-NOW.

## Estado compartido

- `mode`: `CAMERA` o `LANDMARK`.
- `landmark`: `TOP_RIGHT`, `TOP_LEFT`, `DOWN_RIGHT`, `DOWN_LEFT` o `CENTER`.
- `modeRevision`: permite que `main.cpp` envíe un único `MODE` por selección.
- `idleRevision`: permite reenviar IDLE aunque el rover ya esté detenido.
- `running`: el usuario dio START; el motor no se activa hasta que UART confirme READY.

Todo el estado se protege con un mutex. `main.cpp` toma un snapshot por ciclo:
primero procesa cambios de modo/IDLE, después el handshake y finalmente los
setpoints de `TEL.wheels`. Este módulo no genera ni modifica velocidades.
