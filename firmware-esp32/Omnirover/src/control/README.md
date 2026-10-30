# Control (estado run/stop + trayectoria seleccionada)

Guarda el estado compartido que el dashboard usa para arrancar/detener al
rover y elegir qué trayectoria demo corre, de forma segura entre tareas
FreeRTOS (el servidor web corre en una tarea, `loop()` en otra).

Archivos: [`control.h`](control.h), [`control.cpp`](control.cpp)

## Qué guarda

- `g_running` (bool): si el rover debe estar ejecutando la trayectoria activa.
- `g_trajectory` (`RoverTrajectory`): `TRAJ_SQUARE`, `TRAJ_CIRCLE` o `TRAJ_FORWARD`
  (avance recto continuo, sin pasos — ver `main.cpp::loop()`).

Ambos protegidos por un mutex (`g_mutex`) porque se escriben desde el handler
HTTP `/control` (tarea del servidor web, ver `web_server.cpp`) y se leen desde
`loop()` (`main.cpp`) en cada iteración.

## Flujo real

1. El dashboard llama `GET /control?cmd=start` o `?trajectory=circle`.
2. `web_server.cpp` traduce eso a `control_start()` / `control_stop()` /
   `control_setTrajectory()`.
3. `main.cpp::loop()` pregunta `control_isRunning()` / `control_getTrajectory()`
   cada ciclo y decide si llamar `runSquareTrajectory()` o `moveCircle()`.

## Nota

Este módulo solo guarda estado — no mueve motores ni conoce las trayectorias
en sí; esa lógica vive en `main.cpp` (`moveForward`/`moveCircle`/
`runSquareTrajectory`/etc.).
