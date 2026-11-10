# Motores (4x DRV8871 + encoders de cuadratura)

Controla las 4 ruedas mecanum (driver DRV8871 por rueda, PWM vía LEDC) y lee
sus encoders de cuadratura para telemetría/odometría.

Archivos: [`motors.h`](motors.h), [`motors.cpp`](motors.cpp)

Índice de motor usado en **todo** el firmware (`main.cpp`, `odometry.cpp`,
dashboard), verificado físicamente el 2026-11-02: `0` = adelante-izquierda,
`1` = adelante-derecha, `2` = atrás-izquierda, `3` = atrás-derecha. Lado
izquierdo = {0, 2}, lado derecho = {1, 3}. (Un comentario anterior decía
0=atrás-derecha… — estaba mal y explicaba por qué el lazo de rumbo salió con
el signo invertido al principio.)

## API

- `init()`: configura los 8 canales LEDC (2 por motor) y los encoders.
- `setWheelSpeed(i, radPerSec)` + `updateSpeedControl()`: **lazo cerrado** de
  velocidad por rueda. Setpoint en rad/s, `+` = adelante para las 4 ruedas
  (el signo del driver de M3 se aplica adentro vía `MOTOR_CMD_DIR`). Se recorta
  a `±MAX_WHEEL_RAD_S` (7.2 rad/s). `updateSpeedControl()` debe llamarse cada
  ciclo de `loop()`; corre los PIDs cada `PID_PERIOD_MS=20`. Es la API que usa
  `main.cpp` para las trayectorias.
- `setMotorSpeed(i, speed)`: lazo abierto, aplica duty inmediatamente, `speed`
  en [-255,255] en "espacio de driver" (M3 invertido).
- `setMotorSpeedRamped(i, target)` + `update()`: igual, pero sube el duty
  gradualmente (`RAMP_TIME_MS=300`) en vez de un salto instantáneo. `update()`
  debe llamarse cada ciclo de `loop()`.
- `stopMotor(i)` / `stopAll()`: paran de inmediato (sin rampa), desactivan el
  PID de esa rueda y resetean su integrador y el estado de rampa.
- `updateEncoderTelemetry()`: calcula RPM real por rueda y pose de odometría;
  llamarlo periódicamente (lo hace `web_server.cpp`'s `telemetryTask`).

## Lazo de velocidad (PID) y tope de PWM en 7.2 rad/s

Por simulación en MATLAB, 7.2 rad/s (~69 RPM) es la velocidad máxima de rueda
sin saturar los actuadores, es decir, **7.2 rad/s ≡ 100% de PWM** en el modelo.
Por eso:

- `MAX_WHEEL_RAD_S = 7.2` es a la vez el tope del setpoint, el tope de la
  salida del PID y el valor que se mapea a duty 255.
- La salida del PID está en rad/s "equivalentes" (saturada a ±7.2) y se mapea
  linealmente a duty: `u / 7.2 * 255` → `setMotorSpeed()`. Ya **no** hay un
  `motorMaxDuty` por motor: la diferencia entre M1 (140 RPM) y el resto la
  absorbe su Ki más alto (4.35 vs 3). `MOTOR_RATED_RPM` queda solo como
  referencia de datasheet (el bus real de la LiPo 1S da bastante menos).
- La medición es `Δcuentas * ENCODER_DIR / CPR * 2π / dt` con cuentas leídas
  directamente del ISR en cada tick de 20 ms (independiente de la ventana de
  100 ms de `updateEncoderTelemetry()`, que sigue alimentando telemetría).
- Ganancias y detalles del controlador: ver [pid/README.md](../pid/README.md).

Lección (2026-11-02): un primer intento escalaba el duty a `69/150 ≈ 46%` máx.
Con la LiPo 1S eso no vence la fricción estática y los motores no arrancaban
aunque el integrador estuviera saturado. No volver a topar el duty por debajo
del 100% sin medir primero la RPM real en vacío al voltaje real del bus.

## Encoders: por qué hay una tabla de transición y un `ENCODER_DIR`

- `QUAD_TABLE`: decodifica cuadratura por flancos (ISR en `CHANGE`, ambos
  canales A/B) — mapea cada transición de 2 bits válida a `+1`/`-1`, e ignora
  transiciones imposibles (glitches/flancos simultáneos).
- `ENCODER_COUNTS_PER_REV[m]`: distinto por motor, medido físicamente (M1 es
  el motor de reemplazo, con una relación de engranes distinta).
- `ENCODER_DIR[m] = {-1,1,-1,1}`: los motores 0 y 2 tienen A/B cableados con
  la fase opuesta respecto a 1 y 3 — sin esta corrección, el signo crudo del
  conteo no coincidiría con "la dirección en que `moveForward()` mueve esa
  rueda" para esos dos motores. Con la corrección, **las 4 ruedas usan la
  misma convención de signo**, lo que la cinemática de `odometry.cpp` asume.
- Motores 2/3 (`M2`/`M3` en `pins.h`) usan GPIOs input-only (34/35/36/39): no
  tienen pull-up interno, necesitan resistencias externas de 10k a 3.3V.

## Arranque suave (`setMotorSpeedRamped`/`update`)

Arrancar las 4 ruedas de golpe desde 0 duty causó un apagón completo del
rover (pico de corriente del LiPo compartido). La rampa de 300ms reduce (pero
no elimina del todo) ese pico — ver la nota en la memoria del repo si vuelve
a pasar, puede necesitar un capacitor de bulk o revisar el rating de la LiPo.
