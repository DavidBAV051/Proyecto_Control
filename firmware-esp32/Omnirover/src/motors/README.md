# Motores (4x DRV8871 + encoders de cuadratura)

Controla las 4 ruedas mecanum (driver DRV8871 por rueda, PWM vía LEDC) y lee
sus encoders de cuadratura para telemetría/odometría.

Archivos: [`motors.h`](motors.h), [`motors.cpp`](motors.cpp)

Índice de motor usado en **todo** el firmware (`main.cpp`, `odometry.cpp`,
dashboard): `0` = atrás-derecha, `1` = atrás-izquierda, `2` = adelante-derecha,
`3` = adelante-izquierda.

## API

- `init()`: configura los 8 canales LEDC (2 por motor) y los encoders.
- `setMotorSpeed(i, speed)`: aplica duty inmediatamente, `speed` en [-255,255].
- `setMotorSpeedRamped(i, target)` + `update()`: igual, pero sube el duty
  gradualmente (`RAMP_TIME_MS=300`) en vez de un salto instantáneo. `update()`
  debe llamarse cada ciclo de `loop()`.
- `stopMotor(i)` / `stopAll()`: paran de inmediato (sin rampa) y resetean el
  estado de rampa.
- `updateEncoderTelemetry()`: calcula RPM real por rueda y pose de odometría;
  llamarlo periódicamente (lo hace `web_server.cpp`'s `telemetryTask`).

## Por qué un duty "cap" por motor (`motorMaxDuty`)

Las 4 ruedas no son del mismo motor: M1(índice 1) fue reemplazado por uno de
140 RPM nominales (los otros 3 son de 150). `motorMaxDuty[m]` escala el 100%
de cada motor para que todos topen en `FLEET_RPM_CAP=140` RPM parejo, en vez
de que M1 corra más rápido que el resto al mismo comando. Ver
[el readme de odometría](../odometry/README.md) para el duty real resultante.

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
