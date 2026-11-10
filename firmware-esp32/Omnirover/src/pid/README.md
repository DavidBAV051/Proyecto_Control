# PID (lazo de velocidad por rueda)

Controlador PID discreto genérico, usado por `MotorController` para cerrar el
lazo de velocidad angular de cada rueda (setpoint y medición en rad/s, salida
en rad/s "equivalentes" que se mapean a PWM, ver [motores](../motors/README.md)).

Archivos: [`pid.h`](pid.h), [`pid.cpp`](pid.cpp)

## Ganancias (diseñadas en MATLAB)

| Motor | Nominal | Kp | Ki   | Kd    |
|-------|---------|----|------|-------|
| M0, M2, M3 | 150 RPM | 0 | 3.00 | 0.005 |
| M1         | 140 RPM | 0 | 3.00 (MATLAB: 4.35) | 0.005 |

Viven en `PID_KP/KI/KD[]` en `motors.h`. Son ganancias de tiempo continuo
(la simulación usó Ts=1e-4 s, efectivamente continua); el firmware las
discretiza con Euler hacia adelante a `PID_PERIOD_MS=20` ms. No se puede
correr a 10 kHz en el ESP32 porque la velocidad se mide contando pulsos de
encoder (~1000 cuentas/rev): a 7.2 rad/s hay ~23 cuentas por muestra de 20 ms,
suficiente para una medición útil; a Ts más corto la cuantización domina.

**Ki de M1 bajado a 3.0 (2026-11-02):** con Kp=0 el controlador es un
integrador puro, así que la pendiente del integrador (`Ki·error`) determina
cuánto tarda cada rueda en llegar al duty de arranque (fricción estática).
Con Ki=4.35 M1 arrancaba notablemente antes que las otras tres. Igualar Ki
sincroniza el arranque; el costo es que M1 (motor más lento) converge un poco
más despacio de lo que predecía MATLAB.

## Decisiones de implementación

- **Derivada sobre la medición** (no sobre el error): evita el "kick" del
  término D cuando el setpoint cambia de golpe (ej. forward -> slideRight).
- **Anti-windup por saturación del integrador**: el integrador se recorta al
  rango de salida `[-7.2, 7.2]` rad/s, igual que la salida. Con Kp=0 el
  controlador es esencialmente un integrador puro + D, así que sin esto una
  rueda bloqueada acumularía error indefinidamente.
- `reset()` borra el integrador y el estado de la derivada; `MotorController`
  lo llama en cada `stop` para que el siguiente arranque no herede un
  integrador viejo.

## Retro de rumbo con la IMU (temporal, solo para probar el PID)

> **Esto NO es parte del diseño final.** Es un andamio para poder validar el
> lazo de velocidad en el piso sin que el rover se vaya chueco. En la
> arquitectura definitiva, el **lazo de nivel superior** (retroalimentación de
> error de pose) será el que mantenga constantes la posición X, Y y el ángulo,
> y generará los setpoints de velocidad de rueda. Cuando ese lazo exista, esta
> corrección de rumbo en `main.cpp` se elimina.

### Por qué hizo falta

En la primera prueba en piso las 4 ruedas marcaban 68–70 RPM (el lazo de
velocidad convergía), pero el rover giraba lentamente a la derecha. Es decir:
en "espacio de encoder" las ruedas iban parejas, pero físicamente no. Causas
probables: `ENCODER_COUNTS_PER_REV` con ~4.7% de dispersión entre ruedas
(1047 vs 1000), diferencias de llanta/rodillos y slip mecanum. El PID por
rueda no puede ver nada de eso — solo ve su propio encoder.

### Qué hace

Implementado en `main.cpp` (`headingHoldCorrection()` / `driveStraight()`),
reutilizando esta misma clase `PID`:

- Al **iniciar cada tramo recto** (forward, backward, slideRight, slideLeft)
  captura el yaw actual de la IMU como objetivo. Es relativo: da igual si el
  rover arranca a 0°, ±90° o ±180°, mantiene *ese* rumbo. `releaseHeadingHold()`
  libera el objetivo en cada stop / cambio de paso de la trayectoria cuadrada.
- Cada ciclo calcula el error de yaw (envuelto a ±180°) y lo pasa a un PI:
  `HEADING_KP = 0.08` rad/s por grado, `HEADING_KI = 0.04` rad/s por grado·s,
  salida saturada a ±1.5 rad/s. Ese valor `c` se **resta a las ruedas 0/2 y se
  suma a las 1/3**, encima del setpoint del tramo. El lazo de velocidad por
  rueda sigue siendo el mismo; solo cambia su referencia.
- `moveCircle()` no pasa por aquí (el giro es intencional).

### Lecciones del ajuste en piso (2026-11-02)

1. **Signo:** la primera versión sumaba `c` a 0/2. El rover giró *más* a la
   derecha y se estabilizó exactamente a 180° del objetivo — firma clásica de
   realimentación positiva con error envuelto (el punto estable queda en ±180°).
   Se confirmó a mano que el yaw de la IMU sí es horario-positivo, así que el
   signo se invirtió en `driveStraight()`, no en `imu.cpp`.
2. **P solo deja error permanente:** con solo `Kp`, compensar una asimetría
   constante exige un error constante → el rover quedaba apuntando unos grados
   a la derecha. El término `Ki` lo elimina.
3. El integrador se resetea al inicio de cada tramo para que un segmento no
   herede la corrección del anterior.

### Límites conocidos

- Solo corrige **rotación** (yaw). Si el yaw se mantiene en ~0° pero el rover
  se desplaza lateralmente (crabbing mecanum), este lazo no lo ve. Eso es
  justo lo que resolverá el lazo superior sobre X/Y de la odometría.
- Con el setpoint en el tope (7.2 rad/s), el lado que recibe `+c` se satura y
  solo el otro lado frena: la autoridad de corrección se reduce a la mitad.
