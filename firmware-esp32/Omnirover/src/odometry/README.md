# Odometría del rover (pulsos de encoder -> RPM -> posición real)

Este documento explica, paso a paso, cómo el firmware convierte los pulsos crudos
de los encoders de cuadratura en una posición `(x, y)` en metros, world-frame,
usada por el dashboard (card "Rover Orientation & Path").

Código relevante:
- Decodificación de encoders y cálculo de RPM: [`src/motors/motors.cpp`](../motors/motors.cpp)
- Cinemática mecanum + integración de pose: [`src/odometry/odometry.h`](odometry.h) / [`src/odometry/odometry.cpp`](odometry.cpp)
- Transporte de la pose al dashboard: [`src/telemetry/telemetry.h`](../telemetry/telemetry.h) / [`telemetry.cpp`](../telemetry/telemetry.cpp)
- Dibujo del trail en la GUI: [`src/web_server/web_dashboard_html.h`](../web_server/web_dashboard_html.h)

## 1. Pulsos de encoder -> conteo (quadrature decoding)

Cada motor tiene un encoder de cuadratura (2 canales A/B). Cada flanco (`CHANGE`)
en A o B dispara `onEncoderChange()`, que mira el par de bits `(A<<1 | B)` antes y
después del flanco y usa una tabla de transición (`QUAD_TABLE`) para decidir si el
conteo avanza `+1`, `-1`, o se ignora (glitch/flanco inválido):

```cpp
uint8_t transition = (encoderPrevState[m] << 2) | state;
encoderCount[m] += QUAD_TABLE[transition];
```

Esto da un conteo acumulado de pulsos por rueda, con signo, en una ISR — no
depende de que el loop() lo lea a tiempo.

## 2. Conteo -> RPM

`MotorController::updateEncoderTelemetry()` corre periódicamente (desde el loop
principal) y por cada rueda calcula el delta de pulsos desde la última lectura,
lo normaliza con `ENCODER_DIR[m]` (compensa ruedas con las fases A/B invertidas
físicamente) y lo convierte a RPM con el tiempo transcurrido real (`elapsedMs`):

```cpp
long delta = rawDelta * ENCODER_DIR[m];
enc.velocityRpm = (delta / ENCODER_COUNTS_PER_REV[m]) * (60000.0f / elapsedMs);
```

`ENCODER_COUNTS_PER_REV[m]` es distinto por motor porque fueron medidos
individualmente (M1 es el motor de 140 RPM que reemplazó al original). Gracias a
`ENCODER_DIR`, el signo de `velocityRpm` siempre significa "la dirección en que
`moveForward()` comandaría esa rueda", igual para las 4 ruedas — por eso se puede
usar directamente en la cinemática del paso 4 sin casos especiales por rueda.

## 3. RPM -> velocidad lineal de la rueda

Con el radio de rueda medido $R_w = 0.04\ m$ (diámetro 80 mm):

$$v_i = \frac{RPM_i}{60} \cdot 2\pi R_w \quad \text{[m/s]}$$

(`odometry_update()`, un `for` sobre las 4 ruedas).

## 4. Velocidad de rueda -> velocidad del cuerpo del robot (cinemática mecanum)

Fórmula estándar para un robot de 4 ruedas mecanum en configuración X (modelo
MATLAB de control, $\omega_1..\omega_4$ = velocidad angular de cada rueda,
$L$ = semi-distancia eje Y, $l$ = semi-distancia eje X):

$$v_{xr} = \frac{R_w}{4}(\omega_1+\omega_2+\omega_3+\omega_4)$$
$$v_{yr} = \frac{R_w}{4}(-\omega_1+\omega_2+\omega_3-\omega_4)$$
$$\omega_{zr} = \frac{R_w}{4(L+l)}(-\omega_1+\omega_2-\omega_3+\omega_4)$$

**Mapeo a los índices de motor de este firmware** (`main.cpp`: 0=atrás-derecha,
1=atrás-izquierda, 2=adelante-derecha, 3=adelante-izquierda) corresponde a
$\omega_4=$ motor 0, $\omega_3=$ motor 1, $\omega_2=$ motor 2, $\omega_1=$ motor 3
en la convención académica de arriba (FL=1, FR=2, RL=3, RR=4). Sustituyendo:

```cpp
float vxBody = (v[0] + v[1] + v[2] + v[3]) / 4.0f;        // == Rw/4 * sum(wi)
float vyBody = (v[0] - v[1] - v[2] + v[3]) / 4.0f;        // + = derecha
```

`vxBody` es idéntica a $v_{xr}$. `vyBody` es el **negativo** de $v_{yr}$: la
fórmula académica usa la convención ROS (x=adelante, y=izquierda), mientras que
aquí se definió `vyBody` como positivo hacia la **derecha** (para que coincida
con el nombre del comando `slideRight()` sin tener que acordarse de un signo
invertido al leerlo). Es solo una elección de convención, no un error — está
documentado en el código (`odometry.cpp`) y validado con la prueba del cuadrado
(sección 7).

$\omega_{zr}$ (yaw-rate derivado de ruedas) **no se usa** — el rumbo viene
directo del IMU (sección 5), que es más confiable que integrar ruido de 4
encoders para obtener orientación. Por eso `L` y `l` están declaradas en
`odometry.cpp` pero no entran todavía en ninguna cuenta; quedan ahí para si en
el futuro se quiere una estimación de yaw redundante basada en ruedas.

## 5. Velocidad del cuerpo -> velocidad en el mundo (rotación por yaw)

Forma estándar (ángulo $\theta$ medido antihorario desde el eje X del mundo,
"adelante" del robot alineado con +X cuando $\theta=0$):

$$\begin{bmatrix}\dot x\\ \dot y\\ \dot\theta\end{bmatrix} = R^T(\theta)\begin{bmatrix}v_{xr}\\ v_{yr}\\ \omega_{zr}\end{bmatrix}, \qquad R^T(\theta)=\begin{bmatrix}\cos\theta & -\sin\theta & 0\\ \sin\theta & \cos\theta & 0\\ 0 & 0 & 1\end{bmatrix}$$

Este firmware usa una convención distinta para el ángulo (igual que el IMU y el
dashboard): $yaw=0$ = "adelante" apunta a +Y del mundo (arriba en pantalla, estilo
brújula), y $yaw$ positivo es **horario**, no antihorario. Es la misma rotación
física, solo con los ejes y el sentido de giro re-etiquetados; al re-derivar con
esa convención (ver `odometry.cpp`):

```cpp
float yawRad = yawDeg * DEG_TO_RAD;
float sinY = sinf(yawRad), cosY = cosf(yawRad);
g_pose.x += (vxBody * sinY + vyBody * cosY) * dt;
g_pose.y += (vxBody * cosY - vyBody * sinY) * dt;
```

`yawDeg` es `imu.yaw` (BNO08x, quaternion -> yaw), no un rumbo derivado de las
ruedas.

## 6. Integración -> pose acumulada

Integración de Euler simple con el mismo `elapsedMs` que ya se usó para el RPM
(mismo ciclo de `updateEncoderTelemetry()`, típicamente cada ciclo del loop):

$$x \mathrel{+}= \dot x \cdot dt, \qquad y \mathrel{+}= \dot y \cdot dt, \qquad dt = elapsedMs/1000$$

El resultado (`OdometryPose{x,y}`) se guarda como estado global en
`odometry.cpp` y se publica a `telemetry_setPose()` cada vez que se actualiza,
para que el dashboard solo tenga que graficar los metros que ya vienen
calculados (ya no recalcula nada en JS).

## 7. Constantes físicas usadas

| Constante | Valor | De dónde sale | Dónde vive |
|---|---|---|---|
| $R_w$ (radio de rueda) | 0.04 m (diámetro 80 mm) | medido / modelo MATLAB | `odometry.cpp: WHEEL_RADIUS_M` |
| $L$ (semi-distancia eje Y) | 0.08 m | modelo MATLAB | `odometry.cpp: HALF_WHEELBASE_M` |
| $l$ (semi-distancia eje X) | 0.105 m (incluye acoples) | modelo MATLAB | `odometry.cpp: HALF_TRACK_M` |
| Cuentas de encoder / rev | {1047, 2046, 1028, 1000} por motor | medido con cada motor real | `motors.h: ENCODER_COUNTS_PER_REV` |

## 8. Validación (2026-10-27)

Se corrió `TRAJ_SQUARE` (adelante -> strafe derecha -> atrás -> strafe
izquierda, en loop) **con el rover elevado (ruedas sin tocar el piso)**. El
trail en el dashboard trazó un cuadrado y volvió a `(-0.01, 0.0)` m, muy cerca
del origen — confirma que el signo de `vxBody`/`vyBody` y el mapeo de índices de
motor son correctos. El pequeño error residual es esperado: todavía no hay PID,
así que cada rueda no gira exactamente al mismo RPM entre segmentos.

**Importante**: esa prueba solo valida la matemática/consistencia de encoders,
no el comportamiento real en piso. Con las ruedas flotando no hubo
deslizamiento lateral (típico de ruedas mecanum) ni fricción/carga real — una
prueba con el rover apoyado en el piso casi seguro mostrará más deriva.

## 9. Limitaciones conocidas

- **Sin compensación de deslizamiento**: si una rueda patina (especialmente
  lateral, común en mecanum), la odometría no tiene forma de detectarlo — es
  dead reckoning puro.
- **Sin PID todavía**: la posición no depende de que el RPM comandado sea
  exacto (se usa el RPM *medido*), pero sin PID cada rueda puede tener un sesgo
  distinto de velocidad real entre segmentos de una trayectoria, lo que se
  traduce en error acumulado.
- **Integración de Euler**: asume velocidad constante durante cada intervalo
  `elapsedMs`; a mayor frecuencia de muestreo, menor error de este tipo.
- **No hay corrección de deriva**: no hay todavía una referencia absoluta
  (ej. cámara/HuskyLens) que resetee la posición acumulada.
