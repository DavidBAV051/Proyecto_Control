# IMU (BNO08x)

Lee el IMU BNO08x (acelerómetro, giroscopio y un quaternion de orientación)
por I2C y expone un `IMU_Data` simple al resto del firmware, principalmente
para el yaw (rumbo) que usa la odometría (`src/odometry/`).

Archivos: [`imu.h`](imu.h), [`imu.cpp`](imu.cpp)

## API

- `init_IMU()`: arranca el bus I2C (400kHz) y habilita los reportes del
  sensor. Devuelve `false` si no detecta el chip (dirección `0x4A`) —
  `main.cpp` se queda bloqueado en un loop infinito si esto falla, a propósito,
  para no mover motores sin IMU.
- `update_IMU()`: debe llamarse cada ciclo de `loop()`; drena los eventos
  pendientes del sensor y actualiza accel/gyro/yaw.
- `get_IMU_Data()` / `print_IMU_Data()`: lectura y debug por Serial.

## Yaw: de quaternion a grados, con cero propio

`SH2_GAME_ROTATION_VECTOR` da orientación fusionando gyro+accel pero **sin
magnetómetro**, así que no tiene una referencia absoluta — su "cero" es la
orientación del chip al bootear/resetear, que es arbitraria. Este firmware:

1. Extrae yaw del quaternion con `atan2` (`quaternionToYawDeg()`), **negado**:
   el quaternion del BNO08x sigue la regla de la mano derecha sobre +Z
   (antihorario-positivo), opuesto a la convención de brújula que usa este
   firmware — sin la negación, girar en sentido horario se reportaba como
   grados negativos (bug corregido 2026-10-27).
2. Captura la primera lectura después de boot/reset como offset
   (`yawOffsetDeg`) y reporta todo relativo a ella, para que "yaw=0" siempre
   signifique "la orientación que tenía el chasis cuando arrancó el firmware".
3. Si el BNO08x se resetea solo (`wasReset()`), se vuelve a capturar un offset
   nuevo — el chip pierde su propia referencia en cada reset.

**Convención de yaw** (coincide con el dashboard y la odometría): 0° =
adelante, positivo = horario (como un rumbo de brújula).

**Pendiente**: nada garantiza todavía que "yaw=0 tras boot" sea exactamente
"chasis apuntando hacia adelante" — falta calibrar/taring con el chasis en
una orientación conocida, o un offset de montaje fijo.

## Acelerómetro: filtrado pero no usado para posición

`accelX/Y/Z` se suavizan con un EMA (`ACCEL_EMA_ALPHA=0.2`) porque el reporte
calibrado sigue siendo ruidoso con vibración. **Se probó usar esto para
dead-reckoning de movimiento lateral/adelante y se abandonó** — a las
magnitudes de movimiento de este rover el ruido era demasiado grande para
distinguir confiablemente un strafe. Hoy solo se leen/imprimen, sin consumidor
real; la posición usa encoders + yaw (`src/odometry/`), no el acelerómetro.

## Hardware: wiring crítico

Ver `include/pins.h` para el mapeo de pines y advertencias de wiring
(RST es activo-bajo — a GND deja el chip en reset permanente; CS/RST deben ir
a 3.3V regulado, nunca a 5V, o se puede freír el chip).
