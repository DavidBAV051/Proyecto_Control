#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision_tracker.py
=================

Módulo de visión: cámara + AprilTag tag25h9 (cv2.aruco nativo) + seguimiento,
para Raspberry Pi 4 (aarch64, Python 3.13) con Logitech Brio 100 a 1080p30.
Lo consume follow_controller.py, pero también se puede ejecutar solo para
probar la visión:  python3 vision_tracker.py

IDs (tag25h9):
    1 -> TRACK  "DAVID"
    2 -> TRACK  "CUAUHTEMOC"
    3 -> STOP   prioridad global: se busca en todo el frame aunque haya lock.

Reglas de objetivo:
    * Sin objetivo (SEARCH) se fija a la persona más cercana (tag más grande).
    * Con objetivo fijado se ignora a la otra persona hasta recibir STOP o
      perder al objetivo por timeout.

API pública
-----------
    from vision_tracker import VisionTracker, Config

    with VisionTracker(Config(show_gui=True)) as vision:
        while True:
            out = vision.read(timeout=0.1)       # VisionOutput o None (sin frame nuevo)
            if out is None:
                continue
            out.is_tracking    # bool: TRACK u OCCLUDED
            out.target_name    # "DAVID" | "CUAUHTEMOC" | "NONE"
            out.tag_x, out.tag_y   # centro del tag en pantalla [px, 1920x1080]; None sin objetivo
            out.width_px       # ancho del tag [px]; 0 sin objetivo
            out.frame          # frame BGR a color (resolución completa, solo lectura)
            out.state          # "TRACK" | "OCCLUDED" | "SEARCH" | "STOP"
            key = vision.show(out, extra_lines=["texto del controlador"])  # GUI (throttled)

    state  "TRACK"    coordenadas medidas sobre el AprilTag
           "OCCLUDED" tag oculto; coordenadas del tracker corporal (o de la
                      predicción de Kalman). width_px = último ancho medido.
           "SEARCH"   sin objetivo
           "STOP"     ID 3 visible (se mantiene STOP_HOLD_S tras dejar de verlo)

Color y escala de grises
------------------------
El frame se decodifica SIEMPRE a color (BGR) y la GUI se muestra a color. La
conversión a gris solo se hace internamente sobre lo que entra al detector:
el recorte ROI de 1080p (LOCKED) y la vista reducida 640x360 del escaneo
global; el tracker corporal usa su propia vista gris de 320x180.

Rendimiento a 30 FPS
--------------------
1. Exposición manual corta (auto_exposure=1, exposure_time_absolute) y
   exposure_dynamic_framerate=0: con exposición automática la Brio 100 baja a
   15 FPS con poca luz; además se reduce el motion blur.
2. 4 buffers V4L2 + JPEG crudo (CAP_PROP_CONVERT_RGB=0, solo memcpy) +
   decodificación en DECODE_THREADS hilos (cv2.imdecode libera el GIL). Con
   BUFFERSIZE=1 el backend V4L2 de OpenCV pierde un frame de cada dos porque
   devuelve el buffer al driver solo después de decodificar.
3. La ventana se dibuja cada GUI_EVERY_N frames sobre la vista 640x360; el
   tracking sigue a 30 Hz.

Hilos
-----
    captura         abre, lee y CIERRA la cámara (finally)  -> JPEG crudo
    decodificador×N JPEG -> frame BGR más reciente
    llamador        VisionTracker.read() -> TargetTracker -> VisionOutput

Pipeline de seguimiento (TargetTracker)
---------------------------------------
    SEARCH   escaneo 640x360 de todo el frame (todos los IDs).
    LOCKED   Kalman (velocidad constante) predice la posición; se detecta en
             un ROI de 1080p centrado en la predicción cuyo tamaño crece con la
             incertidumbre del filtro y con cada frame perdido. El recorte se
             reduce para que el tag mida ~ROI_TAG_PX (más rápido y tolera más
             motion blur) y, si falla, se reintenta a ~ROI_RETRY_TAG_PX.
             Cada STOP_SCAN_EVERY frames: escaneo global 640x360 (STOP y
             re-enganche del objetivo).
             Tras ROI_MISSES_FALLBACK fallos: 1 frame de escaneo global antes
             de soltar el objetivo.
    OCCLUDED (ENABLE_PERSON_FALLBACK) tracker ligero sobre 320x180
             (plantilla NCC por defecto; KCF/MOSSE opcionales) inicializado en
             el torso bajo el tag, que busca alrededor de la predicción de
             Kalman y tolera BODY_MAX_FAILS fallos seguidos (oclusión parcial).
             Da (tag_x, tag_y) hasta OCCLUSION_TIMEOUT_S y centra el ROI para
             re-enganchar el tag automáticamente. Solo sigue a ESA persona.
    STOP     ID 3 visto en cualquier parte: suelta el objetivo; se mantiene
             hasta STOP_HOLD_S después de dejar de verlo.

Instalación
-----------
    sudo apt install python3-opencv v4l-utils   # Trixie/Py 3.13: OpenCV 4.10 + contrib

Arranque autónomo (systemd, sin monitor)
----------------------------------------
    SAOAH_HEADLESS=1 o --no-gui: nunca se abre una ventana de OpenCV (también
    se desactiva sola si no hay DISPLAY/WAYLAND_DISPLAY o si falla la ventana).
    --device auto: /dev/v4l/by-id/*-video-index0 (nombre USB estable) y, si no
    existe, el índice 0. Al arrancar se reintenta abrir la cámara durante
    CAMERA_OPEN_RETRY_S (USB enumerando); con el stream ya activo se fija
    MJPG 1920x1080 @ 30 FPS y la exposición manual, y se verifica con v4l2-ctl.

Uso (solo visión)
-----------------
    python3 vision_tracker.py
    python3 vision_tracker.py --no-gui --print-interval 0      # cada frame
    python3 vision_tracker.py --exposure 60 --gain 120         # menos blur
Salir: 'q' / ESC en la ventana o Ctrl+C.
"""
from __future__ import annotations

import os

# Tiempo máximo de bloqueo de read() en V4L2 (por defecto 10 s). Debe fijarse
# antes del primer grab para que un cierre/desconexión no congele el hilo.
os.environ.setdefault("OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT", "2")

import argparse
import glob
import logging
import math
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, NamedTuple, Optional, Sequence

import cv2
import numpy as np

# =============================================================================
# CONFIGURACIÓN
# =============================================================================
# --- Cámara (Logitech Brio 100) ---
CAMERA_DEVICE: int | str = "auto"   # "auto" (by-id USB, si no índice 0), índice o ruta
CAMERA_BY_ID_GLOB = "/dev/v4l/by-id/*-video-index0"   # nodo de captura de cada cámara USB
CAMERA_OPEN_RETRY_S = 10.0          # al arrancar: reintentar abrir la cámara este tiempo [s]
CAMERA_RETRY_INTERVAL_S = 1.0       # pausa entre intentos [s]
CAMERA_APPLY_ATTEMPTS = 3           # aplicar + verificar exposición (v4l2-ctl) hasta N veces
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080
CAMERA_FPS = 30
CAPTURE_BUFFERS = 4                 # NO usar 1 (ver docstring): 4 = sin pérdidas, latencia ~1 frame
RAW_MJPEG = True                    # JPEG crudo + decodificación en hilos propios
DECODE_THREADS = 3                  # hilos de cv2.imdecode a color (RPi 4: 4 núcleos)
MANUAL_EXPOSURE = True              # auto_exposure = 1 (manual)
EXPOSURE_VALUE = 83                 # unidades de 100 µs -> 8.3 ms: poco blur y sin parpadeo a 60 Hz
GAIN_VALUE: Optional[int] = None    # 0..255; súbelo si la imagen queda oscura con exposición corta
POWER_LINE_FREQ = 2                 # 0 = off, 1 = 50 Hz, 2 = 60 Hz (México)
READ_FAIL_LIMIT = 30                # lecturas fallidas seguidas antes de reabrir la cámara
CV_THREADS: Optional[int] = None    # hilos internos de OpenCV (None = por defecto)

# --- IDs AprilTag ---
PEOPLE: dict[int, str] = {1: "DAVID", 2: "CUAUHTEMOC"}
STOP_ID = 3
STOP_HOLD_S = 1.0                   # STOP se mantiene este tiempo tras dejar de ver el ID 3

# --- Búsqueda global / ROI dinámico ---
SEARCH_WIDTH = 640                  # escaneo global 640x360
STOP_SCAN_EVERY = 3                 # con lock: escaneo global cada N frames (STOP <= 100 ms)
ROI_SCALE = 1.5                     # semilado base del ROI = ROI_SCALE * lado del tag
ROI_MIN_HALF = 80                   # semilado mínimo [px a 1080p]
ROI_GROWTH = 1.6                    # el ROI crece este factor por cada frame perdido
ROI_SIGMA_K = 3.0                   # + k * incertidumbre de posición del Kalman
ROI_MAX_DIM = 640                   # ROI mayor -> se reduce antes de detectar
ROI_TAG_PX = 60                     # tag mayor -> se reduce el ROI (más rápido y tolera más blur)
ROI_RETRY_TAG_PX = 35               # si falla, 2º intento en el mismo ROI con el tag a ~35 px (blur)
ROI_RETRY_MAX_DIM = 400             # ... solo si el ROI ya reducido es pequeño (coste acotado)
ROI_MISSES_FALLBACK = 4             # fallos seguidos en ROI -> 1 frame de escaneo global
RESCAN_EVERY = 3                    # durante oclusión: escaneo global cada N frames
SUBPIXEL_REFINE = True              # esquinas subpíxel en el ROI

# --- Filtro de Kalman (velocidad constante, px a 1080p) ---
KF_ACCEL_STD = 4000.0               # aceleración esperada [px/s²]: mayor = sigue giros más bruscos
KF_MEAS_STD_TAG = 1.5               # ruido de medida del tag en ROI [px]
KF_MEAS_STD_SCAN = 4.0              # ruido de medida del escaneo 640x360 [px]
KF_MEAS_STD_BODY = 12.0             # ruido de medida del tracker corporal [px]
KF_MAX_SPEED = 4000.0               # límite de velocidad estimada [px/s]

# --- Fallback corporal (tag oculto) ---
ENABLE_PERSON_FALLBACK = True
OCCLUSION_TIMEOUT_S = 2.5
BODY_TRACKER = "TEMPLATE"           # "TEMPLATE": NCC centrada en la predicción de Kalman (recomendado:
                                    # el más preciso en movimientos bruscos y sin contrib)
                                    # "KCF" | "MOSSE": opencv-contrib; buscan donde se quedaron -> derivan
TRACK_WIDTH = 320                   # resolución del tracker corporal: 320x180
BODY_BOX = (2.4, 2.4, 0.55)         # torso DEBAJO del tag: ancho, alto y bajada del borde superior
                                    # respecto al centro del tag (todo en lados del tag)
BODY_MIN_PX = 16                    # tamaño mínimo de la caja corporal a 320x180
BODY_MAX_JUMP = 0.5                 # salto máximo por frame (fracción de la caja) -> anti-cambio de persona
BODY_MAX_FAILS = 6                  # fallos seguidos del tracker corporal tolerados (oclusión parcial):
                                    # mientras tanto se usa la predicción de Kalman
TEMPLATE_MIN_SCORE = 0.45           # NCC mínima del tracker por plantilla

# --- Salida ---
HEADLESS_ENV = "SAOAH_HEADLESS"     # =1 en el servicio systemd: sin GUI por defecto
SHOW_GUI = os.environ.get(HEADLESS_ENV, "0").strip().lower() not in ("1", "true", "yes", "on")
GUI_WIDTH = 640
GUI_EVERY_N = 3                     # renderizar 1 de cada N frames (30 FPS -> GUI a 10 Hz)
PRINT_INTERVAL_S = 0.5              # modo consola; 0 = imprimir cada frame

# Estados de salida
TRACK, OCCLUDED, SEARCH, STOP = "TRACK", "OCCLUDED", "SEARCH", "STOP"

LOG = logging.getLogger("vision_tracker")


@dataclass(slots=True)
class Config:
    # cámara
    device: int | str = CAMERA_DEVICE
    open_retry_s: float = CAMERA_OPEN_RETRY_S
    width: int = FRAME_WIDTH
    height: int = FRAME_HEIGHT
    fps: int = CAMERA_FPS
    capture_buffers: int = CAPTURE_BUFFERS
    raw_mjpeg: bool = RAW_MJPEG
    decode_threads: int = DECODE_THREADS
    manual_exposure: bool = MANUAL_EXPOSURE
    exposure: int = EXPOSURE_VALUE
    gain: Optional[int] = GAIN_VALUE
    power_line_freq: int = POWER_LINE_FREQ
    # ids
    people: dict[int, str] = field(default_factory=lambda: dict(PEOPLE))
    stop_id: int = STOP_ID
    stop_hold_s: float = STOP_HOLD_S
    # búsqueda / roi
    search_width: int = SEARCH_WIDTH
    stop_scan_every: int = STOP_SCAN_EVERY
    roi_scale: float = ROI_SCALE
    roi_min_half: int = ROI_MIN_HALF
    roi_growth: float = ROI_GROWTH
    roi_sigma_k: float = ROI_SIGMA_K
    roi_max_dim: int = ROI_MAX_DIM
    roi_tag_px: int = ROI_TAG_PX
    roi_retry_tag_px: int = ROI_RETRY_TAG_PX
    roi_retry_max_dim: int = ROI_RETRY_MAX_DIM
    roi_misses_fallback: int = ROI_MISSES_FALLBACK
    rescan_every: int = RESCAN_EVERY
    subpixel_refine: bool = SUBPIXEL_REFINE
    # kalman
    kf_accel_std: float = KF_ACCEL_STD
    kf_meas_std_tag: float = KF_MEAS_STD_TAG
    kf_meas_std_scan: float = KF_MEAS_STD_SCAN
    kf_meas_std_body: float = KF_MEAS_STD_BODY
    kf_max_speed: float = KF_MAX_SPEED
    # fallback corporal
    enable_person_fallback: bool = ENABLE_PERSON_FALLBACK
    occlusion_timeout_s: float = OCCLUSION_TIMEOUT_S
    body_tracker: str = BODY_TRACKER
    track_width: int = TRACK_WIDTH
    body_box: tuple[float, float, float] = BODY_BOX
    body_min_px: int = BODY_MIN_PX
    body_max_jump: float = BODY_MAX_JUMP
    body_max_fails: int = BODY_MAX_FAILS
    template_min_score: float = TEMPLATE_MIN_SCORE
    # salida
    show_gui: bool = SHOW_GUI
    gui_width: int = GUI_WIDTH
    gui_every_n: int = GUI_EVERY_N
    print_interval: float = PRINT_INTERVAL_S


# =============================================================================
# Tipos de datos
# =============================================================================
class Geometry(NamedTuple):
    cx: float
    cy: float
    width: float
    height: float


class Detection(NamedTuple):
    tag_id: int
    corners: np.ndarray        # (4, 2) a resolución completa
    geo: Geometry


class BodyEstimate(NamedTuple):
    cx: float                  # posición estimada del TAG (no del torso) a 1080p
    cy: float
    box: tuple[float, float, float, float]   # caja corporal (x0, y0, x1, y1) a 1080p


@dataclass(slots=True)
class TrackResult:
    frame_id: int
    timestamp: float                       # time.perf_counter() de la captura
    state: str                             # TRACK | OCCLUDED | SEARCH | STOP
    person: Optional[str] = None           # "DAVID" | "CUAUHTEMOC"
    tag_id: Optional[int] = None
    source: Optional[str] = None           # TAG (ROI) | SCAN (640x360) | BODY | PREDICT
    cx: Optional[float] = None             # [px, resolución completa]
    cy: Optional[float] = None
    width_px: float = 0.0                  # ancho del tag; en OCCLUDED, último ancho medido
    ex: float = 0.0                        # [-1, 1]  + = derecha
    ey: float = 0.0                        # [-1, 1]  + = abajo
    corners: Optional[np.ndarray] = None
    roi: Optional[tuple[int, int, int, int]] = None
    body_box: Optional[tuple[float, float, float, float]] = None
    scanned: bool = False                  # hubo escaneo global en este frame
    fps: float = 0.0                       # FPS reales de procesamiento
    proc_ms: float = 0.0                   # tiempo de procesamiento del frame

    def as_tuple(self) -> tuple:
        """(person_name, state, cx, cy, width_px, ex, fps) para el controlador."""
        return (self.person, self.state, self.cx, self.cy, self.width_px, self.ex, self.fps)


# =============================================================================
# Utilidades
# =============================================================================
def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def to_gray(img: np.ndarray) -> np.ndarray:
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


class FPSMeter:
    """FPS sobre una ventana deslizante de marcas de tiempo."""

    def __init__(self, window: int = 30) -> None:
        self._t: deque[float] = deque(maxlen=window)

    def tick(self, t: Optional[float] = None) -> float:
        self._t.append(time.perf_counter() if t is None else t)
        n = len(self._t)
        dt = self._t[-1] - self._t[0] if n > 1 else 0.0
        return (n - 1) / dt if dt > 0 else 0.0


def tag_geometry(pts: np.ndarray) -> Geometry:
    """Centro = cruce de diagonales (exacto en perspectiva); ancho = media de bordes sup./inf."""
    (x0, y0), (x1, y1), (x2, y2), (x3, y3) = pts.tolist()
    d1x, d1y, d2x, d2y = x2 - x0, y2 - y0, x3 - x1, y3 - y1
    den = d1x * d2y - d1y * d2x
    if abs(den) > 1e-9:
        t = ((x1 - x0) * d2y - (y1 - y0) * d2x) / den
        cx, cy = x0 + t * d1x, y0 + t * d1y
    else:
        cx, cy = (x0 + x1 + x2 + x3) / 4.0, (y0 + y1 + y2 + y3) / 4.0
    width = 0.5 * (math.hypot(x1 - x0, y1 - y0) + math.hypot(x2 - x3, y2 - y3))
    height = 0.5 * (math.hypot(x3 - x0, y3 - y0) + math.hypot(x2 - x1, y2 - y1))
    return Geometry(cx, cy, width, height)


def to_full_res(pts: np.ndarray, sx: float, sy: float,
                ox: float = 0.0, oy: float = 0.0) -> np.ndarray:
    """Imagen reducida (sx, sy) y desplazada (ox, oy) -> 1080p, con el convenio de cv2.resize."""
    out = pts.astype(np.float64, copy=True)
    out[:, 0] = (out[:, 0] + 0.5) / sx - 0.5 + ox
    out[:, 1] = (out[:, 1] + 0.5) / sy - 0.5 + oy
    return out


# =============================================================================
# Vistas del frame (lazy: cada reducción se calcula una vez y solo si se usa)
# =============================================================================
class ViewSizes(NamedTuple):
    full: tuple[int, int]
    small: tuple[int, int]     # escaneo global / GUI   (640x360)
    tiny: tuple[int, int]      # tracker corporal       (320x180)

    @classmethod
    def build(cls, w: int, h: int, small_w: int, tiny_w: int) -> "ViewSizes":
        sw = min(small_w, w)
        tw = min(tiny_w, sw)
        return cls((w, h), (sw, max(1, round(h * sw / w))), (tw, max(1, round(h * tw / w))))


class FrameViews:
    __slots__ = ("full", "frame_id", "ts", "sizes", "_small", "_small_gray", "_tiny")

    def __init__(self, full: np.ndarray, frame_id: int, ts: float, sizes: ViewSizes) -> None:
        self.full, self.frame_id, self.ts, self.sizes = full, frame_id, ts, sizes
        self._small = self._small_gray = self._tiny = None

    @property
    def small(self) -> np.ndarray:
        if self._small is None:
            self._small = (self.full if self.sizes.small == self.sizes.full else
                           cv2.resize(self.full, self.sizes.small, interpolation=cv2.INTER_AREA))
        return self._small

    @property
    def small_gray(self) -> np.ndarray:
        if self._small_gray is None:
            self._small_gray = to_gray(self.small)
        return self._small_gray

    @property
    def tiny(self) -> np.ndarray:
        if self._tiny is None:
            self._tiny = (self.small if self.sizes.tiny == self.sizes.small else
                          cv2.resize(self.small, self.sizes.tiny, interpolation=cv2.INTER_AREA))
        return self._tiny

    def roi_gray(self, x0: int, y0: int, x1: int, y1: int) -> np.ndarray:
        """Recorte (vista numpy, sin copia) y conversión a gris SOLO del ROI."""
        return to_gray(self.full[y0:y1, x0:x1])


# =============================================================================
# Cámara: V4L2 + MJPEG crudo + decodificación paralela
# =============================================================================
def _dev_path(device: int | str) -> str:
    return f"/dev/video{device}" if isinstance(device, int) else str(device)


def resolve_camera_device(device: int | str) -> int | str:
    """
    "auto" -> primera /dev/v4l/by-id/*-video-index0 (o índice 0 si no hay).
    Los enlaces (by-id/by-path) se resuelven a /dev/videoN -> índice N. Se
    llama en cada apertura: si la cámara se reconecta con otro N, se sigue.
    """
    if isinstance(device, str) and device.strip().lower() == "auto":
        found = sorted(glob.glob(CAMERA_BY_ID_GLOB))
        if not found:
            return 0
        device = found[0]
    if isinstance(device, str):
        if device.isdigit():
            return int(device)
        if os.path.exists(device):
            device = os.path.realpath(device)
        m = re.fullmatch(r"/dev/video(\d+)", device)
        if m:
            return int(m.group(1))
    return device


def _v4l2_set(device: int | str, controls: list[tuple[str, int]]) -> None:
    """Aplica controles con v4l2-ctl en orden (los nombres inexistentes se ignoran)."""
    exe = shutil.which("v4l2-ctl")
    if exe is None:
        return
    for name, value in controls:
        try:
            subprocess.run([exe, "-d", _dev_path(device), "-c", f"{name}={value}"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=2.0, check=False)
        except (OSError, subprocess.SubprocessError):
            pass


def _v4l2_values(device: int | str) -> Optional[dict[str, int]]:
    """Valores actuales de todos los controles (v4l2-ctl --list-ctrls); None sin v4l2-ctl."""
    exe = shutil.which("v4l2-ctl")
    if exe is None:
        return None
    try:
        out = subprocess.run([exe, "-d", _dev_path(device), "--list-ctrls"],
                             capture_output=True, text=True, timeout=2.0, check=False).stdout
    except (OSError, subprocess.SubprocessError):
        return {}
    vals: dict[str, int] = {}
    for line in out.splitlines():
        m = re.match(r"\s*(\w+)\s+0x[0-9a-fA-F]+\s+\(\w+\)\s*:.*?\bvalue=(-?\d+)", line)
        if m:
            vals[m.group(1)] = int(m.group(2))
    return vals


def _v4l2_get(device: int | str, names: str) -> str:
    exe = shutil.which("v4l2-ctl")
    if exe is None:
        return "v4l2-ctl no instalado (sudo apt install v4l-utils)"
    try:
        out = subprocess.run([exe, "-d", _dev_path(device), f"--get-ctrl={names}"],
                             capture_output=True, text=True, timeout=2.0, check=False)
        return " ".join(out.stdout.split()) or out.stderr.strip()
    except (OSError, subprocess.SubprocessError) as e:
        return str(e)


def open_camera(cfg: Config, device: int | str | None = None) -> cv2.VideoCapture:
    """Abre la cámara. Orden en V4L2: FOURCC -> resolución -> FPS -> buffers."""
    dev = resolve_camera_device(cfg.device) if device is None else device
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"No se pudo abrir la cámara {cfg.device!r} ({_dev_path(dev)}) con V4L2")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
    cap.set(cv2.CAP_PROP_FPS, cfg.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, cfg.capture_buffers)
    return cap


def configure_exposure(cap: cv2.VideoCapture, cfg: Config,
                       device: int | str | None = None) -> None:
    """
    Exposición manual corta + FPS fijos. Se aplica por OpenCV y, si existe,
    por v4l2-ctl (nombres de controles nuevos y antiguos del kernel); luego se
    lee de vuelta y se reaplica hasta CAMERA_APPLY_ATTEMPTS veces si no quedó.
    """
    dev = resolve_camera_device(cfg.device) if device is None else device
    if cfg.manual_exposure:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)          # V4L2: 1 = manual, 3 = auto
        cap.set(cv2.CAP_PROP_EXPOSURE, cfg.exposure)
    else:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
    if cfg.gain is not None:
        cap.set(cv2.CAP_PROP_GAIN, cfg.gain)

    ctrls: list[tuple[str, int]] = [
        ("exposure_dynamic_framerate", 0), ("exposure_auto_priority", 0),  # sin FPS dinámicos
        ("power_line_frequency", cfg.power_line_freq),
    ]
    if cfg.manual_exposure:
        ctrls += [("auto_exposure", 1), ("exposure_auto", 1),              # primero el modo
                  ("exposure_time_absolute", cfg.exposure), ("exposure_absolute", cfg.exposure)]
    else:
        ctrls += [("auto_exposure", 3), ("exposure_auto", 3)]
    if cfg.gain is not None:
        ctrls.append(("gain", cfg.gain))
    wanted = dict(ctrls)
    for attempt in range(1, max(1, CAMERA_APPLY_ATTEMPTS) + 1):
        _v4l2_set(dev, ctrls)
        current = _v4l2_values(dev)
        if current is None:
            LOG.warning("v4l2-ctl no instalado: exposición solo por OpenCV "
                        "(sudo apt install v4l-utils para fijarla y verificarla)")
            break
        bad = {k: (v, current[k]) for k, v in wanted.items() if k in current and current[k] != v}
        if not bad:
            break
        if attempt == CAMERA_APPLY_ATTEMPTS:
            LOG.warning("V4L2: controles que no quedaron (pedido, actual): %s", bad)
        else:
            time.sleep(0.3)

    LOG.info("V4L2 %s: %s", _dev_path(dev), _v4l2_get(
        dev, "auto_exposure,exposure_time_absolute,exposure_dynamic_framerate,gain"))
    if cfg.manual_exposure and cfg.exposure * 100e-6 > 1.0 / cfg.fps:
        LOG.warning("Exposición %.1f ms > periodo de frame %.1f ms: los FPS caerán",
                    cfg.exposure / 10, 1000 / cfg.fps)


class CameraStream:
    """
    Hilo de captura: abre, lee y libera la cámara (finally) -> /dev/video* nunca
    queda bloqueado. En modo RAW_MJPEG publica el JPEG crudo y DECODE_THREADS
    hilos lo decodifican; siempre se procesa el frame más reciente (sin cola).
    Sin .copy(): read()/imdecode devuelven arrays nuevos; el consumidor no los
    modifica en sitio.
    """

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._frame_cond = threading.Condition()
        self._jpeg_cond = threading.Condition()
        self._frame: Optional[np.ndarray] = None
        self._frame_id = 0
        self._frame_ts = 0.0
        self._jpeg: Optional[np.ndarray] = None
        self._jpeg_id = 0
        self._jpeg_ts = 0.0
        self._jpeg_taken = 0
        self._error: Optional[str] = None
        self.raw_mode = False
        self.width = self.height = 0
        self.fps = 0.0                 # FPS reales que entrega la cámara
        self.decode_ms = 0.0           # tiempo medio de decodificación JPEG
        self._threads = [threading.Thread(target=self._capture_loop, name="capture", daemon=True)]
        self._threads += [threading.Thread(target=self._decode_loop, name=f"decode{i}", daemon=True)
                          for i in range(max(1, cfg.decode_threads))]

    # ---------------- API ----------------
    def start(self, timeout: Optional[float] = None) -> "CameraStream":
        if timeout is None:
            timeout = max(0.0, self.cfg.open_retry_s) + 8.0
        for t in self._threads:
            t.start()
        if not self._ready.wait(timeout) or self._error or self._frame is None:
            self.stop()
            raise RuntimeError(self._error or "La cámara no entregó frames a tiempo")
        return self

    def read(self, last_id: int, timeout: float = 1.0
             ) -> Optional[tuple[int, np.ndarray, float]]:
        """Espera un frame más nuevo que last_id -> (id, frame, ts) o None."""
        with self._frame_cond:
            self._frame_cond.wait_for(
                lambda: self._frame_id != last_id or self._stop.is_set(), timeout)
            if self._frame_id == last_id:
                return None
            return self._frame_id, self._frame, self._frame_ts

    @property
    def alive(self) -> bool:
        return self._threads[0].is_alive() and not self._stop.is_set()

    def stop(self) -> None:
        self._stop.set()
        for cond in (self._frame_cond, self._jpeg_cond):
            with cond:
                cond.notify_all()
        me = threading.current_thread()
        for t in self._threads:
            if t.is_alive() and t is not me:
                t.join(timeout=3.0)
                if t.is_alive():
                    LOG.warning("El hilo %s no terminó a tiempo", t.name)

    # ---------------- interno ----------------
    def _publish(self, frame: np.ndarray, fid: int, ts: float) -> None:
        with self._frame_cond:
            if fid <= self._frame_id:          # un decodificador más lento llegó tarde
                return
            self._frame, self._frame_id, self._frame_ts = frame, fid, ts
            self._frame_cond.notify_all()
        if not self._ready.is_set():
            self.height, self.width = frame.shape[:2]
            self._ready.set()

    @staticmethod
    def _is_jpeg(buf: np.ndarray) -> bool:
        return (buf.dtype == np.uint8 and buf.ndim <= 2 and buf.size > 4
                and (buf.ndim == 1 or buf.shape[0] == 1)
                and int(buf.flat[0]) == 0xFF and int(buf.flat[1]) == 0xD8)

    def _open(self) -> cv2.VideoCapture:
        cfg = self.cfg
        dev = resolve_camera_device(cfg.device)
        cap = open_camera(cfg, dev)
        code = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc = "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))
        self.raw_mode = False
        if cfg.raw_mjpeg and fourcc == "MJPG":
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
            for _ in range(5):
                ok, buf = cap.read()
                if ok and buf is not None:
                    self.raw_mode = self._is_jpeg(buf)
                    break
            if not self.raw_mode:
                cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        else:
            cap.read()                          # arranca el streaming
        configure_exposure(cap, cfg, dev)       # tras el primer frame: el stream ya está activo
        w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        LOG.info("Cámara %r (%s): %dx%d @ %.0f FPS, %s, buffers=%d, decodificación=%s",
                 cfg.device, _dev_path(dev), w, h, fps, fourcc,
                 int(cap.get(cv2.CAP_PROP_BUFFERSIZE)),
                 f"{cfg.decode_threads} hilos (JPEG crudo)" if self.raw_mode else "OpenCV interna")
        if fourcc != "MJPG":
            LOG.warning("La cámara no aceptó MJPG (%r): a 1080p los FPS caerán", fourcc)
        if (w, h) != (cfg.width, cfg.height) or (fps > 0 and abs(fps - cfg.fps) > 0.5):
            LOG.warning("La cámara no aceptó %dx%d @ %d FPS (quedó %dx%d @ %.0f)",
                        cfg.width, cfg.height, cfg.fps, w, h, fps)
        return cap

    def _open_retry(self) -> cv2.VideoCapture:
        """Primera apertura: reintenta durante open_retry_s (USB aún enumerando al encender)."""
        deadline = time.monotonic() + max(0.0, self.cfg.open_retry_s)
        attempt = 0
        while True:
            attempt += 1
            try:
                cap = self._open()
                if attempt > 1:
                    LOG.info("Cámara abierta al intento %d", attempt)
                return cap
            except RuntimeError as e:
                left = deadline - time.monotonic()
                if self._stop.is_set() or left <= 0.0:
                    raise RuntimeError(f"{e} (tras {attempt} intentos)") from None
                LOG.warning("%s; reintento %d en %.0f s", e, attempt, CAMERA_RETRY_INTERVAL_S)
                self._stop.wait(min(CAMERA_RETRY_INTERVAL_S, left))

    def _reopen(self) -> Optional[cv2.VideoCapture]:
        while not self._stop.is_set():
            try:
                return self._open()
            except RuntimeError as e:
                LOG.error("%s; reintentando en 1 s", e)
                self._stop.wait(1.0)
        return None

    def _capture_loop(self) -> None:
        cap: Optional[cv2.VideoCapture] = None
        meter = FPSMeter()
        fails = 0
        cap_id = 0
        try:
            cap = self._open_retry()
            while not self._stop.is_set():
                ok, buf = cap.read()           # RAW: solo memcpy -> el buffer vuelve al driver ya
                if not ok or buf is None:
                    fails += 1
                    if fails >= READ_FAIL_LIMIT:
                        LOG.error("La cámara no entrega frames; reabriendo...")
                        cap.release()
                        cap = self._reopen()
                        if cap is None:
                            break
                        fails = 0
                    else:
                        time.sleep(0.005)
                    continue
                fails = 0
                ts = time.perf_counter()
                cap_id += 1
                self.fps = meter.tick(ts)
                if self.raw_mode:
                    with self._jpeg_cond:
                        self._jpeg, self._jpeg_id, self._jpeg_ts = buf, cap_id, ts
                        self._jpeg_cond.notify()
                else:
                    self._publish(buf, cap_id, ts)
        except Exception as e:  # noqa: BLE001 - se registra y se cierra limpio
            self._error = f"{type(e).__name__}: {e}"
            LOG.error("Hilo de captura: %s", self._error)
        finally:
            if cap is not None:
                cap.release()
                LOG.info("Cámara liberada")
            self._stop.set()
            self._ready.set()
            for cond in (self._frame_cond, self._jpeg_cond):
                with cond:
                    cond.notify_all()

    def _decode_loop(self) -> None:
        flags = cv2.IMREAD_COLOR               # siempre a color: el gris solo se hace sobre el ROI
        while not self._stop.is_set():
            with self._jpeg_cond:
                self._jpeg_cond.wait_for(
                    lambda: self._jpeg_id > self._jpeg_taken or self._stop.is_set(), 0.5)
                if self._stop.is_set() or self._jpeg_id <= self._jpeg_taken:
                    continue
                buf, fid, ts = self._jpeg, self._jpeg_id, self._jpeg_ts
                self._jpeg_taken = fid         # los JPEG más viejos se descartan
            t0 = time.perf_counter()
            frame = cv2.imdecode(buf, flags)   # libera el GIL -> decodificación en paralelo
            self.decode_ms = 0.9 * self.decode_ms + 0.1 * (time.perf_counter() - t0) * 1e3
            if frame is not None:
                self._publish(frame, fid, ts)


# =============================================================================
# Detección ArUco (AprilTag 25h9)
# =============================================================================
def build_detector_params(subpixel: bool) -> "cv2.aruco.DetectorParameters":
    aruco = cv2.aruco
    try:
        p = aruco.DetectorParameters()          # OpenCV >= 4.7
    except (AttributeError, TypeError, cv2.error):
        p = aruco.DetectorParameters_create()   # OpenCV <= 4.6 (contrib)
    # Una sola pasada de umbralizado adaptativo (por defecto son 3: 3, 13, 23)
    p.adaptiveThreshWinSizeMin = 11
    p.adaptiveThreshWinSizeMax = 11
    p.adaptiveThreshWinSizeStep = 10
    p.adaptiveThreshConstant = 7
    p.minMarkerPerimeterRate = 0.03
    p.maxMarkerPerimeterRate = 4.0
    p.polygonalApproxAccuracyRate = 0.05   # tolera esquinas redondeadas por motion blur
    p.detectInvertedMarker = False
    if subpixel:
        p.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        p.cornerRefinementWinSize = 5
        p.cornerRefinementMaxIterations = 20
        p.cornerRefinementMinAccuracy = 0.05
    else:
        p.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
    return p


class ArucoBackend:
    """API nueva (ArucoDetector, OpenCV >= 4.7) con respaldo a la API antigua."""

    def __init__(self, subpixel: bool) -> None:
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("Esta build de OpenCV no incluye cv2.aruco "
                               "(instala python3-opencv u opencv-contrib-python)")
        self._dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
        self._params = build_detector_params(subpixel)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._fn: Callable = cv2.aruco.ArucoDetector(self._dict, self._params).detectMarkers
        else:
            self._fn = lambda g: cv2.aruco.detectMarkers(g, self._dict, parameters=self._params)

    def detect(self, gray: np.ndarray):
        corners, ids, _ = self._fn(gray)
        return corners, ids


# =============================================================================
# Filtro de Kalman 2D de velocidad constante
# =============================================================================
class ConstantVelocityKF:
    """Estado [x, y, vx, vy] en px (1080p) y px/s; dt real entre frames."""

    def __init__(self, accel_std: float, max_speed: float) -> None:
        self._q = accel_std ** 2
        self._vmax = max_speed
        self.x = np.zeros(4)
        self.P = np.eye(4)
        self.t = 0.0

    def reset(self, x: float, y: float, t: float,
              pos_std: float = 4.0, vel_std: float = 500.0) -> None:
        self.x[:] = (x, y, 0.0, 0.0)
        self.P = np.diag([pos_std ** 2, pos_std ** 2, vel_std ** 2, vel_std ** 2])
        self.t = t

    def predict(self, t: float) -> None:
        dt = clamp(t - self.t, 0.0, 0.5)
        self.t = t
        if dt <= 0.0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q, dt2 = self._q, dt * dt
        Q = np.zeros((4, 4))
        Q[0, 0] = Q[1, 1] = q * dt2 * dt2 / 4.0
        Q[0, 2] = Q[2, 0] = Q[1, 3] = Q[3, 1] = q * dt2 * dt / 2.0
        Q[2, 2] = Q[3, 3] = q * dt2
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, zx: float, zy: float, std: float) -> None:
        S = self.P[:2, :2] + np.eye(2) * std ** 2
        K = self.P[:, :2] @ np.linalg.inv(S)
        self.x = self.x + K @ (np.array([zx, zy]) - self.x[:2])
        self.P = self.P - K @ self.P[:2, :]
        speed = math.hypot(self.x[2], self.x[3])
        if speed > self._vmax:
            self.x[2:] *= self._vmax / speed

    @property
    def position(self) -> tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    @property
    def sigma(self) -> float:
        return math.sqrt(max(self.P[0, 0], self.P[1, 1]))


# =============================================================================
# Tracker corporal ligero (320x180)
# =============================================================================
class TemplateTracker:
    """Seguimiento por región: NCC en una ventana local + actualización lenta del modelo."""

    def __init__(self, min_score: float, margin: float = 0.6, learn_rate: float = 0.1) -> None:
        self._min, self._margin, self._lr = min_score, margin, learn_rate
        self._tpl: Optional[np.ndarray] = None
        self._box = (0, 0, 0, 0)

    def init(self, img: np.ndarray, box: tuple[int, int, int, int]) -> None:
        x, y, w, h = box
        self._tpl = to_gray(img)[y:y + h, x:x + w].copy()   # copia necesaria: el modelo persiste
        self._box = box

    def update(self, img: np.ndarray) -> tuple[bool, tuple[int, int, int, int]]:
        gray = to_gray(img)
        x, y, w, h = self._box
        H, W = gray.shape
        mx, my = int(w * self._margin) + 2, int(h * self._margin) + 2
        x0, y0 = max(0, x - mx), max(0, y - my)
        x1, y1 = min(W, x + w + mx), min(H, y + h + my)
        if x1 - x0 < w or y1 - y0 < h:
            return False, self._box
        res = cv2.matchTemplate(gray[y0:y1, x0:x1], self._tpl, cv2.TM_CCOEFF_NORMED)
        _, score, _, loc = cv2.minMaxLoc(res)
        if score < self._min:
            return False, self._box
        nx, ny = x0 + loc[0], y0 + loc[1]
        self._box = (nx, ny, w, h)
        if score > 0.7:
            self._tpl = cv2.addWeighted(self._tpl, 1.0 - self._lr,
                                        gray[ny:ny + h, nx:nx + w], self._lr, 0.0)
        return True, self._box

    def move_to(self, cx: float, cy: float) -> None:
        """Recentra la ventana de búsqueda (p. ej. en la predicción de Kalman)."""
        _, _, w, h = self._box
        self._box = (int(round(cx - w / 2)), int(round(cy - h / 2)), w, h)


class _OpenCVTracker:
    """Adaptador para KCF / MOSSE de OpenCV (contrib)."""

    def __init__(self, factory: Callable, color: bool) -> None:
        self._factory, self._color, self._t = factory, color, None

    def _prep(self, img: np.ndarray) -> np.ndarray:
        if self._color:                                  # KCF: color (features CN)
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img
        return to_gray(img)                              # MOSSE: gris

    def init(self, img: np.ndarray, box: tuple[int, int, int, int]) -> None:
        self._t = self._factory()
        self._t.init(self._prep(img), box)

    def update(self, img: np.ndarray):
        ok, box = self._t.update(self._prep(img))
        return bool(ok), tuple(int(round(v)) for v in box)


def _find_tracker_factory(kind: str) -> Optional[Callable]:
    paths = {"KCF": ("TrackerKCF_create", "legacy.TrackerKCF_create"),
             "MOSSE": ("legacy.TrackerMOSSE_create", "TrackerMOSSE_create")}
    for path in paths.get(kind, ()):
        obj = cv2
        for part in path.split("."):
            obj = getattr(obj, part, None)
            if obj is None:
                break
        if obj is not None:
            return obj
    return None


class BodyTracker:
    """
    Sigue SOLO el torso de la persona fijada (caja debajo del tag) en 320x180.
    Se inicializa en el último frame donde se vio el tag y devuelve la posición
    estimada del tag (centro de la caja + desplazamiento medido al iniciar).
    """

    def __init__(self, cfg: Config, sizes: ViewSizes) -> None:
        self.cfg = cfg
        self._k = sizes.tiny[0] / sizes.full[0]          # escala 1080p -> 320x180
        self._tw, self._th = sizes.tiny
        kind = cfg.body_tracker.upper()
        factory = None if kind == "TEMPLATE" else _find_tracker_factory(kind)
        if kind != "TEMPLATE" and factory is None:
            LOG.warning("Tracker %s no disponible (requiere opencv-contrib): uso TEMPLATE", kind)
            kind = "TEMPLATE"
        self.kind = kind
        self._make = (lambda: _OpenCVTracker(factory, color=(kind == "KCF"))) if factory else \
            (lambda: TemplateTracker(cfg.template_min_score))
        self._impl = None
        self._box = (0, 0, 0, 0)
        self._offset = (0.0, 0.0)
        self._fails = 0
        LOG.info("Tracker corporal: %s a %dx%d", kind, self._tw, self._th)

    @property
    def active(self) -> bool:
        return self._impl is not None

    def stop(self) -> None:
        self._impl = None

    def start(self, tiny: np.ndarray, geo: Geometry) -> bool:
        bw, bh, down = self.cfg.body_box
        side = max(geo.width, geo.height)
        k = self._k
        w, h = bw * side * k, bh * side * k
        x0 = (geo.cx - bw * side / 2) * k
        y0 = (geo.cy + down * side) * k                  # torso justo debajo del tag
        y0 = max(0.0, min(y0, self._th - h))             # persona cerca: subir la caja para que quepa
        x0, x1 = max(0.0, x0), min(float(self._tw), x0 + w)
        y1 = min(float(self._th), y0 + h)
        if x1 - x0 < self.cfg.body_min_px or y1 - y0 < self.cfg.body_min_px:
            return False                                 # persona demasiado lejos / fuera
        box = (int(x0), int(y0), int(x1 - x0), int(y1 - y0))
        self._impl = self._make()
        self._impl.init(tiny, box)
        self._box = box
        self._fails = 0
        bcx, bcy = (box[0] + box[2] / 2) / k, (box[1] + box[3] / 2) / k
        self._offset = (geo.cx - bcx, geo.cy - bcy)
        return True

    def update(self, tiny: np.ndarray, pred: tuple[float, float]) -> Optional[BodyEstimate]:
        """
        Estimación del tag a partir del torso; None si falla en ESTE frame.
        `pred` = posición del tag predicha por Kalman (1080p): la búsqueda se centra
        ahí (compensa movimientos rápidos) y el salto se mide contra ella.
        Tras BODY_MAX_FAILS fallos seguidos el tracker se desactiva.
        """
        if self._impl is None:
            return None
        k = self._k
        _, _, pw, ph = self._box
        bx, by = (pred[0] - self._offset[0]) * k, (pred[1] - self._offset[1]) * k
        if isinstance(self._impl, TemplateTracker):
            self._impl.move_to(bx, by)
        ok, box = self._impl.update(tiny)
        if ok and box[2] > 0 and box[3] > 0:
            jump = math.hypot((box[0] + box[2] / 2) - bx, (box[1] + box[3] / 2) - by)
            ok = jump <= self.cfg.body_max_jump * max(pw, ph)   # salto imposible: otra persona
        if not ok:
            self._fails += 1
            if self._fails > self.cfg.body_max_fails:
                self._impl = None
            return None
        self._fails = 0
        self._box = box
        k = self._k
        x, y, w, h = box
        return BodyEstimate((x + w / 2) / k + self._offset[0], (y + h / 2) / k + self._offset[1],
                            (x / k, y / k, (x + w) / k, (y + h) / k))


# =============================================================================
# Tracker de objetivo: SEARCH / LOCKED (TRACK, OCCLUDED) / STOP
# =============================================================================
class TargetTracker:
    def __init__(self, cfg: Config, sizes: ViewSizes) -> None:
        self.cfg = cfg
        self.sizes = sizes
        self._W, self._H = sizes.full
        self._ox, self._oy = (self._W - 1) / 2.0, (self._H - 1) / 2.0
        self._ssx, self._ssy = sizes.small[0] / self._W, sizes.small[1] / self._H
        self._scan_det = ArucoBackend(subpixel=False)
        self._roi_det = ArucoBackend(subpixel=cfg.subpixel_refine)
        self._kf = ConstantVelocityKF(cfg.kf_accel_std, cfg.kf_max_speed)
        self._body = BodyTracker(cfg, sizes) if cfg.enable_person_fallback else None
        self._scan_ids = set(cfg.people) | {cfg.stop_id}
        self._n = 0
        self._stopped = False
        self._stop_seen = -math.inf
        self._release()

    # ---------------- ciclo por frame ----------------
    def process(self, v: FrameViews) -> TrackResult:
        self._n += 1
        if self._stopped:
            return self._process_stop(v)
        if self._target is None:
            return self._process_search(v)
        return self._process_locked(v)

    def _process_stop(self, v: FrameViews) -> TrackResult:
        dets = self._scan(v)
        if self._has_stop(dets):
            self._stop_seen = v.ts
        if v.ts - self._stop_seen < self.cfg.stop_hold_s:
            return self._result(v, STOP, scanned=True)
        self._stopped = False
        LOG.info("STOP liberado -> SEARCH")
        return self._process_search(v, dets)

    def _process_search(self, v: FrameViews, dets: Optional[list[Detection]] = None) -> TrackResult:
        if dets is None:
            dets = self._scan(v)
        if self._has_stop(dets):
            return self._enter_stop(v)
        people = [d for d in dets if d.tag_id in self.cfg.people]
        if not people:
            return self._result(v, SEARCH, scanned=True)
        best = max(people, key=lambda d: d.geo.width * d.geo.height)   # la más cercana
        self._target = best.tag_id
        self._kf.reset(best.geo.cx, best.geo.cy, v.ts)
        self._on_hit(best, v, self.cfg.kf_meas_std_scan, fresh=True)
        LOG.info("Objetivo fijado: %s (ID %d)", self.cfg.people[self._target], self._target)
        return self._tag_result(v, best, "SCAN", None, True)

    def _process_locked(self, v: FrameViews) -> TrackResult:
        cfg = self.cfg
        self._kf.predict(v.ts)

        # 1) Tag oculto en frames previos: el tracker corporal guía el ROI
        body = None
        if self._body is not None and self._body.active:
            body = self._body.update(v.tiny, self._kf.position)   # None = fallo puntual
            if body is not None:
                self._kf.update(body.cx, body.cy, cfg.kf_meas_std_body)

        # 2) ROI dinámico en 1080p alrededor de la predicción
        px, py = self._kf.position
        roi = self._roi(px, py)
        hit, stop = self._detect_roi(v, roi, (px, py))
        if stop:
            return self._enter_stop(v)
        source = "TAG"
        if hit is None:
            self._misses += 1

        # 3) Escaneo global 640x360: STOP periódico, fallback a los N fallos y
        #    re-enganche periódico durante la oclusión
        m, n_fb = self._misses, cfg.roi_misses_fallback
        scanned = (self._n % cfg.stop_scan_every == 0 or m == n_fb
                   or (m > n_fb and m % cfg.rescan_every == 0))
        if scanned:
            dets = self._scan(v)
            if self._has_stop(dets):
                return self._enter_stop(v)
            if hit is None:
                mine = [d for d in dets if d.tag_id == self._target]
                if mine:
                    hit = min(mine, key=lambda d: (d.geo.cx - px) ** 2 + (d.geo.cy - py) ** 2)
                    source = "SCAN"

        if hit is not None:
            if self._misses > 0:
                lvl = logging.INFO if self._misses >= cfg.roi_misses_fallback else logging.DEBUG
                LOG.log(lvl, "Tag de %s re-enganchado (%s) tras %d frames",
                        self.cfg.people[self._target], source, self._misses)
            self._on_hit(hit, v, cfg.kf_meas_std_tag if source == "TAG" else cfg.kf_meas_std_scan)
            return self._tag_result(v, hit, source, roi, scanned)

        # 4) Tag no visible: tracker corporal (oclusión) o predicción breve
        if self._body is not None and not self._body.active and self._misses == 1:
            if self._body.start(self._last_views.tiny, self._last_det.geo):
                body = self._body.update(v.tiny, self._kf.position)
                if body is not None:
                    self._kf.update(body.cx, body.cy, cfg.kf_meas_std_body)

        lost_for = v.ts - self._last_hit_t
        within = lost_for <= cfg.occlusion_timeout_s
        if body is not None and within:
            return self._coast_result(v, "BODY", body.cx, body.cy, roi, body.box, scanned)
        body_coasting = self._body is not None and self._body.active and within
        if body_coasting or self._misses < n_fb:
            # Fallo puntual del torso (oclusión parcial) o del tag: predicción de Kalman
            px, py = self._kf.position
            return self._coast_result(v, "PREDICT", px, py, roi, None, scanned)

        LOG.info("Objetivo %s perdido (%.2f s sin tag)", self.cfg.people[self._target], lost_for)
        self._release()
        return self._result(v, SEARCH, scanned=scanned)

    # ---------------- detección ----------------
    def _scan(self, v: FrameViews) -> list[Detection]:
        corners, ids = self._scan_det.detect(v.small_gray)
        return self._to_dets(corners, ids, lambda p: to_full_res(p, self._ssx, self._ssy),
                             self._scan_ids)

    def _roi(self, px: float, py: float) -> tuple[int, int, int, int]:
        cfg = self.cfg
        base = max(cfg.roi_min_half, cfg.roi_scale * self._size)
        # Con el tracker corporal activo la posición es conocida: el ROI crece poco
        # (evita detectar sobre todo el frame en cada frame durante una oclusión).
        body_guided = self._body is not None and self._body.active
        growth = min(self._misses, 1 if body_guided else 6)
        half = base * cfg.roi_growth ** growth + cfg.roi_sigma_k * self._kf.sigma
        px, py = clamp(px, 0.0, self._W - 1.0), clamp(py, 0.0, self._H - 1.0)
        return (int(max(0.0, px - half)), int(max(0.0, py - half)),
                int(min(self._W, px + half + 1)), int(min(self._H, py + half + 1)))

    def _detect_roi(self, v: FrameViews, roi, pred) -> tuple[Optional[Detection], bool]:
        x0, y0, x1, y1 = roi
        gray = v.roi_gray(x0, y0, x1, y1)
        rh, rw = gray.shape
        s = min(1.0, self.cfg.roi_max_dim / max(rw, rh))
        if self._size > self.cfg.roi_tag_px:
            s = min(s, self.cfg.roi_tag_px / self._size)
        sx = sy = 1.0
        if s < 0.999:
            dw, dh = max(1, round(rw * s)), max(1, round(rh * s))
            gray = cv2.resize(gray, (dw, dh), interpolation=cv2.INTER_AREA)
            sx, sy = dw / rw, dh / rh
        allowed = {self._target, self.cfg.stop_id}
        corners, ids = self._roi_det.detect(gray)
        dets = self._to_dets(corners, ids, lambda p: to_full_res(p, sx, sy, x0, y0), allowed)
        mine = [d for d in dets if d.tag_id == self._target]
        retry_s = self.cfg.roi_retry_tag_px / max(self._size, 1.0)
        if not mine and retry_s < 0.8 * sx and max(rw, rh) * sx <= self.cfg.roi_retry_max_dim:
            # 2º intento con el tag a ~ROI_RETRY_TAG_PX: el blur pasa a ocupar menos
            # píxeles y el umbral adaptativo de 11 px cubre mejor el borde (barato).
            dw, dh = max(1, round(rw * retry_s)), max(1, round(rh * retry_s))
            g2 = cv2.resize(gray, (dw, dh), interpolation=cv2.INTER_AREA)
            rx, ry = dw / rw, dh / rh
            corners, ids = self._scan_det.detect(g2)
            dets += self._to_dets(corners, ids, lambda p: to_full_res(p, rx, ry, x0, y0), allowed)
            mine = [d for d in dets if d.tag_id == self._target]
        hit = min(mine, key=lambda d: (d.geo.cx - pred[0]) ** 2 + (d.geo.cy - pred[1]) ** 2) \
            if mine else None
        return hit, self._has_stop(dets)

    @staticmethod
    def _to_dets(corners, ids, mapper: Callable, allowed: set[int]) -> list[Detection]:
        if ids is None:
            return []
        out = []
        for c, i in zip(corners, ids.ravel()):
            tag_id = int(i)
            if tag_id in allowed:
                pts = mapper(c.reshape(4, 2))
                out.append(Detection(tag_id, pts, tag_geometry(pts)))
        return out

    def _has_stop(self, dets: list[Detection]) -> bool:
        return any(d.tag_id == self.cfg.stop_id for d in dets)

    # ---------------- estado ----------------
    def _release(self) -> None:
        self._target: Optional[int] = None
        self._misses = 0
        self._size = 0.0
        self._last_hit_t = 0.0
        self._last_views: Optional[FrameViews] = None
        self._last_det: Optional[Detection] = None
        if getattr(self, "_body", None) is not None:
            self._body.stop()

    def _enter_stop(self, v: FrameViews) -> TrackResult:
        if not self._stopped:
            LOG.info("STOP (ID %d) detectado: seguimiento detenido", self.cfg.stop_id)
        self._release()
        self._stopped = True
        self._stop_seen = v.ts
        return self._result(v, STOP, scanned=True)

    def _on_hit(self, d: Detection, v: FrameViews, meas_std: float, fresh: bool = False) -> None:
        if not fresh:
            px, py = self._kf.position
            if math.hypot(d.geo.cx - px, d.geo.cy - py) > max(150.0, 5.0 * self._kf.sigma):
                self._kf.reset(d.geo.cx, d.geo.cy, v.ts)    # re-enganche lejos: reiniciar filtro
            else:
                self._kf.update(d.geo.cx, d.geo.cy, meas_std)
        self._size = max(d.geo.width, d.geo.height)
        self._misses = 0
        self._last_hit_t = v.ts
        self._last_views, self._last_det = v, d          # referencia (sin copia) para el tracker corporal
        if self._body is not None:
            self._body.stop()

    # ---------------- resultados ----------------
    def _result(self, v: FrameViews, state: str, **kw) -> TrackResult:
        return TrackResult(frame_id=v.frame_id, timestamp=v.ts, state=state, **kw)

    def _xy_result(self, v, state, source, cx, cy, width, **kw) -> TrackResult:
        cx, cy = clamp(cx, 0.0, self._W - 1.0), clamp(cy, 0.0, self._H - 1.0)
        return self._result(v, state, person=self.cfg.people[self._target], tag_id=self._target,
                            source=source, cx=cx, cy=cy, width_px=width,
                            ex=(cx - self._ox) / self._ox, ey=(cy - self._oy) / self._oy, **kw)

    def _tag_result(self, v, d: Detection, source, roi, scanned) -> TrackResult:
        return self._xy_result(v, TRACK, source, d.geo.cx, d.geo.cy, d.geo.width,
                               corners=d.corners, roi=roi, scanned=scanned)

    def _coast_result(self, v, source, cx, cy, roi, body_box, scanned) -> TrackResult:
        return self._xy_result(v, OCCLUDED, source, cx, cy, self._last_det.geo.width,
                               roi=roi, body_box=body_box, scanned=scanned)


# =============================================================================
# Visualización (throttled)
# =============================================================================
STATE_COLORS = {TRACK: (0, 220, 0), OCCLUDED: (0, 165, 255), SEARCH: (180, 180, 180),
                STOP: (0, 0, 255)}


def _text(img: np.ndarray, txt: str, org: tuple[int, int],
          color: tuple[int, int, int] = (255, 255, 255), scale: float = 0.45) -> None:
    """Texto sobre fondo negro sólido (legible sobre cualquier imagen)."""
    (tw, th), base = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = org
    cv2.rectangle(img, (x - 3, y - th - 4), (x + tw + 3, y + base + 2), (0, 0, 0), -1)
    cv2.putText(img, txt, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


class Visualizer:
    WINDOW = "AprilTag tag25h9"

    def __init__(self, cfg: Config, sizes: ViewSizes) -> None:
        self.cfg = cfg
        w, h = sizes.full
        self._size = (cfg.gui_width, max(1, round(h * cfg.gui_width / w)))
        self._reuse_small = self._size == sizes.small
        self._s = cfg.gui_width / w
        self._count = 0
        self._fps = FPSMeter(10)
        self.gui_fps = 0.0
        cv2.namedWindow(self.WINDOW, cv2.WINDOW_AUTOSIZE)

    def maybe_show(self, v: FrameViews, r: TrackResult, stream: CameraStream,
                   extra_lines: Sequence[str] = ()) -> int:
        """
        Dibuja 1 de cada GUI_EVERY_N frames (evitando los frames con escaneo).
        Devuelve la tecla pulsada (cv2.waitKey) o -1.
        """
        n = self.cfg.gui_every_n
        self._count += 1
        if self._count < n or (r.scanned and self._count < 2 * n):
            return -1
        self._count = 0
        self.gui_fps = self._fps.tick()

        base = v.small if self._reuse_small else cv2.resize(v.full, self._size,
                                                            interpolation=cv2.INTER_AREA)
        # Copia necesaria: la vista small (BGR) se comparte con el tracker corporal.
        # Si la cámara entregara un formato mono, se pasa a BGR: la GUI siempre es a color.
        disp = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR) if base.ndim == 2 else base.copy()
        s = self._s
        h, w = disp.shape[:2]
        center = (w // 2, h // 2)
        color = STATE_COLORS.get(r.state, (255, 255, 255))
        cv2.drawMarker(disp, center, (200, 200, 200), cv2.MARKER_CROSS, 18, 1)

        if r.roi is not None:
            x0, y0, x1, y1 = r.roi
            cv2.rectangle(disp, (int(x0 * s), int(y0 * s)), (int(x1 * s), int(y1 * s)),
                          (255, 200, 0), 1)
        if r.body_box is not None:
            x0, y0, x1, y1 = r.body_box
            cv2.rectangle(disp, (int(x0 * s), int(y0 * s)), (int(x1 * s), int(y1 * s)), color, 2)
        if r.corners is not None:
            pts = np.round(r.corners * s).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(disp, [pts], True, color, 2, cv2.LINE_AA)
        if r.cx is not None:
            c = (int(round(r.cx * s)), int(round(r.cy * s)))
            cv2.line(disp, center, c, color, 1, cv2.LINE_AA)
            cv2.circle(disp, c, 4, (0, 0, 255), -1, cv2.LINE_AA)
            _text(disp, f"cx={r.cx:.1f} cy={r.cy:.1f} w={r.width_px:.1f}px ex={r.ex:+.3f}",
                  (8, h - 10))

        label = r.state if r.person is None else f"{r.person}  {r.state}  [{r.source}]"
        _text(disp, label, (8, 44), color, 0.55)
        _text(disp, f"Proc {r.fps:4.1f} FPS ({r.proc_ms:4.1f} ms)  Cam {stream.fps:4.1f}  "
                    f"Dec {stream.decode_ms:4.1f} ms  GUI {self.gui_fps:4.1f}", (8, 20))
        for i, line in enumerate(extra_lines):            # información del controlador
            _text(disp, line, (8, 68 + 20 * i), (255, 255, 255), 0.42)
        if r.state == STOP:
            _text(disp, "STOP", (w // 2 - 40, h // 2 + 12), STATE_COLORS[STOP], 1.2)

        cv2.imshow(self.WINDOW, disp)
        key = cv2.waitKey(1)
        return -1 if key < 0 else key & 0xFF

    @staticmethod
    def close() -> None:
        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except cv2.error:
            pass


# =============================================================================
# API pública: VisionTracker -> VisionOutput
# =============================================================================
NO_TARGET = "NONE"


@dataclass(frozen=True, slots=True)
class VisionOutput:
    """Resultado de un frame. Las coordenadas son de la imagen completa (1920x1080)."""
    is_tracking: bool                      # True en TRACK y OCCLUDED
    target_name: str                       # "DAVID" | "CUAUHTEMOC" | "NONE"
    tag_x: Optional[float]                 # centro del tag en pantalla [px]; None sin objetivo
    tag_y: Optional[float]
    width_px: float                        # ancho del tag [px]; 0 sin objetivo
    frame: np.ndarray = field(repr=False)  # BGR a color, resolución completa (NO modificar)
    state: str = SEARCH                    # TRACK | OCCLUDED | SEARCH | STOP
    ex: float = 0.0                        # error horizontal normalizado [-1, 1] (+ = derecha)
    ey: float = 0.0                        # error vertical normalizado [-1, 1] (+ = abajo)
    fps: float = 0.0                       # FPS reales de procesamiento
    frame_w: int = 0
    frame_h: int = 0
    frame_id: int = 0
    timestamp: float = 0.0                 # time.perf_counter() de la captura
    result: Optional[TrackResult] = field(default=None, repr=False)   # detalle (GUI/depuración)
    views: Optional[FrameViews] = field(default=None, repr=False)

    def as_tuple(self) -> tuple:
        """(is_tracking, target_name, tag_x, tag_y, width_px)"""
        return (self.is_tracking, self.target_name, self.tag_x, self.tag_y, self.width_px)


class VisionTracker:
    """
    Cámara + detección + seguimiento. read() procesa el frame más reciente;
    show() dibuja la GUI a color (1 de cada GUI_EVERY_N llamadas) y devuelve la
    tecla pulsada. Úsalo como context manager para garantizar el cierre de la
    cámara (la libera su propio hilo de captura).
    """

    def __init__(self, cfg: Optional[Config] = None) -> None:
        self.cfg = cfg or Config()
        self.stream = CameraStream(self.cfg)
        self.sizes: Optional[ViewSizes] = None
        self.tracker: Optional[TargetTracker] = None
        self._viz: Optional[Visualizer] = None
        self._fps = FPSMeter()
        self._last_id = 0
        self._started = False

    # ---------------- ciclo de vida ----------------
    def start(self) -> "VisionTracker":
        cfg = self.cfg
        cv2.setUseOptimized(True)
        if CV_THREADS is not None:
            cv2.setNumThreads(CV_THREADS)
        if cfg.show_gui and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            LOG.warning("No hay pantalla disponible: se desactiva la GUI")
            cfg.show_gui = False
        self.stream.start()
        self.sizes = ViewSizes.build(self.stream.width, self.stream.height,
                                     cfg.search_width, cfg.track_width)
        self.tracker = TargetTracker(cfg, self.sizes)
        if cfg.show_gui:
            try:
                self._viz = Visualizer(cfg, self.sizes)
            except cv2.error as e:
                LOG.warning("No se pudo abrir la ventana de OpenCV (%s): GUI desactivada",
                            str(e).strip().splitlines()[-1] if str(e).strip() else e)
                cfg.show_gui = False
        self._started = True
        LOG.info("Frame %dx%d | escaneo %dx%d | tracker corporal %dx%d | GUI %s",
                 *self.sizes.full, *self.sizes.small, *self.sizes.tiny,
                 "color" if cfg.show_gui else "off")
        return self

    def close(self) -> None:
        if self._viz is not None:
            self._viz.close()
            self._viz = None
        self.stream.stop()
        self._started = False

    def __enter__(self) -> "VisionTracker":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.close()

    # ---------------- propiedades ----------------
    @property
    def alive(self) -> bool:
        """False si el hilo de captura terminó (error irrecuperable)."""
        return self._started and self.stream.alive

    @property
    def frame_size(self) -> tuple[int, int]:
        return self.sizes.full if self.sizes else (self.cfg.width, self.cfg.height)

    @property
    def gui_enabled(self) -> bool:
        return self._viz is not None

    # ---------------- uso por frame ----------------
    def read(self, timeout: float = 1.0) -> Optional[VisionOutput]:
        """Procesa el siguiente frame nuevo; None si no llegó ninguno en `timeout` s."""
        assert self.tracker is not None and self.sizes is not None, "llama start() primero"
        pkt = self.stream.read(self._last_id, timeout=timeout)
        if pkt is None:
            return None
        self._last_id, frame, ts = pkt
        t0 = time.perf_counter()
        views = FrameViews(frame, self._last_id, ts, self.sizes)
        r = self.tracker.process(views)
        r.proc_ms = (time.perf_counter() - t0) * 1e3
        r.fps = self._fps.tick()
        tracking = r.state in (TRACK, OCCLUDED) and r.cx is not None
        w, h = self.sizes.full
        return VisionOutput(
            is_tracking=tracking,
            target_name=r.person if tracking and r.person else NO_TARGET,
            tag_x=r.cx if tracking else None,
            tag_y=r.cy if tracking else None,
            width_px=r.width_px if tracking else 0.0,
            frame=frame, state=r.state, ex=r.ex, ey=r.ey, fps=r.fps,
            frame_w=w, frame_h=h, frame_id=self._last_id, timestamp=ts,
            result=r, views=views)

    def show(self, out: VisionOutput, extra_lines: Sequence[str] = ()) -> int:
        """GUI a color (throttled). Devuelve la tecla pulsada o -1 (también sin GUI)."""
        if self._viz is None or out.views is None or out.result is None:
            return -1
        return self._viz.maybe_show(out.views, out.result, self.stream, extra_lines)


def format_output(o: VisionOutput) -> str:
    f = lambda x: "None" if x is None else f"{x:.1f}"   # noqa: E731
    return (f"tracking={o.is_tracking!s:5} target={o.target_name:10} state={o.state:8} "
            f"tag_x={f(o.tag_x):>6} tag_y={f(o.tag_y):>6} width_px={o.width_px:5.1f} "
            f"fps={o.fps:4.1f}")


# =============================================================================
# Main (prueba del módulo de visión por sí solo)
# =============================================================================
def build_arg_parser(add_help: bool = True) -> argparse.ArgumentParser:
    """Opciones de cámara/visión (follow_controller.py las reutiliza)."""
    p = argparse.ArgumentParser(description="Visión: AprilTag tag25h9 + seguimiento",
                                add_help=add_help)
    g = p.add_argument_group("visión")
    g.add_argument("--device", default=str(CAMERA_DEVICE),
                   help='"auto", índice, /dev/videoN o /dev/v4l/by-id/...')
    g.add_argument("--camera-retry", type=float, default=CAMERA_OPEN_RETRY_S,
                   help="segundos reintentando abrir la cámara al arrancar")
    g.add_argument("--width", type=int, default=FRAME_WIDTH)
    g.add_argument("--height", type=int, default=FRAME_HEIGHT)
    g.add_argument("--fps", type=int, default=CAMERA_FPS)
    g.add_argument("--exposure", type=int, default=EXPOSURE_VALUE, help="unidades de 100 µs")
    g.add_argument("--auto-exposure", action="store_true")
    g.add_argument("--gain", type=int, default=GAIN_VALUE)
    g.add_argument("--buffers", type=int, default=CAPTURE_BUFFERS)
    g.add_argument("--decode-threads", type=int, default=DECODE_THREADS)
    g.add_argument("--stop-scan-every", type=int, default=STOP_SCAN_EVERY)
    g.add_argument("--person-fallback", action=argparse.BooleanOptionalAction,
                   default=ENABLE_PERSON_FALLBACK)
    g.add_argument("--body-tracker", default=BODY_TRACKER, choices=["KCF", "MOSSE", "TEMPLATE"])
    g.add_argument("--occlusion-timeout", type=float, default=OCCLUSION_TIMEOUT_S)
    g.add_argument("--gui", action=argparse.BooleanOptionalAction, default=SHOW_GUI)
    g.add_argument("--gui-every", type=int, default=GUI_EVERY_N)
    g.add_argument("--print-interval", type=float, default=PRINT_INTERVAL_S)
    return p


def config_from_args(a: argparse.Namespace) -> Config:
    return Config(
        device=int(a.device) if a.device.isdigit() else a.device,
        open_retry_s=max(0.0, a.camera_retry),
        width=a.width, height=a.height, fps=a.fps,
        manual_exposure=not a.auto_exposure, exposure=a.exposure, gain=a.gain,
        capture_buffers=max(1, a.buffers), decode_threads=max(1, a.decode_threads),
        stop_scan_every=max(1, a.stop_scan_every),
        enable_person_fallback=a.person_fallback, body_tracker=a.body_tracker,
        occlusion_timeout_s=a.occlusion_timeout, show_gui=a.gui,
        gui_every_n=max(1, a.gui_every), print_interval=a.print_interval,
    )


def _raise_keyboard_interrupt(signum, frame) -> None:   # SIGTERM -> mismo cierre que Ctrl+C
    raise KeyboardInterrupt


def line_buffered_output() -> None:
    """stdout/stderr por línea: logs en tiempo real con journalctl (además de PYTHONUNBUFFERED=1)."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(line_buffering=True)
        except (AttributeError, ValueError):
            pass


def main(argv: Optional[list[str]] = None) -> int:
    line_buffered_output()
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(threadName)s: %(message)s")
    cfg = config_from_args(build_arg_parser().parse_args(argv))
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    try:
        with VisionTracker(cfg) as vision:
            last_print = -math.inf
            while True:
                out = vision.read(timeout=1.0)
                if out is None:
                    if not vision.alive:
                        LOG.error("La captura se detuvo")
                        return 1
                    continue
                if vision.gui_enabled:
                    if vision.show(out) in (ord("q"), 27):
                        break
                elif out.timestamp - last_print >= cfg.print_interval:
                    last_print = out.timestamp
                    print(format_output(out), flush=True)
    except KeyboardInterrupt:
        LOG.info("Interrumpido por el usuario")
    except RuntimeError as e:
        LOG.error("%s", e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
