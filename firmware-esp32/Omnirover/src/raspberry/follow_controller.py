#!/usr/bin/env python3
from __future__ import annotations

import argparse
import dataclasses
import glob
import json
import logging
import math
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import vision_tracker as vt
from vision_tracker import VisionOutput, VisionTracker

try:
    import serial
    from serial import SerialException, SerialTimeoutException
except ImportError:
    serial = None
    SerialException = SerialTimeoutException = OSError

try:
    import config as _user_config
except ImportError:
    _user_config = None


def _cfg(name: str, default):
    return getattr(_user_config, name, default)


WHEEL_RADIUS_M = 0.04
WHEEL_MAX_RAD_S = 7.6
V_MAX_MPS = 0.15
MOTION_DEADBAND_MPS = 0.001

KP_DIST = _cfg("KP_DIST", 0.40)
KP_LATERAL = _cfg("KP_LATERAL", 0.30)
KP_CAMERA_WZ = _cfg("KP_CAMERA_WZ", 0.6)
WZ_MAX_CAMERA_RAD_S = _cfg("WZ_MAX_CAMERA_RAD_S", 0.6)
CAMERA_EX_DEADBAND = _cfg("CAMERA_EX_DEADBAND", 0.02)
CAMERA_ACCEL_MPS2 = _cfg("CAMERA_ACCEL_MPS2", 0.30)
CAMERA_DECEL_MPS2 = _cfg("CAMERA_DECEL_MPS2", 0.30)
CAMERA_ALPHA_RAD_S2 = _cfg("CAMERA_ALPHA_RAD_S2", 1.5)
DISTANCE_REF_M = _cfg("DISTANCE_REF_M", 0.6)
DEFAULT_TAG_REF_WIDTH_PX = _cfg("DEFAULT_TAG_REF_WIDTH_PX", 140.0)
TAG_CALIBRATION_FILE = _cfg("TAG_CALIBRATION_FILE",
                            str(Path(__file__).with_name("tag_calibration.json")))
TAG_MIN_SIDE_PX = _cfg("TAG_MIN_SIDE_PX", 12)
OCCLUDED_DISTANCE_CONTROL = _cfg("OCCLUDED_DISTANCE_CONTROL", False)

LANDMARKS: dict[str, tuple[float, float, float]] = {
    "CENTER":     (0.0, 0.0, 0.0),
    "TOP_RIGHT":  (1.5, 1.5, 0.0),
    "TOP_LEFT":   (-1.5, 1.5, 0.0),
    "DOWN_RIGHT": (1.5, -1.5, 0.0),
    "DOWN_LEFT":  (-1.5, -1.5, 0.0),
}
LANDMARK_KEYS = ("TOP_RIGHT", "TOP_LEFT", "DOWN_RIGHT", "DOWN_LEFT", "CENTER")
KP_LANDMARK = _cfg("KP_LANDMARK", 0.5)
V_MAX_LANDMARK_MPS = min(_cfg("V_MAX_LANDMARK_MPS", 0.10), V_MAX_MPS)
V_MIN_LANDMARK_MPS = _cfg("V_MIN_LANDMARK_MPS", 0.03)
LANDMARK_ACCEL_MPS2 = _cfg("LANDMARK_ACCEL_MPS2", 0.08)
LANDMARK_DECEL_MPS2 = _cfg("LANDMARK_DECEL_MPS2", 0.08)
LANDMARK_ALPHA_RAD_S2 = _cfg("LANDMARK_ALPHA_RAD_S2", 0.8)
KP_THETA = _cfg("KP_THETA", 2.0)
KI_THETA = _cfg("KI_THETA", 0.8)
WZ_I_MAX_RAD_S = _cfg("WZ_I_MAX_RAD_S", 0.15)
WZ_MAX_RAD_S = _cfg("WZ_MAX_RAD_S", 0.5)
HEADING_DEADBAND_RAD = _cfg("HEADING_DEADBAND_RAD", 0.005)
LANDMARK_TOL_M = _cfg("LANDMARK_TOL_M", 0.02)
LANDMARK_RELEASE_M = _cfg("LANDMARK_RELEASE_M", 0.04)
THETA_TOL_RAD = _cfg("THETA_TOL_RAD", 0.02)
THETA_RELEASE_RAD = _cfg("THETA_RELEASE_RAD", 0.05)
LANDMARK_DIVERGE_M = _cfg("LANDMARK_DIVERGE_M", 0.30)
HEADING_DIVERGE_RAD = _cfg("HEADING_DIVERGE_RAD", 0.40)
POSE_TIMEOUT_S = _cfg("POSE_TIMEOUT_S", 0.5)
THETA_IN_DEGREES = _cfg("THETA_IN_DEGREES", False)
LANDMARK_PRINT_S = 0.5

MECANUM_LX_M = _cfg("MECANUM_LX_M", 0.10)
MECANUM_LY_M = _cfg("MECANUM_LY_M", 0.10)
WZ_WHEEL_SIGNS = tuple(_cfg("WZ_WHEEL_SIGNS", (-1.0, +1.0, -1.0, +1.0)))
if len(WZ_WHEEL_SIGNS) != 4 or any(abs(abs(float(v)) - 1.0) > 1e-9 for v in WZ_WHEEL_SIGNS):
    raise SystemExit(f"WZ_WHEEL_SIGNS inválido en config.py: {WZ_WHEEL_SIGNS} (4 valores +1/-1)")
WZ_WHEEL_SIGNS = tuple(float(v) for v in WZ_WHEEL_SIGNS)
ODOM_HEADING_OFFSET_RAD = float(_cfg("ODOM_HEADING_OFFSET_RAD", math.pi / 2))
INVERT_THETA = bool(_cfg("INVERT_THETA", False))
INVERT_VX = bool(_cfg("INVERT_VX", False))
INVERT_VY = bool(_cfg("INVERT_VY", True))
INVERT_WZ = bool(_cfg("INVERT_WZ", False))
INVERT_ODOM_VY = bool(_cfg("INVERT_ODOM_VY", True))
ODOM_JUMP_M = _cfg("ODOM_JUMP_M", 0.10)
INVERT_CAMERA_VY = bool(_cfg("INVERT_CAMERA_VY", _cfg("CAMERA_INVERT_VY", False)))
INVERT_CAMERA_WZ = bool(_cfg("INVERT_CAMERA_WZ", False))
KIN_TEST_V_MPS = 0.08
KIN_TEST_WZ_RAD_S = 0.6

CONTROL_HZ = _cfg("CONTROL_HZ", 30.0)
VISION_STALE_S = 0.3
MOTION_ENABLED = True
PRINT_INTERVAL_S = 0.5

SERIAL_PORT = _cfg("SERIAL_PORT", "auto")
SERIAL_BAUD = _cfg("SERIAL_BAUD", 115200)
SERIAL_USB_CANDIDATES = ("/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*")
SERIAL_GPIO_CANDIDATES = ("/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0")
SERIAL_CANDIDATES = SERIAL_USB_CANDIDATES + SERIAL_GPIO_CANDIDATES
SERIAL_USB_WAIT_S = _cfg("SERIAL_USB_WAIT_S", 10.0)
SERIAL_RETRY_MAX_S = 2.0
SERIAL_WARN_EVERY_S = 30.0
READY_RESEND_S = 1.0
RX_MAX_LINE = 512
PROTOCOL_NAME = "SAOAH_UART_V2"

CAMERA, LANDMARK = "CAMERA", "LANDMARK"
MODES = (CAMERA, LANDMARK)
WAIT_MODE, WAIT_START, RUN = "WAIT_MODE", "WAIT_START", "RUN"
NONE = "NONE"

LOG = logging.getLogger("follow_controller")


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _finite(v: float, default: float = 0.0) -> float:
    return v if math.isfinite(v) else default


@dataclass(frozen=True)
class CameraMeasurement:
    x: float = 0.0
    y: float = 0.0
    width_px: float = 0.0
    ex: float = 0.0
    ey: float = 0.0
    size_error: float = 0.0
    valid: int = 0
    distance_calibrated: bool = False
    occluded: bool = False


def camera_measurement(out: Optional[VisionOutput],
                       ref_width_px: Optional[float]) -> CameraMeasurement:
    if out is None or not out.is_tracking or out.tag_x is None or out.frame_w <= 0:
        return CameraMeasurement()
    fw, fh = out.frame_w, out.frame_h
    ex = clamp((out.tag_x - fw / 2.0) / (fw / 2.0), -1.0, 1.0)
    ey = clamp((out.tag_y - fh / 2.0) / (fh / 2.0), -1.0, 1.0)
    calibrated = (ref_width_px is not None and math.isfinite(ref_width_px)
                  and ref_width_px > 0 and out.width_px > 0)
    size_error = clamp(1.0 - out.width_px / ref_width_px, -1.0, 1.0) if calibrated else 0.0
    return CameraMeasurement(x=out.tag_x, y=out.tag_y, width_px=out.width_px,
                             ex=ex, ey=ey, size_error=size_error, valid=1,
                             distance_calibrated=calibrated,
                             occluded=out.state == vt.OCCLUDED)


def wrap_to_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def mecanum_wheels(vx: float, vy: float, wz: float = 0.0
                   ) -> tuple[float, float, float, list[float]]:
    r = WHEEL_RADIUS_M
    k = MECANUM_LX_M + MECANUM_LY_M
    base = (vx + vy, vx - vy, vx - vy, vx + vy)
    wheels = [(b + sg * k * wz) / r for b, sg in zip(base, WZ_WHEEL_SIGNS)]
    highest = max(abs(v) for v in wheels)
    if highest > WHEEL_MAX_RAD_S:
        scale = WHEEL_MAX_RAD_S / highest
        wheels = [v * scale for v in wheels]
        vx, vy, wz = vx * scale, vy * scale, wz * scale
    return vx, vy, wz, wheels


def ccw_sign() -> float:
    return -1.0 if INVERT_THETA else 1.0


def proportional_reference(m: CameraMeasurement) -> tuple[float, float, float, list[float]]:
    if not m.valid:
        return 0.0, 0.0, 0.0, [0.0] * 4
    vx = KP_DIST * m.size_error if m.distance_calibrated else 0.0
    if m.occluded and not OCCLUDED_DISTANCE_CONTROL:
        vx = 0.0
    ex = math.copysign(max(0.0, abs(m.ex) - CAMERA_EX_DEADBAND), m.ex)
    vy = -KP_LATERAL * ex * (-1.0 if INVERT_CAMERA_VY else 1.0)
    wz = -KP_CAMERA_WZ * ex * ccw_sign() * (-1.0 if INVERT_CAMERA_WZ else 1.0)
    vx = clamp(vx, -V_MAX_MPS, V_MAX_MPS)
    vy = clamp(vy, -V_MAX_MPS, V_MAX_MPS)
    wz = clamp(wz, -WZ_MAX_CAMERA_RAD_S, WZ_MAX_CAMERA_RAD_S)
    if abs(vx) < MOTION_DEADBAND_MPS:
        vx = 0.0
    if abs(vy) < MOTION_DEADBAND_MPS:
        vy = 0.0
    return body_to_wheels(vx, vy, wz)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float
    t: float


@dataclass(frozen=True)
class LandmarkCommand:
    vx: float
    vy: float
    wz: float
    wheels: list[float]
    dx_world: float
    dy_world: float
    dist: float
    heading_err: float
    arrived: bool


def odom_heading(theta: float) -> float:
    return (-theta if INVERT_THETA else theta) + ODOM_HEADING_OFFSET_RAD


def world_to_body(dx: float, dy: float, theta: float) -> tuple[float, float]:
    psi = odom_heading(theta)
    c, s = math.cos(psi), math.sin(psi)
    return c * dx + s * dy, -s * dx + c * dy


def body_to_wheels(vx: float, vy: float, wz: float) -> tuple[float, float, float, list[float]]:
    sx = -1.0 if INVERT_VX else 1.0
    sy = 1.0 if INVERT_VY else -1.0
    sz = -1.0 if INVERT_WZ else 1.0
    mvx, mvy, mwz, wheels = mecanum_wheels(sx * vx, sy * vy, sz * wz)
    return sx * mvx, sy * mvy, sz * mwz, wheels


def heading_rate(heading_err: float, wz_i: float = 0.0) -> float:
    wz = wz_i if abs(heading_err) < HEADING_DEADBAND_RAD else KP_THETA * heading_err + wz_i
    return clamp(wz, -WZ_MAX_RAD_S, WZ_MAX_RAD_S)


class LateralOdomFix:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._raw: Optional[tuple[float, float, float]] = None
        self._xy = (0.0, 0.0)

    def apply(self, x: float, y: float, theta: float) -> tuple[float, float]:
        raw, self._raw = self._raw, (x, y, theta)
        if raw is None or (x == 0.0 and y == 0.0) or \
                math.hypot(x - raw[0], y - raw[1]) > ODOM_JUMP_M:
            self._xy = (x, y)
            return self._xy
        psi = odom_heading(raw[2] + 0.5 * wrap_to_pi(theta - raw[2]))
        c, s = math.cos(psi), math.sin(psi)
        dx, dy = x - raw[0], y - raw[1]
        fwd, left = c * dx + s * dy, s * dx - c * dy
        self._xy = (self._xy[0] + c * fwd - s * left, self._xy[1] + s * fwd + c * left)
        return self._xy


def landmark_reference(pose: Pose, goal: tuple[float, float, float], arrived_before: bool,
                       wz_i: float = 0.0) -> LandmarkCommand:
    gx, gy, gth = goal
    th = pose.theta
    dx_world = gx - pose.x
    dy_world = gy - pose.y
    dist = math.hypot(dx_world, dy_world)
    heading_err = wrap_to_pi(gth - th)
    tol_pos = LANDMARK_RELEASE_M if arrived_before else LANDMARK_TOL_M
    tol_th = THETA_RELEASE_RAD if arrived_before else THETA_TOL_RAD
    if dist <= tol_pos and abs(heading_err) <= tol_th:
        return LandmarkCommand(0.0, 0.0, 0.0, [0.0] * 4, dx_world, dy_world, dist,
                               heading_err, True)
    ex_body, ey_body = world_to_body(dx_world, dy_world, th)
    vx_body = KP_LANDMARK * ex_body
    vy_body = KP_LANDMARK * ey_body
    speed = math.hypot(vx_body, vy_body)
    if speed > V_MAX_LANDMARK_MPS:
        k = V_MAX_LANDMARK_MPS / speed
        vx_body, vy_body = vx_body * k, vy_body * k
    elif dist > LANDMARK_TOL_M and 1e-9 < speed < V_MIN_LANDMARK_MPS:
        k = V_MIN_LANDMARK_MPS / speed
        vx_body, vy_body = vx_body * k, vy_body * k
    if abs(vx_body) < MOTION_DEADBAND_MPS:
        vx_body = 0.0
    if abs(vy_body) < MOTION_DEADBAND_MPS:
        vy_body = 0.0
    wz = heading_rate(heading_err, wz_i)
    vx_body, vy_body, wz, wheels = body_to_wheels(vx_body, vy_body, wz)
    return LandmarkCommand(vx_body, vy_body, wz, wheels, dx_world, dy_world, dist,
                           heading_err, False)


@dataclass
class ControlOutput:
    wheels: list[float] = field(default_factory=lambda: [0.0] * 4)
    vx: float = 0.0
    vy: float = 0.0
    reason: str = ""
    measurement: Optional[CameraMeasurement] = None
    dist: Optional[float] = None
    arrived: bool = False
    wz: float = 0.0
    err_world: Optional[tuple[float, float]] = None
    heading_err: Optional[float] = None
    pose: Optional[Pose] = None
    goal: Optional[tuple[float, float, float]] = None
    hard_stop: bool = False


@dataclass(frozen=True)
class RampLimits:
    accel: float
    decel: float
    alpha: float


LANDMARK_RAMP = RampLimits(LANDMARK_ACCEL_MPS2, LANDMARK_DECEL_MPS2, LANDMARK_ALPHA_RAD_S2)
CAMERA_RAMP = RampLimits(CAMERA_ACCEL_MPS2, CAMERA_DECEL_MPS2, CAMERA_ALPHA_RAD_S2)


class SlewLimiter:
    def __init__(self) -> None:
        self.vx = self.vy = self.wz = 0.0

    def reset(self) -> None:
        self.vx = self.vy = self.wz = 0.0

    def apply(self, cmd: ControlOutput, dt: float, lim: RampLimits) -> ControlOutput:
        if cmd.hard_stop:
            self.reset()
            return dataclasses.replace(cmd, vx=0.0, vy=0.0, wz=0.0, wheels=[0.0] * 4)
        dt = clamp(dt, 0.0, 0.2)
        vx, vy, wz = cmd.vx, cmd.vy, cmd.wz
        dvx, dvy = vx - self.vx, vy - self.vy
        step = math.hypot(dvx, dvy)
        a = lim.accel if math.hypot(vx, vy) >= math.hypot(self.vx, self.vy) else lim.decel
        cap = a * dt
        if step > cap:
            vx, vy = self.vx + dvx * cap / step, self.vy + dvy * cap / step
        cap_w = lim.alpha * dt
        if abs(wz - self.wz) > cap_w:
            wz = self.wz + math.copysign(cap_w, wz - self.wz)
        vx, vy, wz, wheels = body_to_wheels(vx, vy, wz)
        self.vx, self.vy, self.wz = vx, vy, wz
        return dataclasses.replace(cmd, vx=vx, vy=vy, wz=wz, wheels=wheels)


class FollowController:
    def __init__(self, ref_width_px: Optional[float]) -> None:
        self.ref_width_px = ref_width_px
        self.reset()

    def reset(self) -> None:
        self._arrived = False
        self._prev_t: Optional[float] = None
        self._wz_i = 0.0
        self._d_min = math.inf
        self._h_min = math.inf
        self.fault = ""

    def step(self, mode: Optional[str], landmark: Optional[str],
             vision: Optional[VisionOutput], pose: Optional[Pose], now: float) -> ControlOutput:
        if mode == CAMERA:
            m = camera_measurement(vision, self.ref_width_px)
            vx, vy, wz, wheels = proportional_reference(m)
            stop = vision is not None and vision.state == vt.STOP
            reason = ("STOP" if stop else "" if m.valid and m.distance_calibrated else
                      "sin objetivo" if not m.valid else "solo lateral: sin W_ref")
            return ControlOutput(wheels, vx, vy, reason, measurement=m, wz=wz, hard_stop=stop)

        if mode == LANDMARK:
            if landmark not in LANDMARKS:
                return ControlOutput(reason="sin landmark")
            goal = LANDMARKS[landmark]
            if pose is None or now - pose.t > POSE_TIMEOUT_S:
                self._prev_t = None
                return ControlOutput(reason="sin pose", pose=pose, goal=goal)
            if self.fault:
                return ControlOutput(reason=self.fault, pose=pose, goal=goal, hard_stop=True)
            dt = min(0.2, now - self._prev_t) if self._prev_t is not None else 1.0 / CONTROL_HZ
            self._prev_t = now
            err = wrap_to_pi(goal[2] - pose.theta)
            if not self._arrived and abs(err) >= HEADING_DEADBAND_RAD:
                self._wz_i = clamp(self._wz_i + KI_THETA * err * dt,
                                   -WZ_I_MAX_RAD_S, WZ_I_MAX_RAD_S)
            cmd = landmark_reference(pose, goal, self._arrived, self._wz_i)
            if cmd.arrived:
                self._wz_i = 0.0
            self.fault = self._divergence(landmark, cmd)
            if self.fault:
                self._arrived = False
                return ControlOutput(reason=self.fault, dist=cmd.dist,
                                     err_world=(cmd.dx_world, cmd.dy_world),
                                     heading_err=cmd.heading_err, pose=pose, goal=goal,
                                     hard_stop=True)
            if cmd.arrived and not self._arrived:
                LOG.info("Landmark %s alcanzado (error %.3f m, orientación %+.3f rad)",
                         landmark, cmd.dist, cmd.heading_err)
            elif self._arrived and not cmd.arrived:
                LOG.info("Landmark %s: se alejó %.3f m / %+.3f rad -> corrigiendo de regreso",
                         landmark, cmd.dist, cmd.heading_err)
            self._arrived = cmd.arrived
            return ControlOutput(cmd.wheels, cmd.vx, cmd.vy, "en destino" if cmd.arrived else "",
                                 dist=cmd.dist, arrived=cmd.arrived, wz=cmd.wz,
                                 err_world=(cmd.dx_world, cmd.dy_world),
                                 heading_err=cmd.heading_err, pose=pose, goal=goal)

        return ControlOutput(reason="sin modo")

    def _divergence(self, landmark: str, cmd: LandmarkCommand) -> str:
        h = abs(cmd.heading_err)
        if cmd.arrived:
            self._d_min, self._h_min = cmd.dist, h
            return ""
        self._d_min = min(self._d_min, cmd.dist)
        self._h_min = min(self._h_min, h)
        if LANDMARK_DIVERGE_M > 0 and cmd.dist > self._d_min + LANDMARK_DIVERGE_M:
            LOG.error("LANDMARK %s: la distancia al destino creció de %.2f a %.2f m -> ruedas en 0. "
                      "El movimiento real y la odometría no coinciden en signo: si el rover fue "
                      "físicamente hacia el destino, cambia INVERT_ODOM_VY a %s; si fue al lado "
                      "contrario, cambia INVERT_VY a %s (o INVERT_VX). Envía un MODE para "
                      "reintentar", landmark, self._d_min, cmd.dist, not INVERT_ODOM_VY,
                      not INVERT_VY)
            return "DIVERGE posicion"
        if HEADING_DIVERGE_RAD > 0 and h > self._h_min + HEADING_DIVERGE_RAD:
            LOG.error("LANDMARK %s: |error de theta| creció de %.2f a %.2f rad -> ruedas en 0. "
                      "+wz no hace crecer theta: cambia INVERT_WZ a %s (verifícalo con "
                      "--kin-test). Envía un MODE para reintentar", landmark, self._h_min, h,
                      not INVERT_WZ)
            return "DIVERGE theta"
        return ""


class KinematicTest:
    STEPS = (("vx", (KIN_TEST_V_MPS, 0.0, 0.0)), ("vy", (0.0, KIN_TEST_V_MPS, 0.0)),
             ("wz", (0.0, 0.0, KIN_TEST_WZ_RAD_S)), ("vx_girado", (KIN_TEST_V_MPS, 0.0, 0.0)),
             ("wz_regreso", (0.0, 0.0, -KIN_TEST_WZ_RAD_S)))
    SETTLE_S, DRIVE_S = 1.2, 1.5

    def __init__(self) -> None:
        self.i, self.stage, self.t0 = 0, "settle", None
        self.p0: Optional[Pose] = None
        self.done = False
        self.meas: dict[str, tuple[Pose, Pose]] = {}
        self.report: list[str] = []

    def step(self, pose: Optional[Pose], now: float) -> ControlOutput:
        if self.done:
            return ControlOutput(reason="kin-test terminado (envía un MODE para seguir)", pose=pose)
        if pose is None or now - pose.t > POSE_TIMEOUT_S:
            if self.t0 is not None:
                LOG.error("kin-test abortado: no llega la pose de la ESP32")
                self.done = True
            return ControlOutput(reason="kin-test: esperando pose")
        if self.t0 is None:
            self.t0 = now
            LOG.info("kin-test: %s durante %.1f s cada uno (deja ~0.3 m libres o rover suspendido)",
                     ", ".join(n for n, _ in self.STEPS), self.DRIVE_S)
        name, (vx, vy, wz) = self.STEPS[self.i]
        el = now - self.t0
        if self.stage == "settle":
            if el >= self.SETTLE_S:
                self.p0, self.stage, self.t0 = pose, "drive", now
            return ControlOutput(reason=f"kin-test {name}: preparando", pose=pose)
        if self.stage == "drive":
            if el >= self.DRIVE_S:
                self.stage, self.t0 = "stop", now
                return ControlOutput(reason=f"kin-test {name}: frenando", pose=pose)
            bvx, bvy, bwz, wheels = body_to_wheels(vx, vy, wz)
            return ControlOutput(wheels, bvx, bvy, f"kin-test {name}", wz=bwz, pose=pose)
        if el >= self.SETTLE_S:
            self.meas[name] = (self.p0, pose)
            self._log_step(name, self.p0, pose)
            self.i += 1
            self.stage, self.t0 = "settle", now
            if self.i >= len(self.STEPS):
                self.done = True
                self._evaluate()
        return ControlOutput(reason=f"kin-test {name}: frenando", pose=pose)

    @staticmethod
    def _disp(p0: Pose, p1: Pose) -> tuple[float, float, float, float]:
        dX, dY = p1.x - p0.x, p1.y - p0.y
        return dX, dY, math.hypot(dX, dY), wrap_to_pi(p1.theta - p0.theta)

    def _log_step(self, name: str, p0: Pose, p1: Pose) -> None:
        dX, dY, d, dth = self._disp(p0, p1)
        ang = f"{math.degrees(math.atan2(dY, dX)):+.0f}°" if d > 0.02 else "-"
        LOG.info("kin-test %-10s dX=%+.3f dY=%+.3f m (dirección en el mapa %s) dtheta=%+.3f rad",
                 name, dX, dY, ang, dth)

    def _evaluate(self) -> None:
        rec = {"ODOM_HEADING_OFFSET_RAD": ODOM_HEADING_OFFSET_RAD, "INVERT_THETA": INVERT_THETA,
               "INVERT_VX": INVERT_VX, "INVERT_VY": INVERT_VY, "INVERT_WZ": INVERT_WZ,
               "INVERT_ODOM_VY": INVERT_ODOM_VY}
        notes: list[str] = []
        p0x, p1x = self.meas["vx"]
        dX, dY, dx_len, _ = self._disp(p0x, p1x)
        _, _, _, dth_w = self._disp(*self.meas["wz"])
        if dth_w > 0.15:
            notes.append(f"+wz: theta {dth_w:+.2f} rad -> OK")
        elif dth_w < -0.15:
            rec["INVERT_WZ"] = not INVERT_WZ
            notes.append(f"+wz: theta {dth_w:+.2f} rad -> INVERTIDO")
        else:
            notes.append(f"+wz: theta {dth_w:+.2f} rad -> NO GIRÓ: revisa WZ_WHEEL_SIGNS / "
                         "orden de w1..w4 / que la ESP32 actualice theta")
        if abs(self.meas["wz"][1].theta - self.meas["wz"][0].theta) > 2 * math.pi + 0.5 \
                and not THETA_IN_DEGREES:
            notes.append("theta parece venir en GRADOS: usa THETA_IN_DEGREES = True")
        if dx_len < 0.03:
            notes.append(f"+vx: se movió solo {dx_len:.3f} m -> revisa motores / odometría")
            self._print(rec, notes, ok=False)
            return
        phi_x = math.atan2(dY, dX)
        sigma = -1.0 if INVERT_THETA else 1.0
        p0g, p1g = self.meas["vx_girado"]
        gX, gY, g_len, _ = self._disp(p0g, p1g)
        if abs(dth_w) > 0.15 and g_len > 0.03:
            turned = wrap_to_pi(math.atan2(gY, gX) - phi_x)
            dth_odom = wrap_to_pi(p0g.theta - p0x.theta)
            sigma = 1.0 if turned * dth_odom > 0 else -1.0
            rec["INVERT_THETA"] = sigma < 0
            notes.append(f"giro en el mapa {turned:+.2f} rad vs theta {dth_odom:+.2f} rad -> "
                         + ("mismo sentido (INVERT_THETA = False)" if sigma > 0
                            else "sentido contrario (INVERT_THETA = True)"))
        off = wrap_to_pi(phi_x - sigma * p0x.theta)
        q = round(off / (math.pi / 2)) * (math.pi / 2)
        if abs(off - q) < math.radians(15):
            off = wrap_to_pi(q) if abs(wrap_to_pi(q)) > 1e-9 else 0.0
        else:
            notes.append(f"+vx salió a {math.degrees(off):+.0f}° (no múltiplo de 90°): "
                         "revisa el orden de las ruedas w1..w4")
        rec["ODOM_HEADING_OFFSET_RAD"] = off
        notes.append(f"+vx: adelante = {math.degrees(phi_x):+.0f}° en el mapa con theta = "
                     f"{p0x.theta:+.2f} -> ODOM_HEADING_OFFSET_RAD = {off:+.4f} "
                     f"({math.degrees(off):+.0f}°)")
        yX, yY, y_len, _ = self._disp(*self.meas["vy"])
        if y_len < 0.03:
            notes.append("+vy: no se movió lateralmente -> revisa el orden de las ruedas w1..w4")
        else:
            rel = math.degrees(wrap_to_pi(math.atan2(yY, yX) - phi_x))
            if abs(rel - 90) < 30:
                notes.append(f"+vy: {rel:+.0f}° respecto a adelante (izquierda) -> OK")
            elif abs(rel + 90) < 30:
                rec["INVERT_ODOM_VY"] = not INVERT_ODOM_VY
                notes.append(f"+vy: {rel:+.0f}° respecto a adelante (derecha según la odometría) "
                             "-> la odometría no coincide con el comando. Si en el piso el rover "
                             f"fue a SU izquierda, INVERT_ODOM_VY = {not INVERT_ODOM_VY}; si fue a "
                             f"su derecha, deja INVERT_ODOM_VY = {INVERT_ODOM_VY} y usa "
                             f"INVERT_VY = {not INVERT_VY}")
            else:
                notes.append(f"+vy: {rel:+.0f}° respecto a adelante -> revisa el orden de w1..w4")
        self._print(rec, notes, ok=True)

    def _print(self, rec: dict, notes: list[str], ok: bool) -> None:
        cur = {"ODOM_HEADING_OFFSET_RAD": ODOM_HEADING_OFFSET_RAD, "INVERT_THETA": INVERT_THETA,
               "INVERT_VX": INVERT_VX, "INVERT_VY": INVERT_VY, "INVERT_WZ": INVERT_WZ,
               "INVERT_ODOM_VY": INVERT_ODOM_VY}
        same = all((abs(rec[k] - cur[k]) < 1e-3) if isinstance(cur[k], float) else rec[k] == cur[k]
                   for k in cur)
        lines = ["kin-test terminado:"] + [f"  {n}" for n in notes]
        if ok:
            lines.append("  Valores para config.py" + ("  (= los actuales: OK)" if same else
                                                       "  (CAMBIAR):"))
            lines += [f"    {k} = {v:+.4f}" if isinstance(v, float) else f"    {k} = {v}"
                      for k, v in rec.items()]
        self.report = lines
        LOG.info("%s", "\n".join(lines))


def read_tag_calibration(path: Optional[str] = None) -> Optional[float]:
    cal_path = Path(path or TAG_CALIBRATION_FILE)
    try:
        obj = json.loads(cal_path.read_text(encoding="utf-8"))
        width = float(obj["tag_width_at_reference_px"])
        if obj.get("reference_distance_m") != DISTANCE_REF_M:
            return None
        if not math.isfinite(width) or width < TAG_MIN_SIDE_PX:
            return None
        return width
    except (OSError, ValueError, KeyError, TypeError):
        return None


def write_tag_calibration(width: float, path: Optional[str] = None) -> float:
    width = float(width)
    if not math.isfinite(width) or width < TAG_MIN_SIDE_PX:
        raise ValueError("tag demasiado pequeño o ancho inválido")
    cal_path = Path(path or TAG_CALIBRATION_FILE)
    cal_path.write_text(json.dumps({
        "reference_distance_m": DISTANCE_REF_M,
        "tag_width_at_reference_px": round(width, 3),
    }, indent=2) + "\n", encoding="utf-8")
    return width


def _valid_width(w: Optional[float]) -> bool:
    return w is not None and math.isfinite(w) and w >= TAG_MIN_SIDE_PX


def resolve_ref_width(cli_width: Optional[float]) -> tuple[Optional[float], str]:
    if _valid_width(cli_width):
        return float(cli_width), "--ref-width"
    width = read_tag_calibration()
    if width is not None:
        return width, "archivo"
    if _valid_width(DEFAULT_TAG_REF_WIDTH_PX):
        return float(DEFAULT_TAG_REF_WIDTH_PX), "default"
    return None, "ninguno"


def run_calibration(vcfg: vt.Config, samples: int = 45, timeout_s: float = 60.0) -> int:
    LOG.info("Calibración: coloca el tag a %.2f m de la cámara, centrado y de frente",
             DISTANCE_REF_M)
    widths: list[float] = []
    t_end = time.perf_counter() + timeout_s
    with VisionTracker(vcfg) as vision:
        while len(widths) < samples and time.perf_counter() < t_end:
            out = vision.read(timeout=0.5)
            if out is None:
                continue
            if out.state == vt.TRACK and out.result is not None and out.result.source == "TAG":
                widths.append(out.width_px)
            if vision.show(out, [f"CALIBRACION a {DISTANCE_REF_M:.2f} m: "
                                 f"{len(widths)}/{samples} muestras"]) in (ord("q"), 27):
                break
    if len(widths) < samples // 2:
        LOG.error("Calibración incompleta (%d muestras): no se guardó nada", len(widths))
        return 1
    widths.sort()
    w = write_tag_calibration(widths[len(widths) // 2])
    LOG.info("W_ref = %.2f px a %.2f m guardado en %s (dispersión %.1f..%.1f px)",
             w, DISTANCE_REF_M, TAG_CALIBRATION_FILE, widths[0], widths[-1])
    return 0


def frame_ready(mode: Optional[str], landmark: Optional[str], img_size: tuple[int, int],
                motion: bool) -> dict:
    return {"type": "READY", "proto": PROTOCOL_NAME, "mode": mode or NONE,
            "landmark": landmark or NONE, "img_w": int(img_size[0]),
            "img_h": int(img_size[1]), "motion": bool(motion)}


def frame_telemetry(seq: int, mode: Optional[str], vision: Optional[VisionOutput],
                    wheels: list[float], landmark: Optional[str], arrived: bool) -> dict:
    tracking = bool(vision is not None and vision.is_tracking and vision.tag_x is not None)
    return {
        "type": "TEL",
        "seq": seq & 0xFFFFFFFF,
        "mode": mode or NONE,
        "tracking": tracking,
        "target": vision.target_name if tracking else NONE,
        "tag_x": int(round(vision.tag_x)) if tracking else -1,
        "tag_y": int(round(vision.tag_y)) if tracking else -1,
        "wheels": [round(_finite(w), 3) for w in wheels],
        "state": vision.state if vision is not None else "NO_CAMERA",
        "width_px": round(vision.width_px, 1) if tracking else 0.0,
        "landmark": landmark or NONE,
        "arrived": bool(arrived),
    }


def frame_error(msg: str) -> dict:
    return {"type": "ERROR", "msg": msg[:120]}


def encode(obj: dict) -> bytes:
    return (json.dumps(obj, separators=(",", ":"), ensure_ascii=True,
                       allow_nan=False) + "\n").encode("ascii")


@dataclass
class LinkStats:
    port: str = "-"
    connected: bool = False
    rx_ok: int = 0
    rx_bad: int = 0
    tx_ok: int = 0
    tx_drop: int = 0
    last_rx: float = 0.0


class Esp32Link:
    def __init__(self, port: str, baud: int, ready_info: Optional[dict] = None) -> None:
        if serial is None:
            raise RuntimeError("Falta pyserial: sudo apt install python3-serial")
        self.port_cfg, self.baud = port, baud
        self.ready_info = ready_info or {"img_size": (0, 0), "motion": True}
        self.stats = LinkStats()
        self._ser: Optional["serial.Serial"] = None
        self._wlock = threading.Lock()
        self._plock = threading.Lock()
        self._pose: Optional[Pose] = None
        self._odom_fix = LateralOdomFix()
        self._events: deque = deque(maxlen=64)
        self._rxbuf = bytearray()
        self._broken = False
        self._theta_warned = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="uart", daemon=True)
        self._t_start = time.monotonic()
        self._fails = 0
        self._last_warn = -math.inf

    def start(self) -> "Esp32Link":
        self._t_start = time.monotonic()
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        with self._wlock:
            self._close_locked()

    def send(self, obj: dict) -> bool:
        data = encode(obj)
        with self._wlock:
            ser = self._ser
            if ser is None:
                return False
            try:
                ser.write(data)
                self.stats.tx_ok += 1
                return True
            except SerialTimeoutException:
                self.stats.tx_drop += 1
                return False
            except (SerialException, OSError) as e:
                LOG.warning("UART: error de escritura (%s)", e)
                self._broken = True
                return False

    def poll_events(self) -> list[tuple]:
        out = []
        while self._events:
            out.append(self._events.popleft())
        return out

    @property
    def pose(self) -> Optional[Pose]:
        with self._plock:
            return self._pose

    def _resolve_port(self) -> Optional[str]:
        if self.port_cfg.lower() != "auto":
            return self.port_cfg
        usb_only = time.monotonic() - self._t_start < SERIAL_USB_WAIT_S
        for pattern in SERIAL_USB_CANDIDATES if usb_only else SERIAL_CANDIDATES:
            found = sorted(glob.glob(pattern))
            if found:
                return found[0]
        return None

    def _open_failed(self, msg: str, *args) -> None:
        self._fails += 1
        now = time.monotonic()
        if self._fails == 1 or now - self._last_warn >= SERIAL_WARN_EVERY_S:
            self._last_warn = now
            LOG.warning(msg + " (intento %d, reintentando)", *args, self._fails)

    def _open(self) -> bool:
        port = self._resolve_port()
        if port is None:
            usb_only = time.monotonic() - self._t_start < SERIAL_USB_WAIT_S
            self._open_failed("UART: no se encontró puerto (%s)", ", ".join(
                SERIAL_USB_CANDIDATES if usb_only else SERIAL_CANDIDATES))
            return False
        try:
            ser = serial.Serial()
            ser.port, ser.baudrate = port, self.baud
            ser.bytesize, ser.parity, ser.stopbits = serial.EIGHTBITS, serial.PARITY_NONE, \
                serial.STOPBITS_ONE
            ser.timeout, ser.write_timeout = 0.05, 0.05
            ser.dtr = ser.rts = False
            ser.open()
            ser.reset_input_buffer()
        except (SerialException, OSError, ValueError) as e:
            self._open_failed("UART: no se pudo abrir %s: %s", port, e)
            return False
        with self._wlock:
            self._ser, self._broken = ser, False
        self._rxbuf.clear()
        self.stats.port, self.stats.connected = port, True
        LOG.info("UART abierto: %s @ %d baud%s", port, self.baud,
                 f" (tras {self._fails + 1} intentos)" if self._fails else "")
        self._fails = 0
        self._events.append(("LINK_UP",))
        return True

    def _close_locked(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except (SerialException, OSError):
                pass
        self._ser = None
        self.stats.connected = False

    def _drop(self, why: str) -> None:
        LOG.error("UART: %s; reconectando...", why)
        with self._wlock:
            self._close_locked()
        with self._plock:
            self._pose = None
            self._odom_fix.reset()
        self._events.append(("LINK_DOWN",))

    def _run(self) -> None:
        backoff = 0.5
        while not self._stop.is_set():
            if self._ser is None:
                if not self._open():
                    self._stop.wait(backoff)
                    backoff = min(backoff * 2.0, SERIAL_RETRY_MAX_S)
                    continue
                backoff = 0.5
            if self._broken:
                self._drop("fallo de escritura")
                continue
            try:
                ser = self._ser
                data = ser.read(ser.in_waiting or 1)
            except (SerialException, OSError, TypeError, AttributeError) as e:
                self._drop(str(e) or type(e).__name__)
                continue
            if data:
                self._feed(data)

    def _feed(self, data: bytes) -> None:
        buf = self._rxbuf
        buf += data
        while True:
            i = buf.find(b"\n")
            if i < 0:
                if len(buf) > RX_MAX_LINE:
                    buf.clear()
                    self.stats.rx_bad += 1
                return
            line = bytes(buf[:i]).strip()
            del buf[:i + 1]
            if line:
                try:
                    self._handle_line(line.decode("ascii", "replace"))
                except Exception:
                    self.stats.rx_bad += 1
                    LOG.exception("UART: error procesando %r", line[:80])

    def _handle_line(self, line: str) -> None:
        if line.startswith("{"):
            try:
                msg = json.loads(line)
            except ValueError:
                self.stats.rx_bad += 1
                LOG.debug("UART: JSON inválido: %r", line[:80])
                return
            if not isinstance(msg, dict):
                self.stats.rx_bad += 1
                return
        else:
            tok = line.split()
            msg = {"type": tok[0]}
        self.stats.rx_ok += 1
        self.stats.last_rx = time.perf_counter()
        self._dispatch(msg)

    def _dispatch(self, msg: dict) -> None:
        typ = str(msg.get("type", "")).strip().upper()
        if all(k in msg for k in ("x", "y", "theta")):
            self._update_pose(msg)
        if typ == "MODE" or ("mode" in msg and typ in ("START", "READY")):
            self._handle_mode(msg)
        if typ in ("START", "READY"):
            self._events.append(("START",))
        elif typ == "IDLE":
            self._events.append(("IDLE",))
        elif typ not in ("MODE", "POSE"):
            LOG.debug("UART: mensaje desconocido %r", msg)

    def _update_pose(self, msg: dict) -> None:
        try:
            x, y, th = float(msg["x"]), float(msg["y"]), float(msg["theta"])
        except (TypeError, ValueError):
            self.stats.rx_bad += 1
            return
        if not all(map(math.isfinite, (x, y, th))):
            self.stats.rx_bad += 1
            return
        if THETA_IN_DEGREES:
            th = math.radians(th)
        elif abs(th) > 2 * math.pi + 0.3 and not self._theta_warned:
            self._theta_warned = True
            LOG.warning("theta = %.2f fuera de [-2pi, 2pi]: ¿la ESP32 manda GRADOS? "
                        "(usa THETA_IN_DEGREES = True o --theta-deg)", th)
        with self._plock:
            if INVERT_ODOM_VY:
                x, y = self._odom_fix.apply(x, y, th)
            self._pose = Pose(x, y, th, time.perf_counter())

    def _handle_mode(self, msg: dict) -> None:
        mode = str(msg.get("mode", "")).strip().upper()
        if mode not in MODES:
            self.send(frame_error(f"modo desconocido: {mode or '-'}"))
            return
        lm = msg.get("landmark")
        lm = None if lm is None else str(lm).strip().upper()
        if lm in ("", NONE) or mode == CAMERA:
            lm = None
        if lm is not None and lm not in LANDMARKS:
            self.send(frame_error(f"landmark desconocido: {lm}"))
            return
        self.send(frame_ready(mode, lm, self.ready_info["img_size"], self.ready_info["motion"]))
        LOG.info("ESP32 -> MODE %s%s  (READY enviado)", mode, f" {lm}" if lm else "")
        self._events.append(("MODE", mode, lm))


class RoverSim:
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0,
                 offset: Optional[float] = None, flip_vy: bool = False, flip_wz: bool = False,
                 flip_theta: bool = False, yaw_drift: float = 0.0) -> None:
        self.off = ODOM_HEADING_OFFSET_RAD if offset is None else offset
        self.sth = -1.0 if (INVERT_THETA != flip_theta) else 1.0
        self.vy_sign = -1.0 if flip_vy else 1.0
        self.wz_sign = -1.0 if flip_wz else 1.0
        self.yaw_drift = yaw_drift
        self.x, self.y = x, y
        self.psi = self.sth * theta + self.off
        self.sx = -1.0 if INVERT_VX else 1.0
        self.sy = -1.0 if INVERT_VY else 1.0
        self.sz = -1.0 if INVERT_WZ else 1.0

    @property
    def theta(self) -> float:
        return wrap_to_pi(self.sth * (self.psi - self.off))

    def step(self, wheels: list[float], dt: float) -> None:
        r = WHEEL_RADIUS_M
        k = MECANUM_LX_M + MECANUM_LY_M
        w1, w2, w3, w4 = wheels
        fwd = self.sx * r * (w1 + w2 + w3 + w4) / 4.0
        right = self.sy * self.vy_sign * r * (w1 - w2 - w3 + w4) / 4.0
        wz = self.sz * self.wz_sign * r * sum(sg * w for sg, w in zip(WZ_WHEEL_SIGNS, wheels)) / (4.0 * k)
        c, s = math.cos(self.psi), math.sin(self.psi)
        self.x += (fwd * c + right * s) * dt
        self.y += (fwd * s - right * c) * dt
        drift = self.yaw_drift if any(abs(w) > 1e-6 for w in wheels) else 0.0
        self.psi += self.sth * (wz + drift) * dt

    @property
    def pose(self) -> Pose:
        return Pose(self.x, self.y, self.theta, time.perf_counter())


class FollowerApp:
    def __init__(self, a: argparse.Namespace, vcfg: vt.Config) -> None:
        self.a = a
        self.vcfg = vcfg
        self.motion = MOTION_ENABLED and not a.dry_run
        ref, self.ref_source = resolve_ref_width(a.ref_width)
        self.ctrl = FollowController(ref)
        self.ramp = SlewLimiter()
        self.vision: Optional[VisionTracker] = None if a.no_camera else VisionTracker(vcfg)
        self.link: Optional[Esp32Link] = None
        self.sim = (RoverSim(*a.sim_start, offset=None if a.sim_offset_deg is None else
                             math.radians(a.sim_offset_deg), flip_vy=a.sim_flip_vy,
                             flip_wz=a.sim_flip_wz, flip_theta=a.sim_flip_theta,
                             yaw_drift=a.sim_yaw_drift) if a.sim else None)
        self.kin: Optional[KinematicTest] = KinematicTest() if a.kin_test else None
        self.mode: Optional[str] = None
        self.landmark: Optional[str] = None
        self.mode_source = "-"
        self.started = a.no_wait_start
        self.phase = WAIT_MODE
        self.seq = 0
        self.last_out: Optional[VisionOutput] = None
        self.last_cmd = ControlOutput(reason="sin modo")
        self._last_ready = -math.inf
        self._vision_dead_logged = False
        if ref is None:
            LOG.warning("Sin W_ref válido: en CAMERA solo se corrige el error lateral (vx = 0)")
        elif self.ref_source == "default":
            LOG.warning("Sin tag_calibration.json: W_ref por defecto = %.1f px a %.2f m "
                        "(ejecuta --calibrate para ajustarlo a tu tag)", ref, DISTANCE_REF_M)
        else:
            LOG.info("W_ref = %.1f px a %.2f m (%s)", ref, DISTANCE_REF_M, self.ref_source)
        if not self.motion:
            LOG.warning("Movimiento deshabilitado (--dry-run): se envían ruedas en 0")
        LOG.info("Signos: INVERT_VX=%s INVERT_VY=%s INVERT_WZ=%s INVERT_THETA=%s "
                 "INVERT_ODOM_VY=%s ODOM_HEADING_OFFSET=%+.1f° WZ_WHEEL_SIGNS=%s LX+LY=%.3f m "
                 "theta en %s", INVERT_VX, INVERT_VY, INVERT_WZ, INVERT_THETA, INVERT_ODOM_VY,
                 math.degrees(ODOM_HEADING_OFFSET_RAD), WZ_WHEEL_SIGNS,
                 MECANUM_LX_M + MECANUM_LY_M, "grados" if THETA_IN_DEGREES else "rad")
        LOG.info("LANDMARK: KP=%.2f vmax=%.2f m/s rampa +%.2f/-%.2f m/s² y %.2f rad/s² | "
                 "KP_THETA=%.2f KI_THETA=%.2f banda=%.3f rad | llegada %.2f m / %.2f rad, "
                 "re-corrige %.2f m / %.2f rad", KP_LANDMARK, V_MAX_LANDMARK_MPS,
                 LANDMARK_ACCEL_MPS2, LANDMARK_DECEL_MPS2, LANDMARK_ALPHA_RAD_S2, KP_THETA,
                 KI_THETA, HEADING_DEADBAND_RAD, LANDMARK_TOL_M, THETA_TOL_RAD,
                 LANDMARK_RELEASE_M, THETA_RELEASE_RAD)
        LOG.info("CAMERA: KP_LATERAL=%.2f KP_CAMERA_WZ=%.2f INVERT_CAMERA_VY=%s "
                 "INVERT_CAMERA_WZ=%s rampa +%.2f/-%.2f m/s² y %.2f rad/s²", KP_LATERAL,
                 KP_CAMERA_WZ, INVERT_CAMERA_VY, INVERT_CAMERA_WZ, CAMERA_ACCEL_MPS2,
                 CAMERA_DECEL_MPS2, CAMERA_ALPHA_RAD_S2)
        if self.kin is not None:
            LOG.warning("--kin-test: al entrar en RUN el rover se moverá ~12 cm adelante, ~12 cm "
                        "a la izquierda y girará ~50°; luego ruedas en 0 hasta un MODE nuevo")

    def run(self) -> int:
        a = self.a
        try:
            if self.vision is not None:
                self.vision.start()
            img = self.vision.frame_size if self.vision else (self.vcfg.width, self.vcfg.height)
            if a.port.lower() != "none":
                self.link = Esp32Link(a.port, a.baud, {"img_size": img, "motion": self.motion})
                self.link.start()
            if a.mode:
                self._apply_mode(a.mode.upper(), a.landmark, "local")
                self.started = True
            self._loop()
        except KeyboardInterrupt:
            LOG.info("Interrumpido por el usuario")
        except RuntimeError as e:
            LOG.error("%s", e)
            return 1
        finally:
            self._shutdown()
        return 0

    def _shutdown(self) -> None:
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, signal.SIG_IGN)
            except ValueError:
                pass
        if self.link is not None:
            zero = frame_telemetry(self.seq + 1, self.mode, None, [0.0] * 4, self.landmark, False)
            for _ in range(3):
                self.link.send(zero)
                time.sleep(0.02)
            self.link.close()
        if self.vision is not None:
            self.vision.close()

    def _loop(self) -> None:
        period = 1.0 / max(1.0, self.a.control_hz)
        last_ctrl = -math.inf
        last_print = -math.inf
        last_lm_print = -math.inf
        while True:
            out = self._read_vision(period)
            now = time.perf_counter()
            if self.link is not None:
                for ev in self.link.poll_events():
                    self._on_event(ev)
            self._update_phase(now)

            if now - last_ctrl >= 0.8 * period:
                dt = min(0.2, now - last_ctrl) if math.isfinite(last_ctrl) else period
                last_ctrl = now
                self._control_step(now, dt)

            key = self.vision.show(out, self._gui_lines()) if (out and self.vision) else -1
            if key >= 0 and not self._on_key(key):
                return
            if self.phase == RUN and (self.mode == LANDMARK or self._kin_active()):
                if now - last_lm_print >= LANDMARK_PRINT_S:
                    last_lm_print = now
                    print(self._landmark_debug_line(), flush=True)
            elif not (self.vision and self.vision.gui_enabled) and \
                    now - last_print >= self.a.print_interval:
                last_print = now
                print(self._status_line(), flush=True)

    def _read_vision(self, period: float) -> Optional[VisionOutput]:
        if self.vision is not None and self.vision.alive:
            out = self.vision.read(timeout=period)
            if out is not None:
                self.last_out = out
            return out
        if self.vision is not None and not self._vision_dead_logged:
            LOG.error("La captura de la cámara se detuvo: CAMERA queda en 0")
            self._vision_dead_logged = True
        time.sleep(period)
        return None

    def _fresh_vision(self, now: float) -> Optional[VisionOutput]:
        o = self.last_out
        return o if o is not None and now - o.timestamp <= VISION_STALE_S else None

    def _control_step(self, now: float, dt: float) -> None:
        pose = self.sim.pose if self.sim else (self.link.pose if self.link else None)
        vision = self._fresh_vision(now)
        if self.phase == RUN and self.kin is not None:
            cmd = self.ramp.apply(self.kin.step(pose, now), dt, LANDMARK_RAMP)
        elif self.phase == RUN:
            cmd = self.ctrl.step(self.mode, self.landmark, vision, pose, now)
            cmd = self.ramp.apply(cmd, dt, CAMERA_RAMP if self.mode == CAMERA else LANDMARK_RAMP)
        else:
            self.ramp.reset()
            cmd = ControlOutput(reason=self.phase.lower())
        sent = cmd.wheels if self.motion else [0.0] * 4
        self.last_cmd = cmd
        if self.phase == RUN:
            self.seq += 1
            lm = "KIN_TEST" if self.kin is not None else self.landmark
            tel = frame_telemetry(self.seq, self.mode, vision, sent, lm, cmd.arrived)
            if self.link is not None:
                self.link.send(tel)
            if self.a.print_tel:
                print(encode(tel).decode().rstrip(), flush=True)
        if self.sim is not None:
            self.sim.step(sent, dt)

    def _apply_mode(self, mode: str, landmark: Optional[str], source: str) -> None:
        if mode not in MODES:
            LOG.error("Modo desconocido: %s", mode)
            return
        lm = landmark.upper() if (landmark and mode == LANDMARK) else None
        if lm is not None and lm not in LANDMARKS:
            LOG.error("Landmark desconocido: %s (válidos: %s)", lm, ", ".join(LANDMARKS))
            lm = None
        if self.kin is not None and self.kin.done:
            LOG.info("kin-test terminado: MODE nuevo -> control normal")
            self.kin = None
        if (mode, lm) != (self.mode, self.landmark) or self.ctrl.fault:
            LOG.info("Modo %s%s (%s)", mode, (f" -> {lm} (x={LANDMARKS[lm][0]:+.2f}, "
                     f"y={LANDMARKS[lm][1]:+.2f}, theta={LANDMARKS[lm][2]:+.2f})") if lm else "",
                     source)
            self.ctrl.reset()
        self.mode, self.landmark, self.mode_source = mode, lm, source
        if source != "ESP32" and self.link is not None:
            self.link.send(frame_ready(mode, lm, self.vision.frame_size if self.vision else
                                       (self.vcfg.width, self.vcfg.height), self.motion))

    def _idle(self, why: str) -> None:
        if self.mode is not None:
            LOG.info("IDLE (%s): ruedas en 0, esperando MODE", why)
            if self.phase == RUN and self.link is not None:
                self.seq += 1
                self.link.send(frame_telemetry(self.seq, self.mode, None, [0.0] * 4,
                                               self.landmark, False))
        self.mode = self.landmark = None
        self.started = self.a.no_wait_start
        self.ctrl.reset()
        if self.kin is not None and not self.kin.done:
            self.kin = KinematicTest()

    def _on_event(self, ev: tuple) -> None:
        kind = ev[0]
        if kind == "MODE":
            self._apply_mode(ev[1], ev[2], "ESP32")
        elif kind == "START":
            if not self.started:
                LOG.info("ESP32 -> START")
            self.started = True
        elif kind == "IDLE":
            self._idle("ESP32")
        elif kind == "LINK_DOWN" and self.mode_source == "ESP32":
            self._idle("enlace UART perdido")

    def _update_phase(self, now: float) -> None:
        phase = WAIT_MODE if self.mode is None else RUN if self.started else WAIT_START
        if phase != self.phase:
            LOG.info("Fase %s -> %s", self.phase, phase)
            self.phase = phase
        if phase == WAIT_START and self.link is not None and \
                now - self._last_ready >= READY_RESEND_S:
            self._last_ready = now
            self.link.send(frame_ready(self.mode, self.landmark, self.vision.frame_size
                                       if self.vision else (self.vcfg.width, self.vcfg.height),
                                       self.motion))

    def _on_key(self, key: int) -> bool:
        if key in (ord("q"), 27):
            return False
        if key == ord("c"):
            self._apply_mode(CAMERA, None, "teclado")
            self.started = True
        elif ord("1") <= key <= ord("5"):
            self._apply_mode(LANDMARK, LANDMARK_KEYS[key - ord("1")], "teclado")
            self.started = True
        elif key == ord(" "):
            self._idle("teclado")
        return True

    def _gui_lines(self) -> list[str]:
        c = self.last_cmd
        w = " ".join(f"{v:+.2f}" for v in c.wheels)
        lines = [f"{self.phase}  modo {self.mode or '-'}  ({self.mode_source})  "
                 f"{self._link_text()}",
                 f"w=[{w}] rad/s  vx={c.vx:+.2f} vy={c.vy:+.2f} wz={c.wz:+.2f}"
                 + ("" if self.motion else "  DRY-RUN") + (f"  ({c.reason})" if c.reason else "")]
        if self.mode == LANDMARK and self.landmark:
            gx, gy, gth = LANDMARKS[self.landmark]
            p = self.sim.pose if self.sim else (self.link.pose if self.link else None)
            ptxt = f"pose ({p.x:+.2f},{p.y:+.2f},{math.degrees(p.theta):+.0f}deg)" if p else "sin pose"
            d = f"d={c.dist:.2f}m" if c.dist is not None else ""
            lines.append(f"{'KIN_TEST' if self._kin_active() else self.landmark} "
                         f"({gx:+.1f},{gy:+.1f},{gth:+.1f}) {d} "
                         f"{'LLEGO' if c.arrived else ''}  {ptxt}")
        elif self.mode == CAMERA and c.measurement is not None:
            m = c.measurement
            ref = self.ctrl.ref_width_px
            lines.append(f"ex={m.ex:+.2f}  size_err={m.size_error:+.2f}  " +
                         (f"W_ref={ref:.1f}px ({self.ref_source})" if ref
                          else "SIN W_ref: solo lateral"))
        lines.append("c=CAMERA  1-5=LANDMARK  espacio=IDLE  q=salir")
        return lines

    def _kin_active(self) -> bool:
        return self.kin is not None

    def _landmark_debug_line(self) -> str:
        c = self.last_cmd
        name = "KIN_TEST" if self._kin_active() else (self.landmark or "-")
        head = f"[LANDMARK {name}]"
        p = c.pose
        if c.reason in ("sin pose", "kin-test: esperando pose") or p is None:
            age = f"última hace {time.perf_counter() - p.t:.1f} s" if p is not None else "nunca"
            return f"{head} SIN POSE de la ESP32 ({age}) -> wheels=[0, 0, 0, 0] | {self._link_text()}"
        ptxt = (f"pose=({p.x:+.3f}, {p.y:+.3f}, {p.theta:+.3f} rad = "
                f"{math.degrees(p.theta):+.1f}°)")
        if c.goal is not None:
            gx, gy, gth = c.goal
            gtxt = f"goal=({gx:+.2f}, {gy:+.2f}, {gth:+.2f})"
        else:
            gtxt = "goal=-"
        if c.err_world is not None:
            etxt = (f"err_world=({c.err_world[0]:+.3f}, {c.err_world[1]:+.3f}) "
                    f"d={c.dist:.3f} eth={c.heading_err:+.3f}")
        else:
            etxt = "err_world=-"
        wtxt = "wheels=[" + ", ".join(f"{v:+.3f}" for v in c.wheels) + "]"
        state = ("ARRIVED" if c.arrived else c.reason or "->") + ("" if self.motion else " DRY-RUN")
        return (f"{head} {ptxt} | {gtxt} | {etxt} | cmd_body=({c.vx:+.3f}, {c.vy:+.3f}, "
                f"{c.wz:+.3f}) | {wtxt} | {state}")

    def _link_text(self) -> str:
        if self.link is None:
            return "UART off"
        s = self.link.stats
        return f"UART {s.port} {'OK' if s.connected else '--'} rx={s.rx_ok} tx={s.tx_ok}"

    def _status_line(self) -> str:
        o = self._fresh_vision(time.perf_counter())
        c = self.last_cmd
        tag = (f"{o.target_name} ({o.tag_x:.0f},{o.tag_y:.0f}) w={o.width_px:.0f}px"
               if o is not None and o.is_tracking else (o.state if o else "sin visión"))
        extra = (f" {self.landmark} d={c.dist:.2f}m" if self.mode == LANDMARK and
                 c.dist is not None else "")
        return (f"[{self.phase} {self.mode or '-'}] {tag} | w=[" +
                ", ".join(f"{v:+.2f}" for v in c.wheels) + f"]{extra}"
                + (f" ({c.reason})" if c.reason else "") + f" | {self._link_text()}")


def parse_args(argv: Optional[list[str]] = None) -> tuple[argparse.Namespace, vt.Config]:
    p = argparse.ArgumentParser(
        description="Control por retroalimentación de error + UART con la ESP32",
        parents=[vt.build_arg_parser(add_help=False)])
    g = p.add_argument_group("control / UART")
    g.add_argument("--port", default=SERIAL_PORT, help='"auto", /dev/serial0, /dev/ttyUSB0 o "none"')
    g.add_argument("--baud", type=int, default=SERIAL_BAUD)
    g.add_argument("--mode", choices=MODES, type=str.upper,
                   help="modo local inicial (no espera a la ESP32)")
    g.add_argument("--landmark", type=str.upper, choices=list(LANDMARKS),
                   help="destino para --mode LANDMARK")
    g.add_argument("--no-wait-start", action="store_true",
                   help="arrancar al recibir MODE sin esperar START")
    g.add_argument("--control-hz", type=float, default=CONTROL_HZ)
    g.add_argument("--dry-run", action="store_true", help="calcula pero envía ruedas en 0")
    g.add_argument("--ref-width", type=float, default=None,
                   help=f"W_ref [px] a {DISTANCE_REF_M} m (en vez del archivo de calibración)")
    g.add_argument("--calibrate", action="store_true", help="medir W_ref y salir")
    g.add_argument("--no-camera", action="store_true", help="sin visión (solo LANDMARK)")
    g.add_argument("--sim", action="store_true", help="pose simulada a partir de las ruedas")
    g.add_argument("--sim-start", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                   metavar=("X", "Y", "THETA"))
    g.add_argument("--print-tel", action="store_true", help="imprimir cada trama TEL")
    g.add_argument("--kin-test", action="store_true",
                   help="al entrar en RUN: probar +vx, +vy, +wz y verificar signos con la odometría")
    g.add_argument("--heading-offset-deg", type=float, default=None,
                   help="rumbo del frente del rover en el mapa con theta = 0 "
                        f"(por defecto {math.degrees(ODOM_HEADING_OFFSET_RAD):+.0f}°: mira a +Y)")
    g.add_argument("--invert-theta", action=argparse.BooleanOptionalAction, default=None,
                   help=f"theta de la ESP32 crece en horario (por defecto {INVERT_THETA})")
    g.add_argument("--invert-vx", action=argparse.BooleanOptionalAction, default=None,
                   help=f"invertir vx en CAMERA y LANDMARK (por defecto INVERT_VX = {INVERT_VX})")
    g.add_argument("--invert-vy", action=argparse.BooleanOptionalAction, default=None,
                   help=f"invertir vy en CAMERA y LANDMARK (por defecto INVERT_VY = {INVERT_VY})")
    g.add_argument("--invert-wz", action=argparse.BooleanOptionalAction, default=None,
                   help=f"invertir wz en CAMERA y LANDMARK (por defecto INVERT_WZ = {INVERT_WZ})")
    g.add_argument("--invert-odom-vy", action=argparse.BooleanOptionalAction, default=None,
                   help="la Pi corrige el signo lateral de la odometría x, y de la ESP32 "
                        f"(por defecto INVERT_ODOM_VY = {INVERT_ODOM_VY})")
    g.add_argument("--invert-camera-vy", "--camera-invert-vy",
                   action=argparse.BooleanOptionalAction, default=None,
                   help="invertir solo el desplazamiento lateral de CAMERA respecto a ex "
                        f"(por defecto INVERT_CAMERA_VY = {INVERT_CAMERA_VY})")
    g.add_argument("--invert-camera-wz", action=argparse.BooleanOptionalAction, default=None,
                   help="invertir solo el giro de CAMERA respecto a ex "
                        f"(por defecto INVERT_CAMERA_WZ = {INVERT_CAMERA_WZ})")
    g.add_argument("--theta-deg", action="store_true", help="la ESP32 envía theta en grados")
    g.add_argument("--sim-flip-vy", action="store_true", help=argparse.SUPPRESS)
    g.add_argument("--sim-flip-wz", action="store_true", help=argparse.SUPPRESS)
    g.add_argument("--sim-flip-theta", action="store_true", help=argparse.SUPPRESS)
    g.add_argument("--sim-offset-deg", type=float, default=None, help=argparse.SUPPRESS)
    g.add_argument("--sim-yaw-drift", type=float, default=0.0, help=argparse.SUPPRESS)
    a = p.parse_args(argv)
    if a.landmark and a.mode != LANDMARK:
        p.error("--landmark requiere --mode LANDMARK")
    return a, vt.config_from_args(a)


def _raise_keyboard_interrupt(signum, frame) -> None:
    raise KeyboardInterrupt


def main(argv: Optional[list[str]] = None) -> int:
    vt.line_buffered_output()
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(threadName)s: %(message)s")
    a, vcfg = parse_args(argv)
    global INVERT_VX, INVERT_VY, INVERT_WZ, INVERT_THETA, ODOM_HEADING_OFFSET_RAD
    global INVERT_CAMERA_VY, INVERT_CAMERA_WZ, INVERT_ODOM_VY, THETA_IN_DEGREES
    if a.heading_offset_deg is not None:
        ODOM_HEADING_OFFSET_RAD = math.radians(a.heading_offset_deg)
    if a.invert_theta is not None:
        INVERT_THETA = a.invert_theta
    if a.invert_vx is not None:
        INVERT_VX = a.invert_vx
    if a.invert_vy is not None:
        INVERT_VY = a.invert_vy
    if a.invert_wz is not None:
        INVERT_WZ = a.invert_wz
    if a.invert_odom_vy is not None:
        INVERT_ODOM_VY = a.invert_odom_vy
    if a.invert_camera_vy is not None:
        INVERT_CAMERA_VY = a.invert_camera_vy
    if a.invert_camera_wz is not None:
        INVERT_CAMERA_WZ = a.invert_camera_wz
    if a.theta_deg:
        THETA_IN_DEGREES = True
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    if a.calibrate:
        try:
            return run_calibration(vcfg)
        except KeyboardInterrupt:
            return 1
    return FollowerApp(a, vcfg).run()


if __name__ == "__main__":
    sys.exit(main())
