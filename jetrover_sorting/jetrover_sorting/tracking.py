"""Tracking logic for the hand-held cube demo. No ROS imports.

Three pieces, each testable on its own:

  detect_cubes   colour blobs in a frame, with a compactness filter so that a
                 sleeve or an arm does not pass for a cube
  Follower       turns detection error into servo targets. Frame-rate
                 independent: the correction is a rate (servo units per
                 second), scaled by the real time between frames and capped, so
                 the same gains behave the same at 8 fps and at 30 fps.
  StillnessGate  says when the target has been steady long enough to grab
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class Blob:
    color: str
    x: float          # full-resolution image pixels
    y: float
    radius: float
    fill: float       # contour area / enclosing-circle area, 1.0 = perfect disc


def detect_cubes(rgb, lab_ranges: dict, colors, min_area: int = 80,
                 min_fill: float = 0.5, scale: int = 2) -> list[Blob]:
    """rgb: HxWx3 uint8 (RGB order, as the camera publishes). lab_ranges:
    the 'Stereo' section of lab_config.yaml. Works on a downscaled copy, as
    the vendor does; coordinates are returned at full resolution."""
    h, w = rgb.shape[:2]
    small = cv2.resize(rgb, (w // scale, h // scale))
    lab = cv2.cvtColor(cv2.GaussianBlur(small, (3, 3), 3), cv2.COLOR_RGB2LAB)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    out = []
    for color in colors:
        rng = lab_ranges[color]
        mask = cv2.inRange(lab, tuple(rng['min']), tuple(rng['max']))
        mask = cv2.dilate(cv2.erode(mask, kernel), kernel)
        for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            (cx, cy), r = cv2.minEnclosingCircle(c)
            fill = area / (math.pi * r * r) if r > 0 else 0.0
            if fill < min_fill:
                continue
            out.append(Blob(color, cx * scale, cy * scale, r * scale, fill))
    return out


def depth_at(depth_mm, x: float, y: float, half: int = 5,
             min_mm: int = 100, max_mm: int = 5000) -> Optional[float]:
    """Median valid depth (metres) in a small window, or None if there is no
    valid depth there (holes, or closer than the sensor's minimum)."""
    h, w = depth_mm.shape[:2]
    xi, yi = int(round(x)), int(round(y))
    patch = depth_mm[max(yi - half, 0):min(yi + half + 1, h), max(xi - half, 0):min(xi + half + 1, w)]
    valid = patch[(patch > min_mm) & (patch < max_mm)]
    if valid.size < 3:
        return None
    return float(np.median(valid)) / 1000.0


@dataclass
class FollowerConfig:
    yaw_init: int = 500
    pitch_init: int = 150
    yaw_min: int = 0
    yaw_max: int = 1000
    pitch_min: int = 100
    pitch_max: int = 720
    # servo units per second per unit of normalised image error (error of 1.0
    # = target a full frame width away). ~1200 moves a target half a frame off
    # centre by 20 units per frame at 30 fps.
    gain: float = 1200.0
    max_rate: float = 400.0     # servo units per second, absolute cap
    deadband: float = 0.02      # normalised; inside this we do not move
    min_dt: float = 0.01
    max_dt: float = 0.2         # a long gap is treated as this, never as a huge step


class Follower:
    """Keeps the target centred by steering the base yaw (servo 1) and the
    wrist pitch (servo 4), like the vendor tracker, but as a rate."""

    def __init__(self, cfg: FollowerConfig):
        self.cfg = cfg
        self.yaw = float(cfg.yaw_init)
        self.pitch = float(cfg.pitch_init)
        self._last_t: Optional[float] = None
        self.last_step = (0.0, 0.0)

    def reset(self) -> None:
        self.yaw, self.pitch = float(self.cfg.yaw_init), float(self.cfg.pitch_init)
        self._last_t = None
        self.last_step = (0.0, 0.0)

    def update(self, ex: float, ey: float, now: float) -> tuple[int, int, float]:
        """ex, ey: normalised error, (target - centre) / frame size, in -0.5..0.5.
        Returns (yaw, pitch, dt) after this step. Signs follow the vendor:
        target to the right -> yaw decreases; target low -> pitch decreases."""
        c = self.cfg
        if self._last_t is None:            # first sighting: start the clock, no step
            self._last_t = now
            self.last_step = (0.0, 0.0)
            return int(round(self.yaw)), int(round(self.pitch)), 0.0
        dt = min(max(now - self._last_t, c.min_dt), c.max_dt)
        self._last_t = now
        cap = c.max_rate * dt

        def step(err):
            if abs(err) <= c.deadband:
                return 0.0
            return max(-cap, min(cap, -c.gain * err * dt))

        dy, dp = step(ex), step(ey)
        self.yaw = min(max(self.yaw + dy, c.yaw_min), c.yaw_max)
        self.pitch = min(max(self.pitch + dp, c.pitch_min), c.pitch_max)
        self.last_step = (dy, dp)
        return int(round(self.yaw)), int(round(self.pitch)), dt

    def lost(self, now: float) -> None:
        """Target not seen this frame: hold position, keep the clock honest."""
        self._last_t = now
        self.last_step = (0.0, 0.0)


class StillnessGate:
    """The target must sit within `radius_px` of where it was for `hold_s`
    seconds, with the servos no longer stepping, before a grab is allowed."""

    def __init__(self, hold_s: float = 0.75, radius_px: float = 12.0, max_step: float = 2.0):
        self.hold_s, self.radius_px, self.max_step = hold_s, radius_px, max_step
        self._hist: deque = deque()   # (t, x, y)

    def reset(self) -> None:
        self._hist.clear()

    def update(self, x: float, y: float, step: tuple[float, float], now: float) -> bool:
        if abs(step[0]) > self.max_step or abs(step[1]) > self.max_step:
            self._hist.clear()          # arm still moving: start over
            return False
        self._hist.append((now, x, y))
        while self._hist and now - self._hist[0][0] > self.hold_s * 1.5:
            self._hist.popleft()
        if now - self._hist[0][0] < self.hold_s:
            return False
        xs = [p[1] for p in self._hist]
        ys = [p[2] for p in self._hist]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        return all(math.hypot(px - cx, py - cy) <= self.radius_px for px, py in zip(xs, ys))

    @property
    def progress(self) -> float:
        if not self._hist:
            return 0.0
        return min(1.0, (self._hist[-1][0] - self._hist[0][0]) / self.hold_s)


def pick_target(blobs: list[Blob], depth_mm, min_range: float, max_range: float):
    """Largest blob whose depth is unknown-but-plausible or within range.
    Returns (blob, depth_m or None). Blobs that are measurably too far (a
    shirt across the room) are dropped; too close is kept but flagged as None."""
    best = None
    for b in sorted(blobs, key=lambda b: -b.radius):
        z = depth_at(depth_mm, b.x, b.y) if depth_mm is not None else None
        if z is not None and (z > max_range or z < min_range):
            continue
        best = (b, z)
        break
    return best
