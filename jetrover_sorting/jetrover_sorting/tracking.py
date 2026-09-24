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
                 min_fill: float = 0.5, scale: int = 2, border_fill: float = 0.2) -> list[Blob]:
    """rgb: HxWx3 uint8 (RGB order, as the camera publishes). lab_ranges:
    the 'Stereo' section of lab_config.yaml. Works on a downscaled copy, as
    the vendor does; coordinates are returned at full resolution. A blob
    cut off by the image border is judged with the looser `border_fill`, so a
    cube leaving the frame is still followed back in."""
    h, w = rgb.shape[:2]
    small = cv2.resize(rgb, (w // scale, h // scale))
    lab = cv2.cvtColor(cv2.GaussianBlur(small, (3, 3), 3), cv2.COLOR_RGB2LAB)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    out = []
    for color in colors:
        rng = lab_ranges[color]
        mask = cv2.inRange(lab, tuple(rng['min']), tuple(rng['max']))
        mask = cv2.dilate(cv2.erode(mask, kernel), kernel)
        sh, sw = mask.shape[:2]
        for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            (cx, cy), r = cv2.minEnclosingCircle(c)
            fill = area / (math.pi * r * r) if r > 0 else 0.0
            bx, by, bw, bh = cv2.boundingRect(c)
            at_border = bx <= 1 or by <= 1 or bx + bw >= sw - 1 or by + bh >= sh - 1
            if fill < (border_fill if at_border else min_fill):
                continue
            out.append(Blob(color, cx * scale, cy * scale, r * scale, fill))
    return out


def depth_of_blob(depth_mm, blob, margin_px: int = 30, pct: float = 20.0,
                  min_mm: int = 100, max_mm: int = 5000, min_valid: int = 20) -> Optional[float]:
    """Depth (metres) of a blob, robust to the depth/colour misalignment and to
    edge holes of a structured-light sensor: search a window covering the blob
    plus `margin_px` (the colour->depth shift is ~20 px at 20 cm), and take a
    low percentile of the valid readings. The cube (and the hand holding it) is
    the nearest surface there; background and holes lose."""
    h, w = depth_mm.shape[:2]
    r = int(blob.radius * 0.8) + margin_px
    xi, yi = int(round(blob.x)), int(round(blob.y))
    patch = depth_mm[max(yi - r, 0):min(yi + r + 1, h), max(xi - r, 0):min(xi + r + 1, w)]
    valid = patch[(patch > min_mm) & (patch < max_mm)]
    if valid.size < min_valid:
        return None
    return float(np.percentile(valid, pct)) / 1000.0


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
    pitch_init: int = 120        # must match the look-out pose the node homes to
    yaw_min: int = 0
    yaw_max: int = 1000
    pitch_min: int = 100
    pitch_max: int = 720
    # servo units per second per unit of normalised image error (error of 1.0
    # = target a full frame width away). ~1200 moves a target half a frame off
    # centre by 20 units per frame at 30 fps.
    gain: float = 1200.0
    gain_pitch: Optional[float] = None   # up/down gain; None = same as `gain`
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
        self.last_rate = (0.0, 0.0)     # servo units per second of the last step

    def reset(self) -> None:
        self.yaw, self.pitch = float(self.cfg.yaw_init), float(self.cfg.pitch_init)
        self._last_t = None
        self.last_step = (0.0, 0.0)
        self.last_rate = (0.0, 0.0)

    def update(self, ex: float, ey: float, now: float) -> tuple[int, int, float]:
        """ex, ey: normalised error, (target - centre) / frame size, in -0.5..0.5.
        Returns (yaw, pitch, dt) after this step. Signs follow the vendor:
        target to the right -> yaw decreases; target low -> pitch decreases."""
        c = self.cfg
        if self._last_t is None:            # first sighting: start the clock, no step
            self._last_t = now
            self.last_step = self.last_rate = (0.0, 0.0)
            return int(round(self.yaw)), int(round(self.pitch)), 0.0
        dt = min(max(now - self._last_t, c.min_dt), c.max_dt)
        self._last_t = now
        cap = c.max_rate * dt

        def step(err, gain):
            if abs(err) <= c.deadband:
                return 0.0
            return max(-cap, min(cap, -gain * err * dt))

        dy = step(ex, c.gain)
        dp = step(ey, c.gain if c.gain_pitch is None else c.gain_pitch)
        self.yaw = min(max(self.yaw + dy, c.yaw_min), c.yaw_max)
        self.pitch = min(max(self.pitch + dp, c.pitch_min), c.pitch_max)
        self.last_step = (dy, dp)
        self.last_rate = (dy / dt, dp / dt)
        return int(round(self.yaw)), int(round(self.pitch)), dt

    def lost(self, now: float) -> None:
        """Target not seen this frame: hold position, keep the clock honest."""
        self._last_t = now
        self.last_step = self.last_rate = (0.0, 0.0)


class StillnessGate:
    """The target must sit within `radius_px` of where it was for `hold_s`
    seconds, with the arm turning slower than `max_rate` servo units per
    second (frame-rate independent), before a grab is allowed."""

    def __init__(self, hold_s: float = 0.75, radius_px: float = 12.0, max_rate: float = 80.0):
        self.hold_s, self.radius_px, self.max_rate = hold_s, radius_px, max_rate
        self._hist: deque = deque()   # (t, x, y)

    def reset(self) -> None:
        self._hist.clear()

    def update(self, x: float, y: float, rate: tuple[float, float], now: float) -> bool:
        if abs(rate[0]) > self.max_rate or abs(rate[1]) > self.max_rate:
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


CUBE_HALF_DIAG = 0.0212   # m; 30 mm cube face, centre to corner


def size_fits_distance(radius_px: float, z: float, fx: float,
                       lo: float = 0.5, hi: float = 1.6) -> bool:
    """A cube at distance z has an enclosing-circle radius of about
    fx * CUBE_HALF_DIAG / z pixels. Jeans or a shirt at arm's length are far
    bigger than that; fingers hiding part of the cube make it a bit smaller."""
    expected = fx * CUBE_HALF_DIAG / z
    return lo * expected <= radius_px <= hi * expected


def pick_target(blobs: list[Blob], depth_mm, min_range: float, max_range: float,
                fx: Optional[float] = None, size_lo: float = 0.5, size_hi: float = 1.6):
    """Largest blob whose depth is unknown-but-plausible or within range, and
    (when fx is known) whose size fits a cube at that depth.
    Returns (blob, depth_m or None). Blobs that are measurably too far (a
    shirt across the room) or the wrong size for their distance (jeans up
    close) are dropped; too close is kept but flagged as None."""
    best = None
    for b in sorted(blobs, key=lambda b: -b.radius):
        z = depth_of_blob(depth_mm, b) if depth_mm is not None else None
        if z is not None and (z > max_range or z < min_range):
            continue
        if z is not None and fx and not size_fits_distance(b.radius, z, fx, size_lo, size_hi):
            continue
        best = (b, z)
        break
    return best


class TargetFilter:
    """Smooths the tracked position (less twitch) and bridges short gaps: a
    frame without the target, or without valid depth, does not reset anything
    unless it lasts longer than `hold_s`."""

    def __init__(self, alpha: float = 0.5, hold_s: float = 0.3, depth_hold_s: float = 0.6,
                 jump_px: float = 60.0):
        self.alpha, self.hold_s, self.depth_hold_s, self.jump_px = alpha, hold_s, depth_hold_s, jump_px
        self.reset()

    def reset(self) -> None:
        self.x = self.y = None
        self.color = None
        self.z = None
        self._t_seen = self._t_depth = -1e9

    def update(self, blob, z, now: float):
        """Returns (color, x, y, z) smoothed, or None if the target is lost."""
        if blob is None:
            return None if now - self._t_seen > self.hold_s else (self.color, self.x, self.y, self._z(now))
        jumped = (self.x is None or blob.color != self.color
                  or math.hypot(blob.x - self.x, blob.y - self.y) > self.jump_px)
        if jumped:
            self.x, self.y, self.color, self.z = blob.x, blob.y, blob.color, None
            self._t_depth = -1e9
        else:
            a = self.alpha
            self.x += a * (blob.x - self.x)
            self.y += a * (blob.y - self.y)
        self._t_seen = now
        if z is not None:
            self.z, self._t_depth = z, now
        return self.color, self.x, self.y, self._z(now)

    def _z(self, now):
        return self.z if now - self._t_depth <= self.depth_hold_s else None

    @property
    def lost_for(self):
        return self._t_seen
