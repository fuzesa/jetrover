"""What the robot sees, drawn for an audience. No ROS imports.

render() takes a camera frame and a small Overlay describing the node's state
and returns an image ready for cv2.imshow. OpenCV's fonts are ASCII only, so
texts must not contain diacritics (write "rosu", not "roșu").
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import cv2

# BGR colours for drawing each cube colour
DRAW = {'red': (40, 40, 230), 'green': (60, 200, 60), 'blue': (230, 140, 40)}
WHITE, BLACK = (255, 255, 255), (0, 0, 0)


@dataclass
class Texts:
    searching: str = 'Show me a cube!'
    following: str = 'I see a {color} cube'
    steady: str = 'Hold it still...'
    grabbing: str = 'Got it!'
    placing: str = 'The {color} cube goes in its box'
    too_far: str = 'Come closer!'
    too_close: str = 'Move it back a little!'
    colors: dict = field(default_factory=lambda: {'red': 'red', 'green': 'green', 'blue': 'blue'})


@dataclass
class Overlay:
    mode: str = 'searching'          # searching | following | grabbing | placing
    color: Optional[str] = None
    x: float = 0.0                   # target in camera pixels
    y: float = 0.0
    radius: float = 0.0
    z: Optional[float] = None        # metres
    progress: float = 0.0            # stillness 0..1
    hint: Optional[str] = None       # None | 'far' | 'close'


def _text(img, text, org, scale, thickness, color=WHITE, anchor='left'):
    font = cv2.FONT_HERSHEY_DUPLEX
    (w, h), _ = cv2.getTextSize(text, font, scale, thickness)
    x, y = org
    if anchor == 'center':
        x -= w // 2
    cv2.putText(img, text, (x, y), font, scale, BLACK, thickness + 4, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def render(rgb, ov: Overlay, texts: Texts, mirror: bool = True):
    """rgb: HxWx3 uint8 in RGB order. Returns a BGR image of the same size."""
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    h, w = img.shape[:2]
    if mirror:                       # the audience faces the camera: show it like a mirror
        img = cv2.flip(img, 1)
    x = (w - 1 - ov.x) if mirror else ov.x
    s = h / 360.0                    # scale everything to the frame size
    name = texts.colors.get(ov.color, ov.color or '')
    col = DRAW.get(ov.color, WHITE)

    if ov.mode == 'following' and ov.radius > 0:
        c, r = (int(x), int(ov.y)), int(ov.radius)
        cv2.circle(img, c, r + int(4 * s), BLACK, int(6 * s), cv2.LINE_AA)
        cv2.circle(img, c, r + int(4 * s), col, int(3 * s), cv2.LINE_AA)
        label = name.upper() + (f'  {ov.z * 100:.0f} cm' if ov.z is not None else '')
        _text(img, label, (c[0], max(c[1] - r - int(14 * s), int(20 * s))), 0.6 * s, 1, col, 'center')

    if ov.mode == 'searching':
        headline = texts.searching
    elif ov.mode == 'following' and ov.hint == 'far':
        headline = texts.too_far
    elif ov.mode == 'following' and ov.hint == 'close':
        headline = texts.too_close
    elif ov.mode == 'following':
        headline = texts.steady if ov.progress > 0.15 else texts.following.format(color=name)
    elif ov.mode == 'grabbing':
        headline = texts.grabbing
    else:
        headline = texts.placing.format(color=name)
    _text(img, headline, (w // 2, int(40 * s)), 0.9 * s, 2, WHITE, 'center')

    if ov.mode == 'following':       # "getting ready" bar
        x0, x1, y0, y1 = int(w * 0.2), int(w * 0.8), h - int(30 * s), h - int(14 * s)
        cv2.rectangle(img, (x0, y0), (x1, y1), BLACK, -1)
        fill = x0 + int((x1 - x0) * min(max(ov.progress, 0.0), 1.0))
        if fill > x0:
            cv2.rectangle(img, (x0, y0), (fill, y1), col, -1)
        cv2.rectangle(img, (x0, y0), (x1, y1), WHITE, 1)
    return img
