import math

import cv2
import numpy as np
import pytest

from jetrover_sorting.tracking import (Blob, Follower, FollowerConfig, StillnessGate,
                                       depth_at, detect_cubes, pick_target)

LAB = {'red': {'min': [0, 142, 125], 'max': [255, 255, 255]},
       'green': {'min': [30, 0, 105], 'max': [255, 113, 255]},
       'blue': {'min': [20, 70, 0], 'max': [220, 160, 100]}}


def frame(shapes):
    """shapes: list of (rgb, (x, y, w, h)) rectangles on a grey background."""
    img = np.full((360, 640, 3), 200, np.uint8)
    for rgb, (x, y, w, h) in shapes:
        img[y:y + h, x:x + w] = rgb
    return img


def test_detects_a_red_square_at_full_res_coordinates():
    blobs = detect_cubes(frame([((220, 40, 40), (300, 280, 40, 40))]), LAB, ['red', 'green', 'blue'])
    assert len(blobs) == 1
    b = blobs[0]
    assert b.color == 'red'
    assert abs(b.x - 320) < 3 and abs(b.y - 300) < 3
    assert 25 < b.radius < 32 and b.fill > 0.55


def test_elongated_red_thing_is_rejected_as_not_a_cube():
    sleeve = frame([((220, 40, 40), (100, 150, 300, 30))])
    assert detect_cubes(sleeve, LAB, ['red']) == []
    assert len(detect_cubes(sleeve, LAB, ['red'], min_fill=0.0)) == 1


def test_all_three_colours_in_one_frame():
    img = frame([((220, 40, 40), (50, 100, 40, 40)), ((40, 200, 120), (300, 100, 40, 40)),
                 ((60, 160, 230), (550, 100, 40, 40))])
    assert sorted(b.color for b in detect_cubes(img, LAB, ['red', 'green', 'blue'])) == ['blue', 'green', 'red']


def test_depth_at_handles_holes_and_too_close():
    d = np.zeros((360, 640), np.uint16)
    assert depth_at(d, 320, 180) is None
    d[170:190, 310:330] = 316
    assert depth_at(d, 320, 180) == pytest.approx(0.316)
    d[:, :] = 50                                        # closer than the sensor can measure
    assert depth_at(d, 320, 180) is None


def test_pick_target_prefers_largest_within_range_and_drops_far_ones():
    d = np.full((360, 640), 300, np.uint16)
    d[0:50, 0:50] = 2500                                # a shirt across the room
    far_big = Blob('red', 25, 25, 40, 0.9)
    near_small = Blob('blue', 320, 180, 20, 0.9)
    near_big = Blob('green', 400, 200, 30, 0.9)
    assert pick_target([far_big, near_small, near_big], d, 0.12, 0.45)[0] is near_big
    d[190:210, 390:410] = 0                             # depth hole on the big one: still tracked, z None
    b, z = pick_target([near_big, near_small], d, 0.12, 0.45)
    assert b is near_big and z is None
    assert pick_target([], d, 0.12, 0.45) is None


def run_follower(fps, seconds, err=(0.3, 0.0)):
    f = Follower(FollowerConfig())
    n = int(round(fps * seconds))
    for k in range(n + 1):
        f.update(err[0], err[1], k * seconds / n)
    return f


def test_follower_is_frame_rate_independent():
    slow = run_follower(8, 0.5)
    fast = run_follower(30, 0.5)
    assert slow.yaw < 500 and fast.yaw < 500                  # target right -> yaw decreases
    assert abs(slow.yaw - fast.yaw) < 5
    assert slow.pitch == fast.pitch == 150


def test_follower_rate_cap_and_deadband():
    f = Follower(FollowerConfig(max_rate=400.0))
    f.update(0.5, 0.5, 0.0)
    y1, p1, _ = f.update(0.5, 0.5, 0.1)                       # dt = 0.1 -> at most 40 units
    assert 500 - y1 <= 40 and 150 - p1 <= 40
    f2 = Follower(FollowerConfig())
    f2.update(0.01, -0.01, 0.0)
    assert f2.update(0.01, -0.01, 0.033)[:2] == (500, 150)


def test_follower_long_gap_is_not_a_huge_step():
    f = Follower(FollowerConfig())
    f.update(0.4, 0.0, 0.0)
    f.update(0.4, 0.0, 5.0)                                   # 5 s gap
    assert 500 - f.yaw <= 2 * 400 * 0.2 + 1


def test_stillness_gate():
    g = StillnessGate(hold_s=0.75, radius_px=12, max_step=2.0)
    t = 0.0
    for _ in range(40):                                       # 1.3 s of a steady target
        ok = g.update(320 + (t * 37 % 5), 180, (0.0, 0.0), t)
        t += 1 / 30
    assert ok and g.progress == 1.0
    assert not g.update(320, 180, (5.0, 0.0), t)             # arm stepped: reset
    assert g.progress == 0.0
    t += 1 / 30
    for i in range(40):                                       # target drifting 3 px/frame
        ok = g.update(320 + 3 * i, 180, (0.0, 0.0), t)
        t += 1 / 30
    assert not ok


def test_stillness_needs_the_full_hold_time():
    g = StillnessGate(hold_s=0.75)
    assert not any(g.update(320, 180, (0.0, 0.0), i / 30) for i in range(20))   # 0.63 s
    assert g.update(320, 180, (0.0, 0.0), 0.76)
