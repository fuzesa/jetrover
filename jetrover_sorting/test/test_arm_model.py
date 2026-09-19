import os
import sqlite3
import sys

import pytest

from jetrover_sorting.arm_model import SHOULDER_HEIGHT, forward, inverse

TANK = SHOULDER_HEIGHT['JetRover_Tank']
MECANUM = SHOULDER_HEIGHT['JetRover_Mecanum']
GRASP = (195, 300, 285)        # vendor pick.d6a, rows 2-5
VIEW = (700, 15, 215)          # vendor pick_init.d6a


def test_vendor_grasp_pose_is_a_near_vertical_reach_to_table_level():
    reach, height, wrist = forward(GRASP, MECANUM)
    assert reach == pytest.approx(0.257, abs=0.001)
    assert height == pytest.approx(0.027, abs=0.001)      # inside a 30 mm cube
    assert -(wrist + 90) == pytest.approx(-82.8, abs=0.1)  # tool pitch


def test_tank_shoulder_is_10_7_mm_higher_so_the_same_pose_ends_higher():
    dh = forward(GRASP, TANK)[1] - forward(GRASP, MECANUM)[1]
    assert dh == pytest.approx(0.0107, abs=0.0001)
    assert forward(GRASP, TANK)[1] > 0.030                 # above the cube's top face


def test_viewing_pose_looks_down_at_about_45_degrees():
    assert -(forward(VIEW, TANK)[2] + 90) == pytest.approx(-46.8, abs=0.1)


@pytest.mark.parametrize('pulses', [GRASP, VIEW, (180, 355, 245), (650, 15, 215)])
def test_inverse_round_trips_recorded_poses(pulses):
    reach, height, wrist = forward(pulses, TANK)
    assert inverse(reach, height, wrist, TANK) == pytest.approx(pulses, abs=1e-6)


def test_lowering_keeps_reach_and_tilt():
    reach, height, wrist = forward(GRASP, TANK)
    new = [round(p) for p in inverse(reach, height - 0.0107, wrist, TANK)]
    assert new == [181, 307, 292]
    r2, h2, w2 = forward(new, TANK)
    assert r2 == pytest.approx(reach, abs=0.001)
    assert h2 == pytest.approx(height - 0.0107, abs=0.001)
    assert w2 == pytest.approx(wrist, abs=0.5)


def test_unreachable_target_raises():
    with pytest.raises(ValueError):
        inverse(0.60, 0.03, -7.2, TANK)


PICK = [(1, 500, 875, 650, 15, 215, 500, 200), (2, 1500, 875, 195, 300, 285, 500, 200),
        (3, 200, 875, 195, 300, 285, 500, 200), (4, 500, 875, 195, 300, 285, 500, 500),
        (5, 200, 875, 195, 300, 285, 500, 500), (6, 1500, 875, 650, 15, 215, 500, 560)]
PLACE = [(1, 1000, 500, 650, 15, 280, 500, 560), (2, 1500, 500, 180, 355, 245, 500, 560),
         (3, 500, 500, 180, 355, 245, 500, 400), (4, 1500, 500, 650, 15, 215, 500, 400)]


def _make(path, rows):
    db = sqlite3.connect(path)
    db.execute('CREATE TABLE ActionGroup ([Index] INTEGER PRIMARY KEY, Time INT, Servo1 INT, Servo2 INT, '
               'Servo3 INT, Servo4 INT, Servo5 INT, Servo6 INT)')
    db.executemany('INSERT INTO ActionGroup VALUES (?,?,?,?,?,?,?,?)', rows)
    db.commit()
    db.close()


def _read(path):
    return [tuple(r) for r in sqlite3.connect(path).execute('select * from ActionGroup')]


@pytest.fixture
def tool(tmp_path, monkeypatch):
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
    import make_tank_actions as t
    _make(tmp_path / 'pick.d6a', PICK)
    for n in t.PLACE_GROUPS:
        _make(tmp_path / f'{n}.d6a', PLACE)
    monkeypatch.setattr(t, 'ACTION_DIR', str(tmp_path))
    monkeypatch.setenv('MACHINE_TYPE', 'JetRover_Tank')
    return t


def test_default_run_lowers_only_the_grasp_rows(tool, tmp_path, monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['x'])
    tool.main()
    out = _read(tmp_path / 'pick_tank.d6a')
    assert out[0] == PICK[0] and out[5] == PICK[5]
    assert all(r[3:6] == (181, 307, 292) for r in out[1:5])
    assert [r[:3] + r[6:] for r in out] == [r[:3] + r[6:] for r in PICK]   # timing, yaw, roll, gripper untouched
    assert _read(tmp_path / 'pick.d6a') == PICK                            # vendor file untouched
    assert not (tmp_path / 'place_center_tank.d6a').exists()               # not needed without --hold


def test_tighter_gripper_is_carried_through_the_place_groups(tool, tmp_path, monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['x', '13', '--close', '570', '--hold', '620'])
    tool.main()
    assert [r[7] for r in _read(tmp_path / 'pick_tank.d6a')] == [200, 200, 200, 570, 570, 620]
    for n in tool.PLACE_GROUPS:
        out = _read(tmp_path / f'{n}_tank.d6a')
        assert [r[7] for r in out] == [620, 620, 400, 400]                 # carried tight, release unchanged
        assert [r[:7] for r in out] == [r[:7] for r in PLACE]
        assert _read(tmp_path / f'{n}.d6a') == PLACE


@pytest.mark.parametrize('argv', [['x', '--close', '900', '--hold', '950'], ['x', '--close', '600', '--hold', '560'],
                                  ['x', '--suffix', '']])
def test_silly_arguments_are_refused(tool, monkeypatch, argv):
    monkeypatch.setattr(sys, 'argv', argv)
    with pytest.raises(SystemExit):
        tool.main()
