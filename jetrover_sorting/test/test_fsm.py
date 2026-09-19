import csv

import pytest

from jetrover_sorting.fsm import (
    AttemptFinished, Detection, FsmConfig, Roi, RunAction, SortingFsm, State, StreakLost,
    OUTCOME_CLEARED, OUTCOME_ERROR, OUTCOME_STILL_PRESENT, OUTCOME_UNVERIFIED)
from jetrover_sorting.pick_log import DetectionTrace, PickLogger

DT = 1 / 30.0
ROI = Roi(x_min=280, x_max=360, y_min=70, y_max=150)  # centre (320, 110)
PLACE = {'red': 'place_center', 'green': 'place_left', 'blue': 'place_right'}


def det(color='red', x=320, y=110, radius=20):
    return Detection(color, x, y, radius)


class Harness:
    """Drives the FSM with a fake clock and collects every command."""

    def __init__(self, **overrides):
        params = dict(roi=ROI, place_actions=PLACE, confirm_frames=5,
                      miss_tolerance=1, max_jitter_px=8.0, min_radius=10.0,
                      detection_timeout=0.5, settle_time=0.5, verify_window=1.0,
                      verify_hits=2, action_timeout=10.0)
        params.update(overrides)
        self.fsm = SortingFsm(FsmConfig(**params))
        self.now = 0.0
        self.cmds = []

    def _take(self, cmds):
        self.cmds.extend(cmds)
        return cmds

    def start(self):
        return self._take(self.fsm.start(self.now))

    def stop(self):
        return self._take(self.fsm.stop(self.now))

    def feed(self, d, n=1):
        out = []
        for _ in range(n):
            self.now += DT
            out += self._take(self.fsm.on_detection(d, self.now))
            out += self._take(self.fsm.tick(self.now))
        return out

    def done(self, name, ok=True, after=1.0, detail=''):
        self.now += after
        return self._take(self.fsm.on_action_done(name, ok, self.now, detail))

    def wait(self, seconds):
        self.now += seconds
        return self._take(self.fsm.tick(self.now))

    def searching(self):
        self.start()
        self.done('pick_init')
        assert self.fsm.state == State.SEARCHING
        self.cmds.clear()
        return self

    def actions(self):
        return [c.name for c in self.cmds if isinstance(c, RunAction)]

    def lost(self):
        return [(c.reason, c.hits) for c in self.cmds if isinstance(c, StreakLost)]

    def records(self):
        return [c.record for c in self.cmds if isinstance(c, AttemptFinished)]


# ---------------------------------------------------------------- start-up

def test_start_homes_before_searching():
    h = Harness()
    assert h.fsm.state == State.IDLE
    assert h.start() == [RunAction('pick_init')]
    assert h.fsm.state == State.HOMING
    h.feed(det(), 20)                       # detections while homing are ignored
    assert h.fsm.state == State.HOMING
    h.done('pick_init')
    assert h.fsm.state == State.SEARCHING


def test_detections_ignored_while_idle():
    h = Harness()
    h.feed(det(), 50)
    assert h.fsm.state == State.IDLE and h.cmds == []


# ------------------------------------------------------------ confirmation

def test_commits_after_consecutive_frames():
    h = Harness().searching()
    h.feed(det(), 4)
    assert h.fsm.state == State.CONFIRMING and h.actions() == []
    h.feed(det(), 1)
    assert h.fsm.state == State.PICKING and h.actions() == ['pick']


def test_stock_bug_nonconsecutive_frames_never_accumulate():
    """The vendor node counted qualifying frames forever without resetting;
    sporadic blips would eventually trigger a pick."""
    h = Harness().searching()
    for _ in range(40):
        h.feed(det(), 2)
        h.feed(None, 3)                     # exceeds miss_tolerance=1
    assert h.actions() == []
    assert h.fsm.state == State.SEARCHING


def test_short_dropout_is_tolerated():
    h = Harness().searching()
    h.feed(det(), 3)
    h.feed(None, 1)                         # within miss_tolerance
    h.feed(det(), 2)
    assert h.actions() == ['pick']


def test_colour_flicker_restarts_streak_and_uses_final_colour():
    h = Harness().searching()
    h.feed(det('red'), 4)
    h.feed(det('blue'), 1)                  # restart, streak = 1
    assert h.fsm.state == State.CONFIRMING and h.actions() == []
    h.feed(det('blue'), 4)
    assert h.actions() == ['pick']
    h.done('pick')
    assert h.actions() == ['pick', 'place_right']


def test_moving_object_does_not_confirm():
    h = Harness().searching()
    for i in range(30):                     # drifts 3 px per frame
        h.feed(det(x=282 + 3 * i % 75, y=110), 1)
    assert h.actions() == []


def test_small_jitter_is_fine():
    h = Harness().searching()
    for i in range(5):
        h.feed(det(x=320 + (i % 2) * 3, y=110 - (i % 2) * 2), 1)
    assert h.actions() == ['pick']


@pytest.mark.parametrize('bad', [
    det(x=200),                             # outside ROI
    det(y=160),
    det(x=280),                             # on the boundary = outside
    det(radius=5),                          # too small
    det(color='yellow'),                    # not a sortable colour
])
def test_non_qualifying_detections_never_trigger(bad):
    h = Harness().searching()
    h.feed(bad, 100)
    assert h.fsm.state == State.SEARCHING and h.actions() == []


def test_detector_silence_abandons_streak():
    h = Harness().searching()
    h.feed(det(), 3)
    h.wait(0.6)                             # > detection_timeout, no messages
    assert h.fsm.state == State.SEARCHING
    h.feed(det(), 4)
    assert h.actions() == []                # had to start over
    h.feed(det(), 1)
    assert h.actions() == ['pick']


def test_confirm_frames_one_commits_immediately():
    h = Harness(confirm_frames=1).searching()
    h.feed(det('green'), 1)
    assert h.actions() == ['pick']


# ------------------------------------------------------------- diagnostics

def test_streak_lost_reports_the_reason():
    h = Harness().searching()
    h.feed(det(), 3); h.feed(None, 2)
    h.feed(det(), 2); h.feed(det(x=365), 2)             # centre hops over the ROI edge
    h.feed(det(), 2); h.feed(det(radius=4), 2)
    h.feed(det('red'), 2); h.feed(det('green'), 1)      # restart, stays CONFIRMING
    h.feed(det('green', x=340), 1)                      # jumped 20 px
    h.wait(0.6)
    assert h.lost() == [('no_detection', 3), ('outside_roi', 2), ('too_small', 2),
                        ('color_changed', 2), ('moved', 1), ('detector_silent', 1)]
    assert h.actions() == []


def test_streak_lost_carries_the_offending_detection():
    h = Harness().searching()
    h.feed(det(), 2); h.feed(det(x=362, y=151, radius=33), 2)
    (c,) = [c for c in h.cmds if isinstance(c, StreakLost)]
    assert (c.color, c.last.x, c.last.y, c.last.radius) == ('red', 362, 151, 33)


def test_detection_trace(tmp_path):
    fsm = SortingFsm(FsmConfig(roi=ROI, place_actions=PLACE))
    tr = DetectionTrace(str(tmp_path), session='t')
    tr.write(10.0, 'searching', det(), fsm.why_not(det()))
    tr.write(10.033, 'confirming', None, fsm.why_not(None))
    tr.write(10.066, 'confirming', det(x=400), fsm.why_not(det(x=400)))
    tr.close()
    rows = list(csv.DictReader(open(tr.path)))
    assert [r['why_not'] for r in rows] == ['', 'no_detection', 'outside_roi']
    assert rows[0]['t'] == '0.000' and rows[2]['x'] == '400'


# ------------------------------------------------------------------ cycle

def run_cycle(h, color='green'):
    h.feed(det(color), 5)
    h.done('pick')
    h.done(PLACE[color])
    h.done('pick_init')


def test_full_cycle_cleared():
    h = Harness().searching()
    run_cycle(h, 'green')
    assert h.actions() == ['pick', 'place_left', 'pick_init']
    assert h.fsm.state == State.VERIFYING
    h.feed(None, 60)                        # 2 s of empty view
    assert h.fsm.state == State.SEARCHING
    (rec,) = h.records()
    assert rec.outcome == OUTCOME_CLEARED
    assert (rec.attempt, rec.color, rec.place_action) == (1, 'green', 'place_left')
    assert rec.frames_to_confirm == 5
    assert rec.cycle_duration > 3.0


def test_verify_still_present_means_missed_grasp():
    h = Harness().searching()
    run_cycle(h, 'red')
    h.feed(det('red', x=300, y=90), 10)     # during settle_time: ignored
    assert h.fsm.state == State.VERIFYING
    h.feed(det('red', x=300, y=90), 10)
    (rec,) = h.records()
    assert rec.outcome == OUTCOME_STILL_PRESENT
    # the block is still sitting in the ROI, so the FSM goes for a retry
    h.feed(det('red', x=300, y=90), 10)
    assert h.actions()[-1] == 'pick' and h.fsm.attempts == 2


def test_verify_ignores_other_colours():
    h = Harness().searching()
    run_cycle(h, 'red')
    h.feed(det('blue'), 60)
    assert h.records()[0].outcome == OUTCOME_CLEARED


def test_verify_times_out_without_any_detector_messages():
    h = Harness().searching()
    run_cycle(h)
    h.wait(2.0)
    assert h.records()[0].outcome == OUTCOME_CLEARED


def test_detections_during_motion_are_ignored():
    h = Harness().searching()
    h.feed(det('red'), 5)
    h.feed(det('blue'), 100)                # camera is swinging around
    assert h.fsm.state == State.PICKING
    h.done('pick')
    assert h.actions() == ['pick', 'place_center']


def test_record_holds_mean_centre_and_offset_from_roi_centre():
    h = Harness().searching()
    for x in (322, 324, 326, 324, 324):
        h.feed(det(x=x, y=114), 1)
    h.done('pick'); h.done('place_center'); h.done('pick_init'); h.wait(2.0)
    rec = h.records()[0]
    assert rec.x == pytest.approx(324) and rec.y == pytest.approx(114)
    assert rec.dx == pytest.approx(4) and rec.dy == pytest.approx(4)


def test_attempt_counter_increments():
    h = Harness().searching()
    for _ in range(3):
        run_cycle(h)
        h.wait(2.0)
    assert [r.attempt for r in h.records()] == [1, 2, 3]


# ------------------------------------------------------------------- stop

def test_stop_while_searching_is_immediate():
    h = Harness().searching()
    h.feed(det(), 3)
    h.stop()
    assert h.fsm.state == State.IDLE
    h.feed(det(), 50)
    assert h.actions() == []


def test_stop_mid_cycle_finishes_the_cycle_first():
    h = Harness().searching()
    h.feed(det('blue'), 5)
    h.stop()
    assert h.fsm.state == State.PICKING     # never freeze holding a block
    h.done('pick'); h.done('place_right'); h.done('pick_init')
    assert h.fsm.state == State.IDLE
    assert h.actions() == ['pick', 'place_right', 'pick_init']
    assert h.records()[0].outcome == OUTCOME_UNVERIFIED


def test_stop_during_initial_homing():
    h = Harness()
    h.start(); h.stop()
    assert h.fsm.state == State.HOMING
    h.done('pick_init')
    assert h.fsm.state == State.IDLE


def test_start_cancels_a_pending_stop():
    h = Harness().searching()
    h.feed(det(), 5)
    h.stop(); h.start()
    h.done('pick'); h.done('place_center'); h.done('pick_init')
    assert h.fsm.state == State.VERIFYING


def test_stop_while_verifying():
    h = Harness().searching()
    run_cycle(h)
    h.stop()
    assert h.fsm.state == State.IDLE
    assert h.records()[0].outcome == OUTCOME_UNVERIFIED


# ------------------------------------------------------------------ errors

def test_failed_action_goes_to_error_and_is_logged():
    h = Harness().searching()
    h.feed(det(), 5)
    h.done('pick', ok=False, detail='pick.d6a not found')
    assert h.fsm.state == State.ERROR
    assert h.fsm.error_detail == 'pick.d6a not found'
    rec = h.records()[0]
    assert rec.outcome == OUTCOME_ERROR and rec.detail == 'pick.d6a not found'
    h.feed(det(), 50)
    assert h.actions() == ['pick']          # nothing further happens


def test_action_timeout():
    h = Harness().searching()
    h.feed(det(), 5)
    h.wait(10.5)
    assert h.fsm.state == State.ERROR
    assert 'timed out' in h.fsm.error_detail
    assert not h.fsm.busy


def test_late_report_after_timeout_is_ignored():
    h = Harness().searching()
    h.feed(det(), 5)
    h.wait(10.5)
    assert h.done('pick') == []
    assert h.fsm.state == State.ERROR


def test_failed_initial_homing_has_no_record():
    h = Harness()
    h.start()
    h.done('pick_init', ok=False)
    assert h.fsm.state == State.ERROR and h.records() == []


def test_restart_from_error():
    h = Harness().searching()
    h.feed(det(), 5)
    h.done('pick', ok=False)
    assert h.start() == [RunAction('pick_init')]
    h.done('pick_init')
    assert h.fsm.state == State.SEARCHING


def test_unexpected_action_report_is_ignored():
    h = Harness().searching()
    h.feed(det(), 5)
    assert h.done('place_left') == []
    assert h.fsm.state == State.PICKING


def test_start_while_running_is_a_noop():
    h = Harness().searching()
    assert h.start() == []
    assert h.fsm.state == State.SEARCHING


# ------------------------------------------------------------------ config

def test_bad_config_rejected():
    with pytest.raises(ValueError):
        Roi(10, 10, 0, 5)
    with pytest.raises(ValueError):
        FsmConfig(roi=ROI, place_actions={})
    with pytest.raises(ValueError):
        FsmConfig(roi=ROI, place_actions=PLACE, confirm_frames=0)


# ------------------------------------------------------------------ logger

def test_logger_roundtrip(tmp_path):
    h = Harness().searching()
    run_cycle(h, 'blue'); h.wait(2.0)
    run_cycle(h, 'red'); h.feed(det('red'), 40)

    log = PickLogger(str(tmp_path), session='t', extra={'detector': 'color_lab'})
    for r in h.records():
        log.write(r)
    log.close()
    # re-opening the same session appends without a second header
    log = PickLogger(str(tmp_path), session='t', extra={'detector': 'color_lab'})
    log.write(h.records()[0])
    log.close()

    rows = list(csv.DictReader(open(log.path)))
    assert [r['outcome'] for r in rows] == ['cleared', 'still_present', 'cleared']
    assert rows[0]['color'] == 'blue' and rows[0]['detector'] == 'color_lab'
    assert rows[0]['wall_time'] and float(rows[0]['dx']) == 0.0
