"""Sorting state machine.

Deliberately free of any ROS imports so it can be unit-tested anywhere.
The ROS node feeds it events (detections, finished arm actions, clock ticks)
and executes the commands it returns. All times are seconds on a monotonic
clock supplied by the caller; the FSM never reads a clock itself.

    IDLE -> HOMING -> SEARCHING <-> CONFIRMING -> PICKING -> PLACING
                          ^                                     |
                          +------ VERIFYING <---- HOMING <------+

Any arm action that fails or times out lands in ERROR; start() recovers.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Union


class State(Enum):
    IDLE = 'idle'
    HOMING = 'homing'
    SEARCHING = 'searching'
    CONFIRMING = 'confirming'
    PICKING = 'picking'
    PLACING = 'placing'
    VERIFYING = 'verifying'
    ERROR = 'error'


# Outcomes written to the pick log.
OUTCOME_CLEARED = 'cleared'              # object gone from view after the cycle
OUTCOME_STILL_PRESENT = 'still_present'  # same colour still visible -> grasp missed
OUTCOME_UNVERIFIED = 'unverified'        # stop requested, verification skipped
OUTCOME_ERROR = 'error'                  # an arm action failed or timed out


@dataclass(frozen=True)
class Detection:
    color: str
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class Roi:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self):
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError(f'degenerate ROI: {self}')

    def contains(self, x: float, y: float) -> bool:
        return self.x_min < x < self.x_max and self.y_min < y < self.y_max

    @property
    def center(self) -> tuple[float, float]:
        return (self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0


@dataclass
class FsmConfig:
    roi: Roi
    # colour -> action group that drops it in the right bin
    place_actions: dict[str, str]
    pick_action: str = 'pick'
    home_action: str = 'pick_init'
    # Confirmation: this many qualifying frames in a row before committing.
    confirm_frames: int = 30
    # Consecutive non-qualifying frames tolerated inside a streak (detector
    # dropouts). A *different colour* in the ROI always resets immediately.
    miss_tolerance: int = 2
    # The object must sit still: centre may not drift further than this from
    # where the streak began (a hand still placing the block fails this).
    max_jitter_px: float = 8.0
    min_radius: float = 10.0
    # No detection message at all for this long -> abandon the streak.
    detection_timeout: float = 0.5
    # After homing, wait for the camera to settle, then watch for leftovers.
    settle_time: float = 0.7
    verify_window: float = 1.0
    verify_hits: int = 3
    action_timeout: float = 30.0

    def __post_init__(self):
        if self.confirm_frames < 1:
            raise ValueError('confirm_frames must be >= 1')
        if not self.place_actions:
            raise ValueError('place_actions must not be empty')


@dataclass(frozen=True)
class RunAction:
    """Ask the arm worker to play one action group, then report back."""
    name: str


@dataclass(frozen=True)
class PickRecord:
    attempt: int
    color: str
    x: float                  # mean centre over the confirmation streak
    y: float
    radius: float
    dx: float                 # offset of that centre from the ROI centre
    dy: float
    frames_to_confirm: int    # frames seen from first sighting to commit
    confirm_duration: float
    place_action: str
    cycle_duration: float     # commit -> outcome
    outcome: str
    detail: str = ''


@dataclass(frozen=True)
class AttemptFinished:
    record: PickRecord


@dataclass(frozen=True)
class StreakLost:
    """A confirmation streak was abandoned. Purely diagnostic: tells you *why*
    the FSM keeps falling back to SEARCHING."""
    reason: str               # see SortingFsm.why_not, plus 'moved',
                              # 'color_changed', 'detector_silent'
    hits: int
    color: str
    last: Optional[Detection]  # the frame that broke it (None = nothing seen)


Command = Union[RunAction, AttemptFinished, StreakLost]


@dataclass
class _Streak:
    color: str
    t0: float
    x0: float
    y0: float
    hits: int = 0
    misses: int = 0           # consecutive
    frames: int = 0           # every frame since the streak began
    sum_x: float = 0.0
    sum_y: float = 0.0
    sum_r: float = 0.0

    def add(self, d: Detection) -> None:
        self.hits += 1
        self.misses = 0
        self.sum_x += d.x
        self.sum_y += d.y
        self.sum_r += d.radius


@dataclass
class _Cycle:
    attempt: int
    color: str
    x: float
    y: float
    radius: float
    frames: int
    confirm_duration: float
    place_action: str
    t_commit: float


class SortingFsm:
    def __init__(self, config: FsmConfig):
        self.cfg = config
        self.state = State.IDLE
        self.error_detail = ''
        self._attempts = 0
        self._streak: Optional[_Streak] = None
        self._cycle: Optional[_Cycle] = None
        self._pending_action: Optional[str] = None
        self._action_deadline = math.inf
        self._last_detection_t = -math.inf
        self._verify_t0 = 0.0
        self._verify_hits = 0
        self._stop_pending = False

    # ------------------------------------------------------------------ API

    @property
    def busy(self) -> bool:
        """True while the arm is (or should be) moving."""
        return self._pending_action is not None

    @property
    def attempts(self) -> int:
        return self._attempts

    @property
    def streak_progress(self) -> tuple[int, int]:
        hits = self._streak.hits if self._streak else 0
        return hits, self.cfg.confirm_frames

    def start(self, now: float) -> list[Command]:
        if self.state not in (State.IDLE, State.ERROR):
            self._stop_pending = False  # a start cancels a queued stop
            return []
        self._reset_transients()
        self.error_detail = ''
        self.state = State.HOMING
        return [self._run(self.cfg.home_action, now)]

    def stop(self, now: float) -> list[Command]:
        """Stop sorting. A cycle already under way is finished first, so the
        arm never freezes holding a block."""
        if self.state in (State.IDLE, State.ERROR):
            self.state = State.IDLE
            return []
        if self.state in (State.SEARCHING, State.CONFIRMING):
            self._reset_transients()
            self.state = State.IDLE
            return []
        if self.state == State.VERIFYING:
            return self._finish_cycle(now, OUTCOME_UNVERIFIED, to_idle=True)
        self._stop_pending = True  # HOMING / PICKING / PLACING
        return []

    def on_detection(self, det: Optional[Detection], now: float) -> list[Command]:
        """Call once per detector frame; det=None means 'nothing seen'."""
        self._last_detection_t = now
        if self.state == State.SEARCHING:
            if self._qualifies(det):
                self._begin_streak(det, now)
                return self._maybe_commit(now)
            return []
        if self.state == State.CONFIRMING:
            return self._confirming(det, now)
        if self.state == State.VERIFYING:
            return self._verifying(det, now)
        return []  # arm is moving, camera view is meaningless

    def on_action_done(self, name: str, success: bool, now: float,
                       detail: str = '') -> list[Command]:
        if name != self._pending_action:
            return []  # stale or unexpected report
        self._pending_action = None
        self._action_deadline = math.inf
        if not success:
            return self._fail(now, detail or f'action "{name}" failed')

        if self.state == State.PICKING:
            self.state = State.PLACING
            return [self._run(self._cycle.place_action, now)]
        if self.state == State.PLACING:
            self.state = State.HOMING
            return [self._run(self.cfg.home_action, now)]
        if self.state == State.HOMING:
            if self._cycle is None:  # initial homing after start()
                if self._stop_pending:
                    self._reset_transients()
                    self.state = State.IDLE
                else:
                    self.state = State.SEARCHING
                return []
            if self._stop_pending:
                return self._finish_cycle(now, OUTCOME_UNVERIFIED, to_idle=True)
            self.state = State.VERIFYING
            self._verify_t0 = now
            self._verify_hits = 0
            return []
        return []

    def tick(self, now: float) -> list[Command]:
        """Call periodically; drives everything that depends on time alone."""
        if self._pending_action is not None and now > self._action_deadline:
            name = self._pending_action
            self._pending_action = None
            self._action_deadline = math.inf
            return self._fail(now, f'action "{name}" timed out')
        if (self.state == State.CONFIRMING
                and now - self._last_detection_t > self.cfg.detection_timeout):
            lost = self._lost('detector_silent', None)
            self._streak = None
            self.state = State.SEARCHING
            return [lost]
        if (self.state == State.VERIFYING
                and now - self._verify_t0 > self.cfg.settle_time + self.cfg.verify_window):
            return self._finish_cycle(now, OUTCOME_CLEARED)
        return []

    # ------------------------------------------------------------ internals

    def why_not(self, det: Optional[Detection]) -> str:
        """Empty string if the detection qualifies, else the reason it doesn't."""
        if det is None:
            return 'no_detection'
        if det.color not in self.cfg.place_actions:
            return 'unsortable_color'
        if det.radius < self.cfg.min_radius:
            return 'too_small'
        if not self.cfg.roi.contains(det.x, det.y):
            return 'outside_roi'
        return ''

    def _qualifies(self, det: Optional[Detection]) -> bool:
        return not self.why_not(det)

    def _lost(self, reason: str, det: Optional[Detection]) -> StreakLost:
        s = self._streak
        return StreakLost(reason=reason, hits=s.hits, color=s.color, last=det)

    def _begin_streak(self, det: Detection, now: float) -> None:
        self._streak = _Streak(color=det.color, t0=now, x0=det.x, y0=det.y, frames=1)
        self._streak.add(det)
        self.state = State.CONFIRMING

    def _confirming(self, det: Optional[Detection], now: float) -> list[Command]:
        s = self._streak
        reason = self.why_not(det)
        if reason:
            s.frames += 1
            s.misses += 1
            if s.misses > self.cfg.miss_tolerance:
                lost = self._lost(reason, det)
                self._streak = None
                self.state = State.SEARCHING
                return [lost]
            return []
        moved = math.hypot(det.x - s.x0, det.y - s.y0) > self.cfg.max_jitter_px
        if det.color != s.color or moved:
            lost = self._lost('color_changed' if det.color != s.color else 'moved', det)
            self._begin_streak(det, now)  # restart from this frame
            return [lost] + self._maybe_commit(now)
        s.frames += 1
        s.add(det)
        return self._maybe_commit(now)

    def _maybe_commit(self, now: float) -> list[Command]:
        s = self._streak
        if s.hits < self.cfg.confirm_frames:
            return []
        self._attempts += 1
        self._cycle = _Cycle(
            attempt=self._attempts, color=s.color,
            x=s.sum_x / s.hits, y=s.sum_y / s.hits, radius=s.sum_r / s.hits,
            frames=s.frames, confirm_duration=now - s.t0,
            place_action=self.cfg.place_actions[s.color], t_commit=now)
        self._streak = None
        self.state = State.PICKING
        return [self._run(self.cfg.pick_action, now)]

    def _verifying(self, det: Optional[Detection], now: float) -> list[Command]:
        if now - self._verify_t0 < self.cfg.settle_time:
            return []
        # The detector only looks in a small window around the pick ROI, so the
        # same colour showing up anywhere in it means the block is still there.
        if (det is not None and det.color == self._cycle.color
                and det.radius >= self.cfg.min_radius):
            self._verify_hits += 1
            if self._verify_hits >= self.cfg.verify_hits:
                return self._finish_cycle(now, OUTCOME_STILL_PRESENT)
        return []

    def _run(self, name: str, now: float) -> RunAction:
        self._pending_action = name
        self._action_deadline = now + self.cfg.action_timeout
        return RunAction(name)

    def _record(self, now: float, outcome: str, detail: str = '') -> AttemptFinished:
        c = self._cycle
        cx, cy = self.cfg.roi.center
        return AttemptFinished(PickRecord(
            attempt=c.attempt, color=c.color, x=c.x, y=c.y, radius=c.radius,
            dx=c.x - cx, dy=c.y - cy, frames_to_confirm=c.frames,
            confirm_duration=c.confirm_duration, place_action=c.place_action,
            cycle_duration=now - c.t_commit, outcome=outcome, detail=detail))

    def _finish_cycle(self, now: float, outcome: str, to_idle: bool = False) -> list[Command]:
        cmds: list[Command] = [self._record(now, outcome)]
        self._reset_transients()
        self.state = State.IDLE if to_idle else State.SEARCHING
        return cmds

    def _fail(self, now: float, detail: str) -> list[Command]:
        cmds: list[Command] = []
        if self._cycle is not None:
            cmds.append(self._record(now, OUTCOME_ERROR, detail))
        self._reset_transients()
        self.error_detail = detail
        self.state = State.ERROR
        return cmds

    def _reset_transients(self) -> None:
        self._streak = None
        self._cycle = None
        self._pending_action = None
        self._action_deadline = math.inf
        self._verify_hits = 0
        self._stop_pending = False
