"""Append-only CSV log of pick attempts. One file per session, flushed after
every row so a crash or power cut loses nothing."""
from __future__ import annotations

import csv
import os
from dataclasses import asdict, fields
from datetime import datetime
from typing import Optional

from .fsm import PickRecord

_FLOAT_FMT = '{:.3f}'


class PickLogger:
    def __init__(self, directory: str, session: Optional[str] = None,
                 extra: Optional[dict[str, str]] = None):
        """extra: constant columns appended to every row (e.g. detector name,
        ROI), so logs from different experiments can be concatenated later."""
        os.makedirs(directory, exist_ok=True)
        session = session or datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(directory, f'picks_{session}.csv')
        self._extra = dict(extra or {})
        self._columns = (['wall_time'] + [f.name for f in fields(PickRecord)]
                         + list(self._extra))
        new_file = not os.path.exists(self.path)
        self._fh = open(self.path, 'a', newline='')
        self._writer = csv.DictWriter(self._fh, fieldnames=self._columns)
        if new_file:
            self._writer.writeheader()
            self._fh.flush()

    def write(self, record: PickRecord) -> None:
        row = {k: (_FLOAT_FMT.format(v) if isinstance(v, float) else v)
               for k, v in asdict(record).items()}
        row['wall_time'] = datetime.now().astimezone().isoformat(timespec='milliseconds')
        row.update(self._extra)
        self._writer.writerow(row)
        self._fh.flush()

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.close()


class DetectionTrace:
    """Optional raw per-frame trace (one row per detector message) for working
    out why confirmation is flaky. ~30 rows/s, flushed once a second."""

    COLUMNS = ['t', 'state', 'color', 'x', 'y', 'radius', 'why_not']

    def __init__(self, directory: str, session: Optional[str] = None):
        os.makedirs(directory, exist_ok=True)
        session = session or datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(directory, f'detections_{session}.csv')
        self._fh = open(self.path, 'a', newline='')
        self._writer = csv.writer(self._fh)
        if self._fh.tell() == 0:
            self._writer.writerow(self.COLUMNS)
        self._t0: Optional[float] = None
        self._last_flush = 0.0

    def write(self, now: float, state: str, det, why_not: str) -> None:
        if self._t0 is None:
            self._t0 = now
        t = f'{now - self._t0:.3f}'
        if det is None:
            self._writer.writerow([t, state, '', '', '', '', why_not])
        else:
            self._writer.writerow([t, state, det.color, f'{det.x:.0f}', f'{det.y:.0f}',
                                   f'{det.radius:.0f}', why_not])
        if now - self._last_flush > 1.0:
            self._last_flush = now
            self._fh.flush()

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.close()
