#!/usr/bin/env python3
"""Write a copy of a place action group turned to another base angle.

The first servo in every row of an action group is the base rotation
(500 = straight ahead, 1000 units = 240 degrees, so 90 degrees = 375 units;
875 = the robot's own left, 125 = its right). This copies a place group and
sets that value in every row, so the carry, drop and return stay the same,
only turned. The source file is never modified. Nothing moves.

    python3 make_place_at.py                               # place_side at 875 (robot's left)
    python3 make_place_at.py --yaw 125                     # robot's right
    python3 make_place_at.py --name place_red --yaw 820 --source place_center_tank
    python3 make_place_at.py --raise 45                    # drop from 45 mm higher
"""
import argparse
import os
import shutil
import sqlite3
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from jetrover_sorting.arm_model import SHOULDER_HEIGHT, forward, inverse  # noqa: E402

ACTION_DIR = '/home/ubuntu/share/arm_pc/ActionGroups'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--name', default='place_side', help='new action group (default place_side)')
    ap.add_argument('--yaw', type=int, default=875, help='base servo value (default 875 = robot left)')
    ap.add_argument('--source', default='place_center_tank', help='group to copy (default place_center_tank)')
    ap.add_argument('--raise', dest='raise_mm', type=float, default=0.0,
                    help='drop this many mm higher, same reach and tilt (default 0)')
    args = ap.parse_args()
    if not 0 <= args.yaw <= 1000:
        sys.exit('yaw must be 0..1000')
    if args.name == args.source or args.name in ('place_center', 'place_left', 'place_right', 'pick'):
        sys.exit('refusing to overwrite an existing vendor or source group')
    src = os.path.join(ACTION_DIR, args.source + '.d6a')
    dst = os.path.join(ACTION_DIR, args.name + '.d6a')
    if not os.path.isfile(src):
        sys.exit(f'{src} not found (for place_center_tank, run make_tank_actions.py first)')

    shutil.copyfile(src, dst)
    db = sqlite3.connect(dst)
    cols = [c[1] for c in db.execute('PRAGMA table_info(ActionGroup)')]
    db.execute(f'UPDATE ActionGroup SET "{cols[2]}"=?', (args.yaw,))
    note = ''
    if args.raise_mm:
        shoulder = SHOULDER_HEIGHT[os.environ.get('MACHINE_TYPE', 'JetRover_Tank')]
        rows = [tuple(r) for r in db.execute('select * from ActionGroup')]
        heights = [forward(r[3:6], shoulder)[1] for r in rows]
        drop = rows[heights.index(min(heights))][3:6]
        reach, height, wrist = forward(drop, shoulder)
        new = tuple(int(round(v)) for v in inverse(reach, height + args.raise_mm / 1000.0, wrist, shoulder))
        for r in rows:
            if r[3:6] == drop:
                db.execute(f'UPDATE ActionGroup SET "{cols[3]}"=?, "{cols[4]}"=?, "{cols[5]}"=? WHERE "{cols[0]}"=?',
                           (*new, r[0]))
        note = (f', drop {drop} -> {new}: tip {height * 1000:.0f} -> '
                f'{forward(new, shoulder)[1] * 1000:.0f} mm above the ground')
    db.commit()
    print(f'wrote {dst} (base {args.yaw}, {(args.yaw - 500) * 0.24:+.0f} degrees from straight ahead{note}):')
    for r in db.execute('select * from ActionGroup'):
        print('  ', tuple(r))
    db.close()


if __name__ == '__main__':
    main()
