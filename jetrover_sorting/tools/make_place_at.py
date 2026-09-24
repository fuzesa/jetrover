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
"""
import argparse
import os
import shutil
import sqlite3
import sys

ACTION_DIR = '/home/ubuntu/share/arm_pc/ActionGroups'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--name', default='place_side', help='new action group (default place_side)')
    ap.add_argument('--yaw', type=int, default=875, help='base servo value (default 875 = robot left)')
    ap.add_argument('--source', default='place_center_tank', help='group to copy (default place_center_tank)')
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
    db.commit()
    print(f'wrote {dst} (base {args.yaw}, {(args.yaw - 500) * 0.24:+.0f} degrees from straight ahead):')
    for r in db.execute('select * from ActionGroup'):
        print('  ', tuple(r))
    db.close()


if __name__ == '__main__':
    main()
