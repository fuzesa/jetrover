#!/usr/bin/env python3
"""Write adjusted copies of the vendor's pick/place action groups.

The vendor's action groups are shared by all JetRover chassis, but the Tank's
shoulder sits 10.7 mm higher than the Mecanum's (vendor constants), so on a
Tank the recorded grasp closes ~1 cm too high. This computes, from the arm
geometry, servo values that put the gripper at the same spot and tilt but
lower, and can tighten the gripper. Results go to NEW files with a suffix
(pick_tank, place_center_tank, ...); vendor files are never modified. Nothing
moves when you run this.

    python3 make_tank_actions.py                      # lower the grasp by 10.7 mm
    python3 make_tank_actions.py 13                   # ... by 13 mm
    python3 make_tank_actions.py 13 --close 570 --hold 620

Gripper scale: 200 is open; the vendor closes to 500 and carries at 560.
Larger = tighter. The carry value also appears in the place groups (until the
release), so those are rewritten too whenever --hold is given.

Then point config/sorting.yaml at the new names (pick_action, place_actions).
"""
import argparse
import os
import shutil
import sqlite3
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from jetrover_sorting.arm_model import SHOULDER_HEIGHT, forward, inverse  # noqa: E402

ACTION_DIR = '/home/ubuntu/share/arm_pc/ActionGroups'
PLACE_GROUPS = ('place_center', 'place_left', 'place_right')
VENDOR_CLOSE, VENDOR_HOLD = 500, 560
GRIPPER_LIMIT = 700     # refuse silly values; a stalled servo only gets hot


def adjust_pick(rows, lower_m, shoulder, close, hold):
    """rows: (index, ms, s1, s2, s3, s4, s5, gripper). The grasp pose is the
    lowest pose in the group; every row holding it gets the new servo values."""
    heights = [forward(r[3:6], shoulder)[1] for r in rows]
    grasp = rows[heights.index(min(heights))][3:6]
    reach, height, wrist = forward(grasp, shoulder)
    new = tuple(int(round(p)) for p in inverse(reach, height - lower_m, wrist, shoulder))
    out = [r[:3] + new + r[6:] if r[3:6] == grasp else r for r in rows]
    out = adjust_gripper(out, close, hold)
    return out, grasp, new, (reach, height, forward(new, shoulder)[1])


def adjust_gripper(rows, close, hold):
    swap = {VENDOR_CLOSE: close, VENDOR_HOLD: hold}
    return [r[:7] + (swap.get(r[7], r[7]),) for r in rows]


def rewrite(src_name, dst_name, fn):
    src, dst = (os.path.join(ACTION_DIR, n + '.d6a') for n in (src_name, dst_name))
    shutil.copyfile(src, dst)
    db = sqlite3.connect(dst)
    cols = [c[1] for c in db.execute('PRAGMA table_info(ActionGroup)')]
    rows = [tuple(r) for r in db.execute('select * from ActionGroup')]
    result = fn(rows)
    new_rows = result[0] if isinstance(result, tuple) else result
    sets = ', '.join(f'"{c}"=?' for c in cols[3:6] + [cols[7]])
    for r in new_rows:
        db.execute(f'UPDATE ActionGroup SET {sets} WHERE "{cols[0]}"=?', (r[3], r[4], r[5], r[7], r[0]))
    db.commit()
    print(f'wrote {dst}:')
    for r in db.execute('select * from ActionGroup'):
        print('  ', tuple(r))
    db.close()
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('lower_mm', nargs='?', type=float, default=10.7, help='lower the grasp by this much (default 10.7)')
    ap.add_argument('--close', type=int, default=VENDOR_CLOSE, help=f'gripper value when closing (vendor {VENDOR_CLOSE})')
    ap.add_argument('--hold', type=int, default=VENDOR_HOLD, help=f'gripper value while carrying (vendor {VENDOR_HOLD})')
    ap.add_argument('--suffix', default='_tank')
    args = ap.parse_args()
    if not args.suffix:
        sys.exit('refusing to overwrite the vendor files: --suffix must not be empty')
    if not 200 < args.close <= args.hold <= GRIPPER_LIMIT:
        sys.exit(f'need 200 < close <= hold <= {GRIPPER_LIMIT}')

    machine = os.environ.get('MACHINE_TYPE', 'JetRover_Tank')
    shoulder = SHOULDER_HEIGHT[machine]
    _, grasp, new, (reach, h0, h1) = rewrite(
        'pick', 'pick' + args.suffix,
        lambda rows: adjust_pick(rows, args.lower_mm / 1000.0, shoulder, args.close, args.hold))
    print(f'{machine}: grasp at reach {reach * 1000:.0f} mm, model tip height {h0 * 1000:.1f} -> {h1 * 1000:.1f} mm')
    print(f'servos 2,3,4: {grasp} -> {new};  gripper close {args.close}, hold {args.hold}\n')

    if args.hold != VENDOR_HOLD:
        for name in PLACE_GROUPS:
            rewrite(name, name + args.suffix, lambda rows: adjust_gripper(rows, args.close, args.hold))
        print("\nplace groups rewritten too - in config/sorting.yaml use:")
        print(f"    place_actions: {[n + args.suffix for n in PLACE_GROUPS]}")
    print(f"    pick_action: 'pick{args.suffix}'")


if __name__ == '__main__':
    main()
