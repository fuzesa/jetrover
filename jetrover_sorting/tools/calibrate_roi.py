#!/usr/bin/env python3
"""Find where the arm's fixed "pick" motion actually grasps, in image pixels.

Headless port of the vendor's debug mode (no GUI window needed):

  1. the arm moves to the grasp pose (action group "pick_debug") and stays there
  2. you put a cube on the table right between the open fingers, press Enter
  3. the arm returns to the viewing pose (pick_init)
  4. the cube's position in the image is measured over 60 frames and printed as
     a ready-to-paste `roi:` block for config/sorting.yaml

Needs the launch running with autostart:=false (controller, camera, detector up;
the sorting node idle). Run from the source tree:

    python3 calibrate_roi.py            # +-10 px window, like the vendor
    python3 calibrate_roi.py 15         # +-15 px
"""
import os
import statistics
import sys
import time

import rclpy
from rclpy.node import Node

from interfaces.msg import ColorDetect, ColorsInfo, ROI
from interfaces.srv import SetCircleROI, SetColorDetectParam
from servo_controller.action_group_controller import ActionGroupController
from servo_controller_msgs.msg import ServosPosition

ACTION_DIR = '/home/ubuntu/share/arm_pc/ActionGroups'
COLORS = ('red', 'green', 'blue')
FRAMES = 60
MIN_RADIUS = 10


def call(node, client, request, what):
    if not client.wait_for_service(timeout_sec=5.0):
        sys.exit(f'{what}: service not available - is the launch running?')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is None or not future.result().success:
        sys.exit(f'{what}: call failed')


def main():
    half = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    for name in ('pick_debug', 'pick_init'):
        if not os.path.isfile(os.path.join(ACTION_DIR, name + '.d6a')):
            sys.exit(f'action group {name}.d6a not found in {ACTION_DIR}')

    rclpy.init()
    node = Node('calibrate_roi')
    servo_pub = node.create_publisher(ServosPosition, 'servo_controller', 1)
    arm = ActionGroupController(servo_pub, ACTION_DIR)
    t0 = time.monotonic()   # the first servo command is lost if nobody is listening yet
    while servo_pub.get_subscription_count() == 0:
        if time.monotonic() - t0 > 5.0:
            sys.exit('servo controller is not listening - is the launch running?')
        rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.3)

    print('Moving to the grasp pose. Keep clear of the arm.')
    arm.run_action('pick_debug')
    input('\nPut ONE cube on the table between the gripper fingers, where they would\n'
          'close on it. Take your hand away, then press Enter ... ')
    print('Returning to the viewing pose.')
    arm.run_action('pick_init')
    time.sleep(2.0)

    roi = ROI()
    roi.x_min, roi.x_max, roi.y_min, roi.y_max = 0, 640, 0, 360   # search the whole frame
    req = SetCircleROI.Request()
    req.data = roi
    call(node, node.create_client(SetCircleROI, '/color_detect/set_circle_roi'), req, 'set_circle_roi')
    req = SetColorDetectParam.Request()
    for c in COLORS:
        cd = ColorDetect()
        cd.color_name, cd.detect_type = c, 'circle'
        req.data.append(cd)
    set_param = node.create_client(SetColorDetectParam, '/color_detect/set_param')
    call(node, set_param, req, 'set_param')

    seen = []

    def on_colors(msg: ColorsInfo):
        hits = [c for c in msg.data if c.color in COLORS and c.radius >= MIN_RADIUS]
        if hits:
            best = max(hits, key=lambda c: c.radius)
            seen.append((best.color, best.x, best.y, best.radius))

    node.create_subscription(ColorsInfo, '/color_detect/color_info', on_colors, 1)
    print(f'Measuring {FRAMES} frames ...')
    deadline = time.monotonic() + 15.0
    while len(seen) < FRAMES and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    call(node, set_param, SetColorDetectParam.Request(), 'set_param (idle)')

    if len(seen) < FRAMES // 2:
        sys.exit(f'Only {len(seen)} detections in 15 s - the cube is not visible from the '
                 'viewing pose (or its colour is not being detected).')

    xs, ys, rs = ([s[i] for s in seen] for i in (1, 2, 3))
    cx, cy, r = round(statistics.median(xs)), round(statistics.median(ys)), round(statistics.median(rs))
    colors = sorted({s[0] for s in seen})
    print(f'\ncube: {"/".join(colors)}  centre ({cx}, {cy})  radius {r}  '
          f'spread x {min(xs)}-{max(xs)}, y {min(ys)}-{max(ys)}  ({len(seen)} frames)')
    if max(xs) - min(xs) > 6 or max(ys) - min(ys) > 6 or len(colors) > 1:
        print('WARNING: unstable detection - treat this result with suspicion.')
    print('\nPaste into config/sorting.yaml (on the Mac), then rsync:\n')
    print('    roi:')
    print(f'      x_min: {cx - half}\n      x_max: {cx + half}')
    print(f'      y_min: {cy - half}\n      y_max: {cy + half}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
