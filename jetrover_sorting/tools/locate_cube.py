#!/usr/bin/env python3
"""Step 2, first measurement: where does the robot THINK the cube is?

Read-only apart from moving the arm to its viewing pose. With a cube sitting on
the known grasp mark, this runs the whole perception chain the closed-loop
grasp will depend on and compares the answer with the truth:

    cube pixel (vendor colour detector)
      -> depth at that pixel                         (depth camera)
      -> 3D point in the camera frame                (camera intrinsics)
      -> 3D point in the gripper frame               (vendor hand-eye matrix)
      -> 3D point in the arm's base frame            (vendor forward kinematics)

Truth: the recorded grasp point, 257 mm out at the pick yaw, on the table.

Needs, in other terminals:
    ros2 launch jetrover_sorting sorting.launch.py autostart:=false
    ros2 launch kinematics kinematics_node.launch.py

    python3 locate_cube.py
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from interfaces.msg import ColorDetect, ColorsInfo, ROI
from interfaces.srv import SetCircleROI, SetColorDetectParam
from kinematics_msgs.srv import GetRobotPose
from servo_controller.action_group_controller import ActionGroupController
from servo_controller_msgs.msg import ServosPosition

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from jetrover_sorting.arm_model import SHOULDER_HEIGHT, forward  # noqa: E402

ACTION_DIR = '/home/ubuntu/share/arm_pc/ActionGroups'
OUT_DIR = '/home/ubuntu/share/tmp/jetrover_logs'
SEARCH = (270, 390, 250, 350)            # x_min, x_max, y_min, y_max around the grasp mark
VIEW_POSE, GRASP_POSE = (700, 15, 215), (195, 300, 285)   # servos 2-4: pick_init, vendor grasp
PICK_YAW_DEG = (875 - 500) / 1000.0 * 240.0               # servo 1 = 875
CUBE = 0.030

# camera frame -> gripper frame, from the vendor's track_and_grab.py
HAND2CAM = np.array([[0.0, 0.0, 1.0, -0.101],
                     [-1.0, 0.0, 0.0, 0.011],
                     [0.0, -1.0, 0.0, 0.045],
                     [0.0, 0.0, 0.0, 1.0]])


def quat_to_mat(p, q):
    w, x, y, z = q.w, q.x, q.y, q.z
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [p.x, p.y, p.z]
    return m


def call(node, client, request, what):
    if not client.wait_for_service(timeout_sec=5.0):
        sys.exit(f'{what}: service not available (see the docstring for what must be running)')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is None:
        sys.exit(f'{what}: no response')
    return future.result()


def mm(v):
    return '(' + ', '.join(f'{c * 1000:7.1f}' for c in v) + ') mm'


def main():
    rclpy.init()
    node = Node('locate_cube')
    got = {}
    node.create_subscription(Image, '/depth_cam/depth/image_raw', lambda m: got.__setitem__('depth', m), 1)
    node.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', lambda m: got.__setitem__('info', m), 1)
    node.create_subscription(Image, '/depth_cam/rgb/image_raw', lambda m: got.__setitem__('rgb', m), 1)
    cubes = []
    node.create_subscription(ColorsInfo, '/color_detect/color_info',
                             lambda m: cubes.extend(c for c in m.data if c.radius >= 10), 1)

    servo_pub = node.create_publisher(ServosPosition, 'servo_controller', 1)
    t0 = time.monotonic()
    while servo_pub.get_subscription_count() == 0:
        if time.monotonic() - t0 > 5.0:
            sys.exit('servo controller is not listening - is the sorting launch running?')
        rclpy.spin_once(node, timeout_sec=0.1)
    print('Moving to the viewing pose (pick_init).')
    ActionGroupController(servo_pub, ACTION_DIR).run_action('pick_init')
    time.sleep(1.5)

    # 1. the vendor's forward kinematics, against our own model
    pose = call(node, node.create_client(GetRobotPose, '/kinematics/get_current_pose'),
                GetRobotPose.Request(), 'get_current_pose').pose
    end = quat_to_mat(pose.position, pose.orientation)
    shoulder = SHOULDER_HEIGHT[os.environ.get('MACHINE_TYPE', 'JetRover_Tank')]
    reach, height, _ = forward(VIEW_POSE, shoulder)
    print('\n[1] gripper tip in the arm base frame, viewing pose')
    print(f'    vendor FK : {mm(end[:3, 3])}   reach {np.hypot(end[0, 3], end[1, 3]) * 1000:.1f}')
    print(f'    our model : reach {reach * 1000:.1f}, height {height * 1000:.1f} mm')

    # 2. find the cube with the vendor detector
    roi = ROI()
    roi.x_min, roi.x_max, roi.y_min, roi.y_max = SEARCH
    req = SetCircleROI.Request()
    req.data = roi
    call(node, node.create_client(SetCircleROI, '/color_detect/set_circle_roi'), req, 'set_circle_roi')
    req = SetColorDetectParam.Request()
    for c in ('red', 'green', 'blue'):
        cd = ColorDetect()
        cd.color_name, cd.detect_type = c, 'circle'
        req.data.append(cd)
    set_param = node.create_client(SetColorDetectParam, '/color_detect/set_param')
    call(node, set_param, req, 'set_param')
    cubes.clear()
    deadline = time.monotonic() + 8.0
    while (len(cubes) < 30 or not {'depth', 'info', 'rgb'} <= got.keys()) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    call(node, set_param, SetColorDetectParam.Request(), 'set_param (idle)')
    if len(cubes) < 15:
        sys.exit('no cube detected near the grasp mark - is it on the mark?')
    if not {'depth', 'info'} <= got.keys():
        sys.exit('no depth image / camera_info received')
    u = float(np.median([c.x for c in cubes]))
    v = float(np.median([c.y for c in cubes]))
    r = float(np.median([c.radius for c in cubes]))
    print(f'\n[2] cube ({cubes[-1].color}) at pixel ({u:.0f}, {v:.0f}), radius {r:.0f}')

    # 3. depth there
    d = got['depth']
    depth = np.frombuffer(d.data, np.uint16).reshape(d.height, d.step // 2)[:, :d.width].astype(np.float32)
    ui, vi, h = int(round(u)), int(round(v)), 5
    patch = depth[vi - h:vi + h + 1, ui - h:ui + h + 1]
    valid = patch[(patch > 0) & (patch < 5000)]
    wide = depth[vi - 30:vi + 31, ui - 30:ui + 31]
    wide_valid = wide[(wide > 0) & (wide < 5000)]
    print(f'\n[3] depth image {d.width}x{d.height} ({d.encoding}); colour image {got["rgb"].width}x{got["rgb"].height}')
    if valid.size == 0:
        sys.exit('    no valid depth at the cube pixel (too close for the sensor, or holes) - see the snapshot')
    z = float(np.median(valid)) / 1000.0
    print(f'    at the cube pixel (11x11): median {z * 1000:.0f} mm, {valid.size}/{patch.size} valid')
    print(f'    61x61 around it: min {wide_valid.min():.0f}, 10th pct {np.percentile(wide_valid, 10):.0f}, '
          f'median {np.median(wide_valid):.0f} mm')

    # 4. into the camera frame, gripper frame, base frame (the vendor's chain, without its fudge terms)
    k = got['info'].k
    cam = np.array([(u - k[2]) * z / k[0], (v - k[5]) * z / k[4], z, 1.0])
    base = end @ HAND2CAM @ cam
    print(f'\n[4] intrinsics fx {k[0]:.1f} fy {k[4]:.1f} cx {k[2]:.1f} cy {k[5]:.1f}')
    print(f'    cube in camera frame : {mm(cam[:3])}')
    print(f'    cube in base frame   : {mm(base[:3])}')

    # 5. truth
    g_reach = forward(GRASP_POSE, shoulder)[0]
    yaw = np.radians(PICK_YAW_DEG)
    truth = np.array([g_reach * np.cos(yaw), g_reach * np.sin(yaw), CUBE])   # top face of the cube
    print(f'\n[5] expected (grasp mark, top face): {mm(truth)}')
    print(f'    error                           : {mm(base[:3] - truth)}   '
          f'|xy| {np.hypot(*(base[:2] - truth[:2])) * 1000:.1f} mm')
    print(f'\nPASTE: fk={np.round(end[:3, 3] * 1000, 1).tolist()} px=({u:.0f},{v:.0f}) z={z * 1000:.0f} '
          f'base={np.round(base[:3] * 1000, 1).tolist()} truth={np.round(truth * 1000, 1).tolist()}')

    # snapshot: depth as colours, with the detection drawn, to judge depth<->colour alignment
    vis = cv2.applyColorMap(np.clip(depth / 800.0 * 255.0, 0, 255).astype(np.uint8), cv2.COLORMAP_JET)
    vis[depth == 0] = 0
    cv2.circle(vis, (ui, vi), int(r), (255, 255, 255), 1)
    cv2.rectangle(vis, (SEARCH[0], SEARCH[2]), (SEARCH[1], SEARCH[3]), (200, 200, 200), 1)
    rgb = got['rgb']
    colour = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.step // 3, 3)[:, :rgb.width]
    colour = cv2.cvtColor(np.ascontiguousarray(colour), cv2.COLOR_RGB2BGR)
    cv2.circle(colour, (ui, vi), int(r), (255, 255, 255), 1)
    if colour.shape[:2] == vis.shape[:2]:
        vis = np.vstack([colour, vis])
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, 'locate_cube.png')
    cv2.imwrite(path, vis)
    print(f'snapshot: {path}  (colour on top, depth below, same circle on both)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
