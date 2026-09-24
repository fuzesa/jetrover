#!/usr/bin/env python3
"""Track-and-sort: follow a hand-held cube, take it, drop it in the box for
its colour.

A lean rewrite of the vendor's track_and_grab:
  - every colour frame is used (no depth/colour timestamp pairing); depth is
    cached and sampled only when it matters
  - all three colours at once, largest compact blob within reach wins
  - frame-rate independent following with gliding servo commands
  - stillness judged from the image, ~0.75 s, not 2 s of servo quiet
  - invalid depth means "wait", never "stop working"
  - one continuous reach, close, lift, then the place action group for the
    colour (the same boxes as the table demo)

Run on the lean stack:
    ros2 launch ~/share/tmp/jetrover/jetrover_sorting/launch/lean_tracking.launch.py
    ros2 run jetrover_sorting track_sort --ros-args --params-file <config/track_sort.yaml>
"""
from __future__ import annotations

import math
import os
import signal
import threading
import time
from typing import Optional

import numpy as np
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from servo_controller.action_group_controller import ActionGroupController
from servo_controller.bus_servo_control import set_servo_position
from servo_controller_msgs.msg import ServosPosition

from .display import Overlay, Texts, render
from .fsm import PickRecord
from .pick_log import PickLogger
from .tracking import (Blob, Follower, FollowerConfig, StillnessGate, TargetFilter,
                       detect_cubes, pick_target)

# camera frame -> gripper frame (vendor track_and_grab.py); validated on
# 2026-09-19 with tools/locate_cube.py: linear, repeatable, ~15 mm bias
HAND2CAM = np.array([[0.0, 0.0, 1.0, -0.101],
                     [-1.0, 0.0, 0.0, 0.011],
                     [0.0, -1.0, 0.0, 0.045],
                     [0.0, 0.0, 0.0, 1.0]])

LOOKOUT = ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500))   # vendor look-out pose


def quat_to_mat(p, q):
    w, x, y, z = q.w, q.x, q.y, q.z
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [p.x, p.y, p.z]
    return m


class TrackSortNode(Node):
    def __init__(self):
        super().__init__('track_sort')
        p = self._declare_params()
        self.colors = list(p['colors'])
        self.places = dict(zip(self.colors, p['place_actions']))
        if len(self.places) != len(self.colors):
            raise ValueError('"colors" and "place_actions" must have the same length')
        self.p = p
        with open(p['lab_config']) as fh:
            self.lab = yaml.safe_load(fh)['lab']['Stereo']
        missing = [a for a in self.places.values()
                   if not os.path.isfile(os.path.join(p['action_group_dir'], a + '.d6a'))]
        if missing:
            raise FileNotFoundError(f'action groups missing: {missing}')

        self.follower = Follower(FollowerConfig(
            yaw_init=LOOKOUT[0][1], pitch_init=LOOKOUT[3][1],
            gain=p['track_gain'], gain_pitch=p['track_gain_pitch'],
            max_rate=p['track_max_rate'], deadband=p['track_deadband']))
        self.gate = StillnessGate(p['still_seconds'], p['still_radius_px'], p['still_max_rate'])
        self.filter = TargetFilter(p['smoothing'], p['lost_hold'], p['depth_hold'])
        self.log = PickLogger(p['log_dir'], extra={'detector': 'track_sort'})
        self.get_logger().info(f'pick log: {self.log.path}')

        self._lock = threading.Lock()
        self._depth: Optional[np.ndarray] = None
        self._k: Optional[list] = None
        self._busy = False            # arm sequence in progress
        self._busy_color = None
        self._enabled = bool(p['autostart'])
        self._attempts = 0
        self._last_seen = 0.0
        self._last_status = 0.0
        self._frames = 0

        self.servo_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.actions = ActionGroupController(self.servo_pub, p['action_group_dir'])
        self.get_pose = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        self.set_pose = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.create_service(Trigger, '~/start', self._srv_start)
        self.create_service(Trigger, '~/stop', self._srv_stop)

        self._ready = False
        self.exit_requested = threading.Event()   # set by a double-click on the screen
        self._phase = None            # None | 'grabbing' | 'placing' while busy
        self._view = None             # (rgb, Overlay) for the display thread
        self._view_lock = threading.Lock()
        if p['display']:
            self._texts = Texts(p['text_searching'], p['text_following'], p['text_steady'],
                                p['text_grabbing'], p['text_placing'],
                                dict(zip(self.colors, p['text_colors'])))
            threading.Thread(target=self._display_loop, daemon=True, name='display').start()
        self.create_timer(0.5, self._bringup)

    def _declare_params(self) -> dict:
        d = {
            'colors': ['red', 'green', 'blue'],
            'place_actions': ['place_center_tank', 'place_left_tank', 'place_right_tank'],
            'action_group_dir': '/home/ubuntu/share/arm_pc/ActionGroups',
            'lab_config': '/home/ubuntu/share/lab_tool/lab_config.yaml',
            'log_dir': '/home/ubuntu/share/tmp/jetrover_logs',
            'autostart': True,
            'min_blob_area': 80, 'min_fill': 0.5, 'border_fill': 0.2,
            'min_range': 0.12, 'max_range': 0.45, 'size_min': 0.5, 'size_max': 1.6,
            'track_gain': 1200.0, 'track_gain_pitch': 1200.0, 'track_max_rate': 400.0, 'track_deadband': 0.02,
            'still_seconds': 0.75, 'still_radius_px': 12.0, 'still_max_rate': 80.0,
            'servo_duration_min': 0.08,
            'depth_offset': 0.03, 'x_offset': -0.01,   # vendor fudges: cube radius + bias, rgb/depth baseline
            'gripper_open': 200, 'gripper_close': 600,
            'reach_seconds': 1.2, 'lift': 0.03,
            'smoothing': 0.5, 'lost_hold': 0.3, 'depth_hold': 0.6,
            'park_action': 'init',        # played on shutdown; '' to stay in the look-out pose
            'display': False, 'display_fps': 12.0, 'display_fullscreen': True, 'display_mirror': True,
            'text_searching': 'Show me a cube!', 'text_following': 'I see a {color} cube',
            'text_steady': 'Hold it still...', 'text_grabbing': 'Got it!',
            'text_placing': 'The {color} cube goes in its box',
            'text_colors': ['red', 'green', 'blue'],
        }
        return {k: self.declare_parameter(k, v).value for k, v in d.items()}

    # -------------------------------------------------------------- bringup

    def _bringup(self) -> None:
        if self._ready:
            return
        waiting = [c.srv_name for c in (self.get_pose, self.set_pose) if not c.service_is_ready()]
        ldp = self.create_client(SetBool, '/depth_cam/set_ldp_enable')
        if waiting or not ldp.service_is_ready() or self.servo_pub.get_subscription_count() == 0:
            self.get_logger().info('waiting for kinematics / camera / servo controller ...')
            return
        self._ready = True
        req = SetBool.Request()
        req.data = False                       # let the depth sensor work at close range
        ldp.call_async(req)
        self._home()
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self._on_depth, 1)
        self.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', self._on_info, 1)
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self._on_rgb, 1)
        self.get_logger().info('tracking ' + ('enabled' if self._enabled else 'disabled (call ~/start)'))

    def _home(self) -> None:
        set_servo_position(self.servo_pub, 1.0, LOOKOUT + ((10, self.p['gripper_open']),))
        self.follower.reset()
        self.gate.reset()
        self.filter.reset()
        time.sleep(1.2)

    def _srv_start(self, request, response):
        self._enabled = True
        response.success, response.message = True, 'tracking enabled'
        return response

    def _srv_stop(self, request, response):
        self._enabled = False
        response.success, response.message = True, 'tracking disabled; current grab finishes'
        return response

    # ------------------------------------------------------------ callbacks

    def _on_depth(self, msg: Image) -> None:
        d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        with self._lock:
            self._depth = d.copy()

    def _on_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._k = list(msg.k)

    def _show(self, rgb, overlay: Overlay) -> None:
        if self.p['display']:
            with self._view_lock:
                self._view = (rgb, overlay)

    def _on_rgb(self, msg: Image) -> None:
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        if self._busy:
            self._show(rgb, Overlay(mode=self._phase or 'grabbing', color=self._busy_color))
            return
        if not self._enabled:
            self._show(rgb, Overlay())
            return
        now = time.monotonic()
        with self._lock:
            depth = self._depth
        blobs = detect_cubes(rgb, self.lab, self.colors, self.p['min_blob_area'], self.p['min_fill'],
                             border_fill=self.p['border_fill'])
        fx = self._k[0] if self._k is not None else None
        target = pick_target(blobs, depth, self.p['min_range'], self.p['max_range'],
                             fx, self.p['size_min'], self.p['size_max'])
        self._frames += 1
        raw_blob, raw_z = target if target is not None else (None, None)
        tracked = self.filter.update(raw_blob, raw_z, now)

        if tracked is None:
            self.follower.lost(now)
            self.gate.reset()
            self._status(now, 'searching')
            self._show(rgb, Overlay())
            return
        color, x, y, z = tracked
        if raw_blob is None:                    # brief dropout: hold still, keep the clock
            self._show(rgb, Overlay('following', color, x, y, 0.0, z, self.gate.progress))
            self.follower.lost(now)
            self._status(now, f'following {color} (held) z={z if z is None else round(z, 3)} '
                              f'still={self.gate.progress:.0%}')
            return
        self._last_seen = now
        ex = x / msg.width - 0.5
        ey = y / msg.height - 0.5
        yaw, pitch, dt = self.follower.update(ex, ey, now)
        set_servo_position(self.servo_pub, max(dt, self.p['servo_duration_min']), ((1, yaw), (4, pitch)))
        still = self.gate.update(x, y, self.follower.last_rate, now)
        self._status(now, f'following {color} z={z if z is None else round(z, 3)} '
                          f'still={self.gate.progress:.0%}')
        self._show(rgb, Overlay('following', color, x, y, raw_blob.radius, z, self.gate.progress))
        if still and z is not None and self._k is not None:
            self._busy_color, self._phase = color, 'grabbing'
            self._busy = True
            blob = Blob(color, x, y, raw_blob.radius, raw_blob.fill)
            threading.Thread(target=self._grab, args=(blob, z, now), daemon=True).start()

    def _status(self, now: float, text: str) -> None:
        if now - self._last_status > 0.5:
            self._last_status = now
            self.state_pub.publish(String(data=text))

    # ------------------------------------------------------------- grabbing

    def _call(self, client, req, timeout=3.0):
        fut = client.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                return None
            time.sleep(0.01)
        return fut.result()

    def _target_in_base(self, blob, z: float) -> Optional[np.ndarray]:
        pose = self._call(self.get_pose, GetRobotPose.Request())
        if pose is None:
            return None
        end = quat_to_mat(pose.pose.position, pose.pose.orientation)
        k = self._k
        z = z + self.p['depth_offset']
        cam = np.array([(blob.x - k[2]) * z / k[0] + self.p['x_offset'],
                        (blob.y - k[5]) * z / k[4], z, 1.0])
        return (end @ HAND2CAM @ cam)[:3]

    def _ik(self, position, pitch: float):
        req = SetRobotPose.Request()
        req.position = [float(v) for v in position]
        req.pitch = float(pitch)
        req.pitch_range = [-180.0, 180.0]
        req.resolution = 1.0
        res = self._call(self.set_pose, req)
        return list(res.pulse) if res is not None and res.pulse else None

    def _move(self, pulses, seconds: float, gripper: Optional[int] = None) -> None:
        cmd = tuple((i + 1, int(v)) for i, v in enumerate(pulses[:5]))
        if gripper is not None:
            cmd += ((10, int(gripper)),)
        set_servo_position(self.servo_pub, seconds, cmd)
        time.sleep(seconds + 0.1)

    def _grab(self, blob, z: float, t_commit: float) -> None:
        outcome, detail = 'error', ''
        target = None
        try:
            target = self._target_in_base(blob, z)
            if target is None:
                detail = 'no arm pose from kinematics'
                return
            reach = math.hypot(target[0], target[1])
            if not self.p['min_range'] <= reach <= self.p['max_range'] + 0.1 or target[2] < -0.02:
                outcome, detail = 'skipped', f'target out of envelope: {np.round(target * 1000).tolist()} mm'
                return
            pitch = 80.0 if target[2] < 0.2 else 30.0
            pulses = self._ik(target, pitch)
            if pulses is None:
                outcome, detail = 'skipped', 'no IK solution'
                return
            self.get_logger().info(f'grab {blob.color} at {np.round(target * 1000).tolist()} mm, servos {pulses[:5]}')
            self._move(pulses, self.p['reach_seconds'], gripper=self.p['gripper_open'])
            set_servo_position(self.servo_pub, 0.5, ((10, self.p['gripper_close']),))
            time.sleep(0.7)
            lifted = self._ik(target + np.array([0.0, 0.0, self.p['lift']]), pitch)
            if lifted:
                self._move(lifted, 0.8)
            self._phase = 'placing'
            set_servo_position(self.servo_pub, 1.0, LOOKOUT + ((10, self.p['gripper_close']),))
            time.sleep(1.2)
            self.actions.run_action(self.places[blob.color])
            outcome = 'placed'
        except Exception as exc:  # noqa: BLE001 - keep the demo alive
            detail = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(f'grab failed: {detail}')
        finally:
            self._attempts += 1
            self.get_logger().info(f'attempt {self._attempts}: {blob.color} -> {outcome} {detail}')
            tx, ty = (target[0], target[1]) if target is not None else (0.0, 0.0)
            self.log.write(PickRecord(
                attempt=self._attempts, color=blob.color, x=blob.x, y=blob.y, radius=blob.radius,
                dx=float(tx) * 1000, dy=float(ty) * 1000, frames_to_confirm=0,
                confirm_duration=self.p['still_seconds'], place_action=self.places[blob.color],
                cycle_duration=time.monotonic() - t_commit, outcome=outcome, detail=detail))
            self._home()
            self._phase = None
            self._busy = False

    def _display_loop(self) -> None:
        """All OpenCV GUI calls live in this one thread, at a capped rate, so a
        slow screen can only skip frames, never delay the tracking."""
        import cv2
        name = 'JetRover'
        def on_mouse(event, *_):
            if event == cv2.EVENT_LBUTTONDBLCLK and not self.exit_requested.is_set():
                self.get_logger().info('double-click on the screen: shutting down')
                self.exit_requested.set()

        try:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            if self.p['display_fullscreen']:
                cv2.setWindowProperty(name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.setMouseCallback(name, on_mouse)
        except cv2.error as exc:
            self.get_logger().warn(f'display disabled, no screen available: {exc}')
            return
        period = 1.0 / max(self.p['display_fps'], 1.0)
        while rclpy.ok() and not self.exit_requested.is_set():
            t0 = time.monotonic()
            with self._view_lock:
                view = self._view
            if view is not None:
                try:
                    cv2.imshow(name, render(view[0], view[1], self._texts, self.p['display_mirror']))
                except Exception as exc:  # noqa: BLE001 - the display must never take the demo down
                    self.get_logger().warn(f'display error: {exc}', throttle_duration_sec=5.0)
            cv2.waitKey(1)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except cv2.error:
            pass

    def park(self) -> None:
        """Let a grab in progress finish, then fold the arm to its rest pose."""
        self._enabled = False
        t0 = time.monotonic()
        while self._busy and time.monotonic() - t0 < 15.0:
            time.sleep(0.1)
        self._home()
        name = self.p['park_action']
        if name:
            if os.path.isfile(os.path.join(self.p['action_group_dir'], name + '.d6a')):
                self.get_logger().info(f'parking: {name}')
                try:
                    self.actions.run_action(name)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'park failed: {exc}')
            else:
                self.get_logger().warn(f'park action {name} not found, staying in the look-out pose')
        self.log.close()


def main() -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = TrackSortNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop.is_set() and not node.exit_requested.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.park()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
