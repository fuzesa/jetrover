#!/usr/bin/env python3
"""ROS 2 shell around SortingFsm.

Threading model: the FSM is only ever touched from the executor thread. The
single ArmWorker thread plays action groups (blocking) and reports back through
a queue that the 20 Hz tick timer drains. Nothing in a callback blocks.
"""
from __future__ import annotations

import os
import queue
import signal
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from interfaces.msg import ColorDetect, ColorsInfo, ROI
from interfaces.srv import SetCircleROI, SetColorDetectParam
from servo_controller.action_group_controller import ActionGroupController
from servo_controller_msgs.msg import ServosPosition

from .fsm import (AttemptFinished, Detection, FsmConfig, Roi, RunAction,
                  SortingFsm, State, StreakLost)
from .pick_log import DetectionTrace, PickLogger


class ArmWorker(threading.Thread):
    """Owns the arm. Plays one action group at a time, in order."""

    def __init__(self, controller: ActionGroupController, action_dir: str, events: queue.Queue):
        super().__init__(daemon=True, name='arm_worker')
        self._controller = controller
        self._dir = action_dir
        self._events = events
        self._jobs: queue.Queue = queue.Queue()
        self._lock = threading.Lock()
        self._outstanding = 0
        self._idle = threading.Event()
        self._idle.set()

    def submit(self, name: str) -> None:
        with self._lock:
            self._outstanding += 1
            self._idle.clear()
        self._jobs.put(name)

    def wait_idle(self, timeout: float) -> bool:
        return self._idle.wait(timeout)

    def play_blocking(self, name: str) -> tuple[bool, str]:
        path = os.path.join(self._dir, name + '.d6a')
        if not os.path.isfile(path):
            return False, f'action group not found: {path}'
        try:
            self._controller.run_action(name)
        except Exception as exc:  # noqa: BLE001 - vendor code, anything goes
            return False, f'{type(exc).__name__}: {exc}'
        return True, ''

    def run(self) -> None:
        while True:
            name = self._jobs.get()
            if name is None:
                return
            ok, detail = self.play_blocking(name)
            self._events.put((name, ok, detail))
            with self._lock:
                self._outstanding -= 1
                if self._outstanding == 0:
                    self._idle.set()

    def shutdown(self) -> None:
        self._jobs.put(None)


class SortingNode(Node):
    def __init__(self):
        super().__init__('sorting')
        p = self._declare_params()

        colors = list(p['colors'])
        places = list(p['place_actions'])
        if len(colors) != len(places):
            raise ValueError('"colors" and "place_actions" must have the same length')
        self._colors = colors
        self._roi = Roi(p['roi.x_min'], p['roi.x_max'], p['roi.y_min'], p['roi.y_max'])
        self._margin = int(p['detector_roi_margin'])
        self._park_action = p['park_action']
        self._autostart = bool(p['autostart'])
        self._debug_image = bool(p['publish_debug_image'])

        self.fsm = SortingFsm(FsmConfig(
            roi=self._roi, place_actions=dict(zip(colors, places)),
            pick_action=p['pick_action'], home_action=p['home_action'],
            confirm_frames=p['confirm_frames'], miss_tolerance=p['miss_tolerance'],
            max_jitter_px=p['max_jitter_px'], min_radius=p['min_radius'],
            detection_timeout=p['detection_timeout'], settle_time=p['settle_time'],
            verify_window=p['verify_window'], verify_hits=p['verify_hits'],
            action_timeout=p['action_timeout']))

        # Find out about a missing action group now, not with a block in the gripper.
        needed = [p['pick_action'], p['home_action'], *places]
        if self._park_action:
            needed.append(self._park_action)
        missing = [a for a in needed
                   if not os.path.isfile(os.path.join(p['action_group_dir'], a + '.d6a'))]
        if missing:
            raise FileNotFoundError(
                f'action groups missing from {p["action_group_dir"]}: {", ".join(missing)}')

        self._log = PickLogger(p['log_dir'], extra={
            'detector': p['detector_name'],
            'roi': f'{self._roi.x_min:g}-{self._roi.x_max:g}x{self._roi.y_min:g}-{self._roi.y_max:g}',
            'confirm_frames': str(p['confirm_frames'])})
        self.get_logger().info(f'pick log: {self._log.path}')
        self._trace = DetectionTrace(p['log_dir']) if p['trace_detections'] else None
        if self._trace:
            self.get_logger().info(f'detection trace: {self._trace.path}')

        self._events: queue.Queue = queue.Queue()
        servo_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self._worker = ArmWorker(
            ActionGroupController(servo_pub, p['action_group_dir']),
            p['action_group_dir'], self._events)
        self._worker.start()

        self._state_pub = self.create_publisher(String, '~/state', 10)
        self._image_pub = self.create_publisher(Image, '~/image_annotated', 1)
        self.create_subscription(ColorsInfo, '/color_detect/color_info', self._on_colors, 1)
        self.create_subscription(Image, '/color_detect/image_result', self._on_image, 1)
        self.create_service(Trigger, '~/start', self._srv_start)
        self.create_service(Trigger, '~/stop', self._srv_stop)

        self._controller_ready = self.create_client(Trigger, '/controller_manager/init_finish')
        self._set_roi = self.create_client(SetCircleROI, '/color_detect/set_circle_roi')
        self._set_colors = self.create_client(SetColorDetectParam, '/color_detect/set_param')

        self._last_state = None
        self._last_state_pub_t = 0.0
        self._bringup_t0 = time.monotonic()
        self._bringup_timer = self.create_timer(0.5, self._bringup)
        self.create_timer(0.05, self._tick)

    # ------------------------------------------------------------ parameters

    def _declare_params(self) -> dict:
        defaults = {
            'roi.x_min': 280, 'roi.x_max': 360, 'roi.y_min': 70, 'roi.y_max': 150,
            'detector_roi_margin': 20,
            'colors': ['red', 'green', 'blue'],
            'place_actions': ['place_center', 'place_left', 'place_right'],
            'pick_action': 'pick', 'home_action': 'pick_init', 'park_action': 'init',
            'action_group_dir': '/home/ubuntu/share/arm_pc/ActionGroups',
            'confirm_frames': 30, 'miss_tolerance': 2, 'max_jitter_px': 8.0,
            'min_radius': 10.0, 'detection_timeout': 0.5, 'settle_time': 0.7,
            'verify_window': 1.0, 'verify_hits': 3, 'action_timeout': 30.0,
            'autostart': True, 'publish_debug_image': True,
            'log_dir': '/home/ubuntu/share/tmp/jetrover_logs',
            'detector_name': 'color_lab',
            'trace_detections': False,
        }
        return {k: self.declare_parameter(k, v).value for k, v in defaults.items()}

    # --------------------------------------------------------------- bringup

    def _bringup(self) -> None:
        waiting = [c.srv_name for c in (self._controller_ready, self._set_roi, self._set_colors)
                   if not c.service_is_ready()]
        if waiting:
            if time.monotonic() - self._bringup_t0 > 5.0:
                self._bringup_t0 = time.monotonic()
                self.get_logger().warn('still waiting for: ' + ', '.join(waiting))
            return
        self._bringup_timer.cancel()
        self.get_logger().info('dependencies up')
        if self._autostart:
            self._start()

    # ------------------------------------------------------- start / stop

    def _start(self) -> None:
        if not (self._set_roi.service_is_ready() and self._set_colors.service_is_ready()):
            self.get_logger().error('color_detect services are not available')
            return
        roi = ROI()
        roi.x_min = int(self._roi.x_min) - self._margin
        roi.x_max = int(self._roi.x_max) + self._margin
        roi.y_min = int(self._roi.y_min) - self._margin
        roi.y_max = int(self._roi.y_max) + self._margin
        req = SetCircleROI.Request()
        req.data = roi
        self._set_roi.call_async(req).add_done_callback(self._after_set_roi)

    def _after_set_roi(self, future) -> None:
        if not self._call_ok(future, 'set_circle_roi'):
            return
        req = SetColorDetectParam.Request()
        for name in self._colors:
            cd = ColorDetect()
            cd.color_name = name
            cd.detect_type = 'circle'
            req.data.append(cd)
        self._set_colors.call_async(req).add_done_callback(self._after_set_colors)

    def _after_set_colors(self, future) -> None:
        if not self._call_ok(future, 'set_param'):
            return
        self.get_logger().info('detector configured, starting')
        self._execute(self.fsm.start(time.monotonic()))

    def _call_ok(self, future, what: str) -> bool:
        try:
            res = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'{what} call raised: {exc}')
            return False
        if res is None or not res.success:
            self.get_logger().error(f'{what} call failed')
            return False
        return True

    def _srv_start(self, request, response):
        self._start()
        response.success = True
        response.message = 'start requested'
        return response

    def _srv_stop(self, request, response):
        self._execute(self.fsm.stop(time.monotonic()))
        response.success = True
        response.message = f'state: {self.fsm.state.value}'
        return response

    # ------------------------------------------------------------- callbacks

    def _on_colors(self, msg: ColorsInfo) -> None:
        best = None
        for c in msg.data:
            if c.color in self._colors and (best is None or c.radius > best.radius):
                best = c
        det = Detection(best.color, float(best.x), float(best.y), float(best.radius)) if best else None
        now = time.monotonic()
        if self._trace:
            self._trace.write(now, self.fsm.state.value, det, self.fsm.why_not(det))
        self._execute(self.fsm.on_detection(det, now))

    def _tick(self) -> None:
        now = time.monotonic()
        while True:
            try:
                name, ok, detail = self._events.get_nowait()
            except queue.Empty:
                break
            if not ok:
                self.get_logger().error(f'arm action "{name}" failed: {detail}')
            self._execute(self.fsm.on_action_done(name, ok, now, detail))
        self._execute(self.fsm.tick(now))
        self._publish_state(now)

    def _execute(self, commands) -> None:
        for cmd in commands:
            if isinstance(cmd, RunAction):
                self.get_logger().info(f'arm: {cmd.name}')
                self._worker.submit(cmd.name)
            elif isinstance(cmd, StreakLost):
                d = cmd.last
                seen = f'{d.color} at ({d.x:.0f},{d.y:.0f}) r={d.radius:.0f}' if d else 'nothing seen'
                self.get_logger().info(
                    f'streak lost at {cmd.hits}/{self.fsm.cfg.confirm_frames} '
                    f'({cmd.color}): {cmd.reason} - {seen}')
            elif isinstance(cmd, AttemptFinished):
                r = cmd.record
                self._log.write(r)
                self.get_logger().info(
                    f'attempt {r.attempt}: {r.color} at ({r.x:.0f},{r.y:.0f}) '
                    f'offset ({r.dx:+.0f},{r.dy:+.0f}) -> {r.outcome} {r.detail}')

    def _publish_state(self, now: float) -> None:
        state = self.fsm.state
        if state is not self._last_state:
            prev = self._last_state
            self._last_state = state
            self.get_logger().info(f'state: {state.value}')
            if state is State.ERROR:
                self.get_logger().error(
                    f'{self.fsm.error_detail} - call ~/start to recover')
            if state is State.IDLE and prev is not None and self._set_colors.service_is_ready():
                self._set_colors.call_async(SetColorDetectParam.Request())  # idle the detector
        elif now - self._last_state_pub_t < 1.0:
            return
        self._last_state_pub_t = now
        self._state_pub.publish(String(data=state.value))

    def _on_image(self, msg: Image) -> None:
        if not self._debug_image or self._image_pub.get_subscription_count() == 0:
            return
        if msg.encoding not in ('bgr8', 'rgb8'):
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)
        img = np.ascontiguousarray(img[:, :msg.width])
        r, m = self._roi, self._margin
        cv2.rectangle(img, (int(r.x_min) - m, int(r.y_min) - m),
                      (int(r.x_max) + m, int(r.y_max) + m), (128, 128, 128), 1)
        cv2.rectangle(img, (int(r.x_min), int(r.y_min)), (int(r.x_max), int(r.y_max)),
                      (0, 255, 255), 2)
        hits, need = self.fsm.streak_progress
        label = self.fsm.state.value
        if self.fsm.state is State.CONFIRMING:
            label += f' {hits}/{need}'
        cv2.putText(img, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        out = Image()
        out.header = msg.header
        out.height, out.width = img.shape[:2]
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = out.width * 3
        out.data = img.tobytes()
        self._image_pub.publish(out)

    # -------------------------------------------------------------- shutdown

    def park(self) -> None:
        """Let the running action finish, then fold the arm away."""
        self.get_logger().info('shutting down: waiting for the arm')
        if not self._worker.wait_idle(timeout=12.0):
            self.get_logger().warn('arm still busy, skipping park')
        elif self._park_action:
            ok, detail = self._worker.play_blocking(self._park_action)
            if not ok:
                self.get_logger().warn(f'park failed: {detail}')
        self._worker.shutdown()
        self._log.close()
        if self._trace:
            self._trace.close()


def main() -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    node = SortingNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.park()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
