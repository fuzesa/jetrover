#!/usr/bin/env python3
"""Measure what the robot's own camera sees, in the detector's colour space.

Put one cube on the mark (arm in pick_init, camera stream running), then:

    python3 lab_probe.py green          # label is free text: red / green / blue / table
    python3 lab_probe.py palm --center  # hand-held demo: patch at the image centre

Grabs 30 frames and runs them through the same pipeline as the vendor colour
detector (RGB -> BGR -> LAB, 3x3 Gaussian blur). It then finds the object by
itself: inside the detector's search window it takes whatever differs in colour
from the dominant background, keeps the biggest blob and shaves off its rim.
So the cube only has to be somewhere inside the window, and the statistics
cover its lit top *and* its shaded sides - the range a threshold must span.
With the label "surface" (or if no object is found) the whole window is
measured instead. Prints L/A/B statistics next to the thresholds currently in
lab_config.yaml and saves an annotated snapshot (object outlined in white).

Not installed by colcon; run it straight from the source tree.
"""
import os
import sys

import cv2
import numpy as np

LAB_CONFIG = '/home/ubuntu/share/lab_tool/lab_config.yaml'
OUT_DIR = '/home/ubuntu/share/tmp/jetrover_logs'
PICK_WINDOW = (280, 360, 70, 150)   # x_min, x_max, y_min, y_max (sorting.yaml roi)
MARGIN = 20             # the detector searches the pick window grown by this
CHROMA_DIST = 18        # A/B distance from the background that counts as "object"
MIN_AREA = 150          # px; smaller blobs are ignored
FRAMES = 30


CENTER_HALF = 15        # --center: 30x30 patch at the image centre


def center_patch(lab):
    h, w = lab.shape[:2]
    cy, cx = h // 2, w // 2
    return lab[cy - CENTER_HALF:cy + CENTER_HALF, cx - CENTER_HALF:cx + CENTER_HALF].reshape(-1, 3)


def to_detector_lab(rgb):
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    return bgr, cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB), (3, 3), 3)


def search_window(shape):
    x0, x1, y0, y1 = PICK_WINDOW
    h, w = shape[:2]
    return max(x0 - MARGIN, 0), min(x1 + MARGIN, w), max(y0 - MARGIN, 0), min(y1 + MARGIN, h)


def object_mask(lab):
    """Mask (window-sized) of the biggest blob whose colour differs from the
    window's dominant background, rim removed. None if nothing qualifies."""
    x0, x1, y0, y1 = search_window(lab.shape)
    win = lab[y0:y1, x0:x1].astype(np.int16)
    bg = np.median(win.reshape(-1, 3), axis=0)
    dist = np.hypot(win[..., 1] - bg[1], win[..., 2] - bg[2])
    mask = (dist > CHROMA_DIST).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n < 2:
        return None
    biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[biggest, cv2.CC_STAT_AREA] < MIN_AREA:
        return None
    blob = (labels == biggest).astype(np.uint8)
    return cv2.erode(blob, np.ones((5, 5), np.uint8))


def sample(lab, mask):
    x0, x1, y0, y1 = search_window(lab.shape)
    win = lab[y0:y1, x0:x1]
    return win[mask.astype(bool)] if mask is not None else win.reshape(-1, 3)


def summarize(pixels):
    """pixels: (N, 3) LAB samples -> {channel: (p1, median, p99)}"""
    return {name: tuple(int(round(v)) for v in np.percentile(pixels[:, i], (1, 50, 99)))
            for i, name in enumerate('LAB')}


def verdicts(stats, thresholds):
    """Which configured colours would the patch's bulk (p1..p99) pass?"""
    out = {}
    for color, rng in thresholds.items():
        fails = [f'{ch} {stats[ch][0]}-{stats[ch][2]} vs [{rng["min"][i]}, {rng["max"][i]}]'
                 for i, ch in enumerate('LAB')
                 if stats[ch][0] < rng['min'][i] or stats[ch][2] > rng['max'][i]]
        out[color] = fails
    return out


def report(label, stats, thresholds):
    print(f'\n=== {label} ===   (p1 / median / p99 over {FRAMES} frames)')
    for ch in 'LAB':
        print(f'  {ch}: {stats[ch][0]:4d} / {stats[ch][1]:4d} / {stats[ch][2]:4d}')
    for color, fails in verdicts(stats, thresholds).items():
        print(f'  {color:6s}: ' + ('PASSES' if not fails else 'fails on ' + '; '.join(fails)))
    print(f'  PASTE: {label} L={stats["L"]} A={stats["A"]} B={stats["B"]}')


def main():
    import rclpy
    import yaml
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    centre = '--center' in sys.argv
    label = args[0] if args else 'unlabelled'
    frames = 90 if centre else FRAMES
    with open(LAB_CONFIG) as fh:
        lab = yaml.safe_load(fh)['lab']['Stereo']
    thresholds = {c: lab[c] for c in ('red', 'green', 'blue') if c in lab}

    want_object = label != 'surface' and not centre
    rclpy.init()
    node = Node('lab_probe')
    samples, last = [], {}

    def on_image(msg):
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        bgr, lab_img = to_detector_lab(np.ascontiguousarray(rgb))
        if centre:
            samples.append(center_patch(lab_img))
            last.update(bgr=bgr, mask=None)
            return
        mask = object_mask(lab_img) if want_object else None
        samples.append(sample(lab_img, mask))
        last.update(bgr=bgr, mask=mask)

    node.create_subscription(Image, '/depth_cam/rgb/image_raw', on_image, 1)
    if centre:
        print(f'collecting {frames} frames (~3 s) for "{label}": hold it over the centre of the image,')
        print('covering the middle of the screen, and turn it slowly so every face is seen ...')
    else:
        print(f'collecting {frames} frames for "{label}" ...')
    while rclpy.ok() and len(samples) < frames:
        rclpy.spin_once(node, timeout_sec=1.0)
        if not samples:
            print('  (no image yet - is the launch running?)')

    report(label, summarize(np.vstack(samples)), thresholds)
    if centre:
        img = last['bgr'].copy()
        h, w = img.shape[:2]
        cv2.rectangle(img, (w // 2 - CENTER_HALF, h // 2 - CENTER_HALF),
                      (w // 2 + CENTER_HALF, h // 2 + CENTER_HALF), (255, 255, 255), 2)
        os.makedirs(OUT_DIR, exist_ok=True)
        path = os.path.join(OUT_DIR, f'probe_{label}.png')
        cv2.imwrite(path, img)
        print(f'  snapshot: {path}')
        node.destroy_node()
        rclpy.shutdown()
        return
    mask = last['mask']
    if want_object and mask is None:
        print('  NOTE: no object found in the window - these numbers are the bare surface')
    elif mask is not None:
        print(f'  object: {int(mask.sum())} px measured per frame')

    img = last['bgr'].copy()
    x0, x1, y0, y1 = PICK_WINDOW
    sx0, sx1, sy0, sy1 = search_window(img.shape)
    cv2.rectangle(img, (sx0, sy0), (sx1, sy1), (128, 128, 128), 1)
    cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 255), 1)
    if mask is not None:
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cv2.drawContours(img, contours, -1, (255, 255, 255), 1, offset=(sx0, sy0))
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f'probe_{label}.png')
    cv2.imwrite(path, img)
    print(f'  snapshot: {path}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
