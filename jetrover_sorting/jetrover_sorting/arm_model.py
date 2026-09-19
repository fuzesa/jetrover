"""Planar geometry of the JetRover arm (shoulder, elbow, wrist pitch).

Pure Python, no ROS. Constants are the vendor's own, from
driver/kinematics/kinematics/transform.py: link lengths, the height of the
shoulder joint above the ground per chassis, and the servo pulse <-> joint
angle maps. Angles follow the vendor's modified-DH convention, in which a
positive joint angle pitches the next link *down*; theta2 = -90 deg is
straight up. The tool runs along the wrist-roll axis, i.e. 90 deg further
down than the last link's x axis.

Servo 1 (base yaw) and servo 5 (wrist roll) don't affect height or reach and
are left out. "Tip" is the end of the vendor's tool_link.
"""
from __future__ import annotations

from math import acos, atan2, cos, degrees, hypot, pi, radians, sin

LINK1 = 0.130
LINK2 = 0.130
TOOL = 0.055 + 0.117

SHOULDER_HEIGHT = {
    'JetRover_Mecanum': 0.22736,
    'JetRover_Acker': 0.05 + 0.0654868 + 0.0338648 + 0.0772047,
    'JetRover_Tank': 0.127 + 0.0338648 + 0.0772047,
}

# (pulse_min, pulse_max, pulse_mid, angle_at_min, angle_at_max, angle_at_mid) for servos 2, 3, 4
_MAPS = ((0, 1000, 500, 30, -210, -90), (0, 1000, 500, 120, -120, 0), (0, 1000, 500, 30, -210, -90))


def _to_angle(pulse: float, m) -> float:
    return (pulse - m[2]) / (m[1] - m[0]) * (m[4] - m[3]) + m[5]


def _to_pulse(angle: float, m) -> float:
    return (angle - m[5]) / (m[4] - m[3]) * (m[1] - m[0]) + m[2]


def forward(pulses, shoulder_height: float):
    """(servo2, servo3, servo4) -> (reach_m, tip_height_m, wrist_angle_deg).

    reach is horizontal distance from the shoulder axis; wrist_angle is the
    cumulative DH angle theta2+theta3+theta4 (tool pitch = -(wrist_angle+90))."""
    t2, t3, t4 = (radians(_to_angle(p, m)) for p, m in zip(pulses, _MAPS))
    c2, c3, c4 = t2, t2 + t3, t2 + t3 + t4
    reach = LINK1 * cos(c2) + LINK2 * cos(c3) + TOOL * cos(c4 + pi / 2)
    height = shoulder_height - LINK1 * sin(c2) - LINK2 * sin(c3) - TOOL * sin(c4 + pi / 2)
    return reach, height, degrees(c4)


def inverse(reach: float, height: float, wrist_angle_deg: float, shoulder_height: float):
    """Elbow-up solution -> (servo2, servo3, servo4) pulses as floats.
    Raises ValueError if the point is out of reach or a servo would leave 0..1000."""
    c4 = radians(wrist_angle_deg)
    wx = reach - TOOL * cos(c4 + pi / 2)
    wz = height + TOOL * sin(c4 + pi / 2) - shoulder_height
    d = hypot(wx, wz)
    cos_elbow = (d * d - LINK1 ** 2 - LINK2 ** 2) / (2 * LINK1 * LINK2)
    if not -1.0 <= cos_elbow <= 1.0:
        raise ValueError(f'wrist target {d * 1000:.0f} mm from the shoulder is out of reach')
    t3 = acos(cos_elbow)
    t2 = atan2(-wz, wx) - atan2(LINK2 * sin(t3), LINK1 + LINK2 * cos(t3))
    t4 = c4 - t2 - t3
    pulses = tuple(_to_pulse(degrees(t), m) for t, m in zip((t2, t3, t4), _MAPS))
    if not all(0 <= p <= 1000 for p in pulses):
        raise ValueError(f'servo pulse out of range: {[round(p) for p in pulses]}')
    return pulses
