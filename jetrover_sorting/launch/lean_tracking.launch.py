"""Bare minimum for arm-only demos: board driver, servo controller, kinematics,
and the depth camera with the streams nobody uses switched off (infrared, point
cloud). No odometry, IMU filter, EKF, robot description or colour detector.

Not installed; run it by path:
    ros2 launch ~/share/tmp/jetrover/jetrover_sorting/launch/lean_tracking.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

VENDOR_SRC = '/home/ubuntu/ros2_ws/src'


def vendor_file(package, src_subdir, relative):
    candidates = [os.path.join(VENDOR_SRC, src_subdir, relative)]
    try:
        candidates.insert(0, os.path.join(get_package_share_directory(package), relative))
    except Exception:  # noqa: BLE001
        pass
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(f'{relative} not found for "{package}": {candidates}')


def include(package, src_subdir, relative, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vendor_file(package, src_subdir, relative)),
        launch_arguments=args.items())


def generate_launch_description():
    os.environ.setdefault('need_compile', 'False')
    return LaunchDescription([
        include('ros_robot_controller', 'driver/ros_robot_controller', 'launch/ros_robot_controller.launch.py'),
        include('servo_controller', 'driver/servo_controller', 'launch/servo_controller.launch.py'),
        include('kinematics', 'driver/kinematics', 'launch/kinematics_node.launch.py'),
        include('peripherals', 'peripherals', 'launch/depth_camera.launch.py',
                enable_ir='false', enable_point_cloud='false'),
    ])
