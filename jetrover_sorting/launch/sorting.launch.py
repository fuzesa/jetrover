"""Bring up the vendor controller + depth camera + colour detector, then our
sorting node.

    ros2 launch jetrover_sorting sorting.launch.py
    ros2 launch jetrover_sorting sorting.launch.py autostart:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

VENDOR_SRC = '/home/ubuntu/ros2_ws/src'


def vendor_file(package, src_subdir, relative):
    """Vendor packages are used from source or from install depending on the
    need_compile variable in .typerc; accept whichever actually exists."""
    candidates = [os.path.join(VENDOR_SRC, src_subdir, relative)]
    try:
        candidates.insert(0, os.path.join(get_package_share_directory(package), relative))
    except Exception:  # noqa: BLE001 - package not installed, fall back to source
        pass
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(f'{relative} not found for vendor package "{package}": {candidates}')


def generate_launch_description():
    # The vendor launch files index os.environ['need_compile'] directly.
    os.environ.setdefault('need_compile', 'False')

    config = os.path.join(get_package_share_directory('jetrover_sorting'), 'config', 'sorting.yaml')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            vendor_file('controller', 'driver/controller', 'launch/controller.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            vendor_file('peripherals', 'peripherals', 'launch/depth_camera.launch.py'))),
        Node(
            package='example', executable='color_detect', output='screen',
            parameters=[vendor_file('example', 'example', 'config/roi.yaml'),
                        {'enable_display': False}]),
        Node(
            package='jetrover_sorting', executable='sorting', output='screen',
            parameters=[config, {'autostart': ParameterValue(autostart, value_type=bool)}],
            # give the arm time to finish its motion and park on Ctrl-C
            sigterm_timeout='20', sigkill_timeout='10'),
    ])
