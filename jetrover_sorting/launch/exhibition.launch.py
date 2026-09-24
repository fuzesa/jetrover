"""Everything for the hand-held cube demo in one launch: the lean vendor stack
(board driver, servos, kinematics, camera) plus track_sort.

    ros2 launch jetrover_sorting exhibition.launch.py
    ros2 launch jetrover_sorting exhibition.launch.py autostart:=false

Ctrl-C lets a grab in progress finish and returns the arm to its look-out pose.
When track_sort exits (for example after a double-click on its screen), the
whole launch shuts down with it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('jetrover_sorting')
    track_sort = Node(
        package='jetrover_sorting', executable='track_sort', name='track_sort', output='screen',
        parameters=[os.path.join(share, 'config', 'track_sort.yaml'),
                    {'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool)}],
        sigterm_timeout='20', sigkill_timeout='10')
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'lean_tracking.launch.py'))),
        track_sort,
        RegisterEventHandler(OnProcessExit(
            target_action=track_sort,
            on_exit=[EmitEvent(event=Shutdown(reason='track_sort exited'))])),
    ])
