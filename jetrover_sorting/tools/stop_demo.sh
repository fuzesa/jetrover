#!/usr/bin/env bash
# Stop the demo politely: the arm finishes a grab in progress, folds to its rest
# pose, then everything shuts down. Run on the robot HOST.
#
# Only track_sort is asked to stop; the servo driver must stay alive while the
# arm parks. When track_sort exits, exhibition.launch.py shuts the rest down.
running() { docker exec jetrover pgrep -f "$1" >/dev/null 2>&1; }

if ! running 'ros2 launch jetrover_sorting'; then
    echo 'The demo was not running.'
    exit 0
fi
echo 'Stopping... (the arm finishes what it is doing, then folds away)'
docker exec jetrover pkill -INT -f 'jetrover_sorting/track_sort' 2>/dev/null \
    || docker exec jetrover pkill -INT -f 'ros2 launch jetrover_sorting'
for _ in $(seq 40); do
    running 'ros2 launch jetrover_sorting' || { echo 'Stopped.'; exit 0; }
    sleep 1
done
echo 'Still running after 40 s, stopping the launch.'
docker exec jetrover pkill -INT -f 'ros2 launch jetrover_sorting'
sleep 10
running 'ros2 launch jetrover_sorting' && docker exec jetrover pkill -KILL -f 'jetrover_sorting|orbbec|servo_controller|ros_robot_controller|kinematics'
echo 'Stopped.'
