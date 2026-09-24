#!/usr/bin/env bash
# Stop the demo politely: the arm finishes a grab in progress and returns to
# its look-out pose, then everything shuts down. Run on the robot HOST.
if docker exec jetrover pkill -INT -f 'ros2 launch jetrover_sorting' 2>/dev/null; then
    echo 'Stopping... (the arm finishes what it is doing first)'
    for _ in $(seq 25); do
        docker exec jetrover pgrep -f 'ros2 launch jetrover_sorting' >/dev/null 2>&1 || { echo 'Stopped.'; exit 0; }
        sleep 1
    done
    echo 'Still running after 25 s, forcing it.'
    docker exec jetrover pkill -KILL -f 'jetrover_sorting|orbbec|servo_controller|ros_robot_controller|kinematics' 2>/dev/null
else
    echo 'The demo was not running.'
fi
