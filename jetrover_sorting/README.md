# jetrover_sorting

Rework of HiWonder's colour-sorting demo for the JetRover (Jetson Nano, ROS 2
Humble inside the vendor container). Step 1 of 3: an explicit state machine,
the vendor bugs fixed, and every pick attempt logged. Detection is still the
vendor `color_detect` node; the arm still plays the vendor action groups.

## Layout

| File | Purpose |
|---|---|
| `jetrover_sorting/fsm.py` | The state machine. Pure Python, no ROS imports. |
| `jetrover_sorting/pick_log.py` | CSV logger, one file per session. |
| `jetrover_sorting/sorting_node.py` | Thin ROS shell: topics, services, arm worker. |
| `config/sorting.yaml` | All parameters, commented. |
| `launch/sorting.launch.py` | Vendor controller + depth camera + detector + this node. |
| `test/test_fsm.py` | Unit tests; run anywhere with `python3 -m pytest`. |

## Build (inside the container)

    ln -s /home/ubuntu/share/tmp/jetrover ~/ros2_ws/src/jetrover    # once
    cd ~/ros2_ws
    colcon build --symlink-install --packages-select jetrover_sorting
    source install/setup.zsh

## Run

    ros2 launch jetrover_sorting sorting.launch.py
    ros2 launch jetrover_sorting sorting.launch.py autostart:=false

    ros2 service call /sorting/start std_srvs/srv/Trigger   # also recovers from ERROR
    ros2 service call /sorting/stop  std_srvs/srv/Trigger   # finishes the current cycle first
    ros2 topic echo /sorting/state

Live view from another machine on the same ROS domain:
`ros2 run rqt_image_view rqt_image_view /sorting/image_annotated`

## Pick log

`/home/ubuntu/share/tmp/jetrover_logs/picks_<session>.csv` in the container,
which is `/home/hiwonder/docker/tmp/jetrover_logs/` on the host. One row per
attempt: colour, mean blob centre, offset from the ROI centre (`dx`, `dy`),
frames and time to confirm, cycle time, and an outcome:

- `cleared` - nothing of that colour left in view after the cycle
- `still_present` - the same colour is still there, so the grasp missed
- `unverified` - a stop was requested, verification skipped
- `error` - an arm action failed or timed out (`detail` says which)

`cleared` is a heuristic: a block knocked out of view also counts as cleared.

## ROI calibration

With the launch running (`autostart:=false`), run `python3 tools/calibrate_roi.py`
and paste the block it prints into `config/sorting.yaml`. (The vendor debug mode
`ros2 launch example color_sorting_node.launch.py debug:=true` also works, but
needs a display.)

## Hand-held cube demo (track_sort)

Follows a cube held in front of the camera, takes it, and drops it in the box
for its colour. Runs on the lean stack (no odometry, IMU, EKF, IR, point cloud):

    ros2 launch ~/share/tmp/jetrover/jetrover_sorting/launch/lean_tracking.launch.py
    ros2 run jetrover_sorting track_sort --ros-args \
        --params-file ~/share/tmp/jetrover/jetrover_sorting/config/track_sort.yaml

`/track_sort/start` and `/track_sort/stop` (std_srvs/Trigger) toggle tracking;
`/track_sort/state` reports what it is doing. Tuning lives in
`config/track_sort.yaml`; the following gains are the first thing to adjust if
the arm hunts (lower `track_gain`) or lags (raise it).

The `_tank` action groups are generated, not committed:
`python3 tools/make_tank_actions.py 13 --close 570 --hold 620`.

## Exhibition: one command

On the robot host (not in the container):

    ~/docker/tmp/jetrover/jetrover_sorting/tools/start_demo.sh

It stops the vendor app service, pins the CPU clocks, starts the container and
runs `exhibition.launch.py` (lean stack + track_sort). Ctrl-C stops it.
After a code or config change: rsync from the Mac, then run it again. New
files (like a new launch file) need one `colcon build` in the container first.
