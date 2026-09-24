#!/usr/bin/env bash
# Start the hand-held cube demo. Run on the robot HOST (not in the container):
#     ~/docker/tmp/jetrover/jetrover_sorting/tools/start_demo.sh
# Ctrl-C stops it; the arm returns to its look-out pose first.
set -e
echo '== stopping the vendor app service (frees camera and servos)'
sudo -n systemctl stop start_app_node.service || sudo systemctl stop start_app_node.service || true
echo '== pinning CPU clocks'
sudo -n jetson_clocks || sudo jetson_clocks || true
echo '== allowing the container to use the screen'
DISPLAY=:0 xhost +local: >/dev/null 2>&1 || echo '   (no screen found - running without display)'
echo '== starting the container'
docker start jetrover >/dev/null
echo '== stopping any demo that is already running'
bash "$(dirname "$0")/stop_demo.sh"
echo '== launching (Ctrl-C or the stop icon to stop)'
exec docker exec -it -e DISPLAY=:0 -u ubuntu -w /home/ubuntu jetrover zsh -ic \
    'source /home/ubuntu/ros2_ws/install/setup.zsh && cd ~ && ros2 launch jetrover_sorting exhibition.launch.py'
