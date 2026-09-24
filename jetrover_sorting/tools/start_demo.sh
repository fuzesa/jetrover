#!/usr/bin/env bash
# Start the hand-held cube demo. Run on the robot HOST (not in the container):
#     ~/docker/tmp/jetrover/jetrover_sorting/tools/start_demo.sh
# Ctrl-C stops it; the arm returns to its look-out pose first.
set -e
echo '== stopping the vendor app service (frees camera and servos)'
sudo systemctl stop start_app_node.service || true
echo '== pinning CPU clocks'
sudo jetson_clocks || true
echo '== starting the container'
docker start jetrover >/dev/null
echo '== launching (Ctrl-C to stop)'
exec docker exec -it -u ubuntu -w /home/ubuntu jetrover zsh -ic \
    'source /home/ubuntu/ros2_ws/install/setup.zsh && cd ~ && ros2 launch jetrover_sorting exhibition.launch.py'
