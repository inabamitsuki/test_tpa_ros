#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
#  slam_map.sh  –  Build yahboom_nav then start the SLAM mapping session
#
#  Edit WORKSPACE below to match where you extracted yahboom_nav_fixed_patched.
#
#  When mapping is done, save the map:
#    ros2 run nav2_map_server map_saver_cli -f ~/smap/my_robot_map
#
#  Then launch autonomous navigation:
#    ros2 launch yahboom_nav auto_nav.launch.py \
#      map:=/home/mitsuki/smap/my_robot_map.yaml
# ─────────────────────────────────────────────────────────────────────────────

WORKSPACE=~/yahboom_nav_fixed_patched   # <-- change if you put it elsewhere

konsole --separate -e bash -c "
set -e

source /opt/ros/humble/setup.bash
source ~/uros_ws/install/setup.bash        2>/dev/null || true
source ~/yahboomcar_ws/install/setup.bash  2>/dev/null || true

echo '=== Building yahboom_nav ==='
cd $WORKSPACE
colcon build --packages-select yahboom_nav --symlink-install
source install/setup.bash

echo '=== Launching SLAM ==='
ros2 launch yahboom_nav map_slamtoolbox_launch.py

echo ''
echo '=== PROCESS ENDED (press Enter to close) ==='
read
"
