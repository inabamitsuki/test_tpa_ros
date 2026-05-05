# Build
- cd yahboom_nav_fixed_patched
- colcon build
# source workspace
- source install/setup.bash      ( bash )
- bass source install/setup.bash ( fish )
# run
ros2 launch yahboom_nav auto_nav.launch.py map:=~/path/to/my_robot_map.yaml
