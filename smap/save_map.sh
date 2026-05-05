#!/bin/bash
konsole --separate -e fish -c "rm -f my_robot_map.yaml my_robot_map.pgm; ros2 run nav2_map_server map_saver_cli -f my_robot_map; sleep 1; exit"
