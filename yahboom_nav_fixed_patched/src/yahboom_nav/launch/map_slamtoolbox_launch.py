#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
#  yahboom_nav / launch / map_slamtoolbox_launch.py
#
#  SLAM mapping phase – run this BEFORE auto_nav.launch.py to build the map.
#
#  Starts:
#    1. yahboomcar_bringup  – micro-ROS bridge + LiDAR driver
#    2. slam_toolbox        – online async SLAM (builds & publishes /map)
#    3. static_transform    – base_link → laser_frame TF
#    4. RViz2               – live map visualisation (slam_config.rviz)
#
#  Usage:
#    ros2 launch yahboom_nav map_slamtoolbox_launch.py
#
#  After mapping is complete, save the map then launch auto_nav:
#    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
#    ros2 launch yahboom_nav auto_nav.launch.py map:=~/maps/my_map.yaml
#
#  NOTE: the micro-ROS UDP agent must be running before this launch file:
#    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4
# ─────────────────────────────────────────────────────────────────────────────

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────────────
    slam_toolbox_dir    = get_package_share_directory('slam_toolbox')
    yahboom_bringup_dir = get_package_share_directory('yahboomcar_bringup')
    yahboom_nav_dir     = get_package_share_directory('yahboom_nav')

    # ── RViz config ───────────────────────────────────────────────────────────
    # Place your slam_config.rviz in yahboom_nav/rviz/.
    # Falls back to nav.rviz if slam_config.rviz is not found.
    slam_rviz = os.path.join(yahboom_nav_dir, 'rviz', 'slam_config.rviz')
    nav_rviz  = os.path.join(yahboom_nav_dir, 'rviz', 'nav.rviz')
    rviz_config = slam_rviz if os.path.isfile(slam_rviz) else nav_rviz

    # ── 1. Yahboom bringup (micro-ROS + LiDAR) ───────────────────────────────
    yahboom_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboom_bringup_dir, 'launch',
                         'yahboomcar_bringup_launch.py')
        )
    )

    # ── 2. SLAM Toolbox (online async) ───────────────────────────────────────
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',

            # Coordinate frames – must match Yahboom firmware
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'scan_topic': '/scan',

            # Map resolution: 5 cm/pixel – good balance for indoor rooms
            'resolution':            '0.05',
            'map_update_interval':   '1.0',

            # Update thresholds: robot must move >15 cm OR rotate >2.8° before
            # the map is updated.  Prevents wasted CPU while stationary.
            'minimum_travel_distance': '0.15',
            'minimum_travel_heading':  '0.05',

            'throttle_scans':   '1',
            'max_laser_range':  '3.0',

            # Scan matching
            'use_scan_matching':   'True',
            'use_scan_barycenter': 'True',

            # Loop closure – keeps the map consistent across revisits
            'do_loop_closing':                  'true',
            'loop_search_maximum_distance':     '3.0',
            'scan_buffer_size':                 '5',
            'scan_buffer_maximum_scan_distance':'2.0',

            # Correlation search – tuned for small indoor spaces
            'correlation_search_space_dimension':  '0.5',
            'correlation_search_space_resolution': '0.01',
        }.items()
    )

    # ── 3. Static TF: base_link → laser_frame ────────────────────────────────
    # Offset measured from Yahboom car CAD / physical measurement.
    # x=-0.0046 m (forward), z=0.094 m (height), no rotation.
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '-0.0046412', '0', '0.094079',  # x y z (metres)
            '0', '0', '0',                   # roll pitch yaw (radians)
            'base_link', 'laser_frame'
        ],
        output='screen',
    )

    # ── 4. RViz2 ─────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        yahboom_bringup,   # micro-ROS bridge + LiDAR
        slam_toolbox,      # SLAM → /map
        laser_tf,          # base_link → laser_frame
        rviz,              # live visualisation
    ])
