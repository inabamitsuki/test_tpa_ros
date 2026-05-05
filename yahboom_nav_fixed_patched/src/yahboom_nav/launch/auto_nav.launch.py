#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
#  yahboom_nav / launch / auto_nav.launch.py
#
#  Fully automatic launch – no RViz interaction required.
#
#  Startup sequence (timed):
#   t= 0 s  micro-ROS agent  (robot hardware: odom, scan, cmd_vel, servo)
#   t= 0 s  micro-ROS agent  (WiFi camera: /camera/image_raw via port 9999)
#   t= 0 s  Nav2 stack       (AMCL + planners + controllers)
#   t= 0 s  apriltag_ros     (36h11 detector)
#   t= 8 s  initial_pose_pub (publishes /initialpose → AMCL auto-localizes)
#   t=20 s  auto_nav_node    (FSM starts after AMCL has a valid pose)
#
#  Usage:
#   ros2 launch yahboom_nav auto_nav.launch.py map:=/path/to/map.yaml
#
#  Override start pose (metres / radians in map frame):
#   ros2 launch yahboom_nav auto_nav.launch.py \
#     map:=~/maps/lab.yaml initial_x:=1.5 initial_y:=0.8 initial_yaw:=1.57
#
#  After the first successful run AMCL saves its pose to ~/.ros/amcl_pose.
#  On every subsequent boot it resumes from that saved pose automatically,
#  so the initial_pose_pub effectively becomes a no-op safety fallback.
# ─────────────────────────────────────────────────────────────────────────────

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('yahboom_nav')

    # ── Launch arguments ──────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'config', 'map.yaml'),
        description='Full path to the SLAM map YAML file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (set true in Gazebo)')

    # Initial pose – only critical on very first boot.
    # After that AMCL reloads ~/.ros/amcl_pose automatically.
    initial_x_arg = DeclareLaunchArgument(
        'initial_x', default_value='0.0',
        description='Robot start X in map frame (metres). '
                    'Set to match where the robot is placed relative to the SLAM origin.')

    initial_y_arg = DeclareLaunchArgument(
        'initial_y', default_value='0.0',
        description='Robot start Y in map frame (metres). '
                    'Set to match where the robot is placed relative to the SLAM origin.')

    initial_yaw_arg = DeclareLaunchArgument(
        'initial_yaw', default_value='0.0',
        description='Robot start yaw in map frame (radians, CCW positive)')

    # FIX (Bug 1): changed default to 'false'.
    # The old behaviour always wiped ~/.ros/amcl_pose, forcing AMCL to
    # re-localize from (initial_x, initial_y, initial_yaw) on every launch.
    # If the robot is not placed at the exact SLAM origin those defaults (0,0,0)
    # cause AMCL to publish a wrong TF, getRobotPose() falls back to odom,
    # and the FSM sends Nav2 goals with odom coordinates in the map frame →
    # nonstop frontier loop.
    #
    # Leave clear_amcl_pose:=false (default) — AMCL reloads its saved pose
    # automatically.  Only set true on the very first run after creating a new
    # map, or when you deliberately move the robot to a different start point.
    #
    # When you DO set clear_amcl_pose:=true, also pass the correct pose:
    #   ros2 launch yahboom_nav auto_nav.launch.py \
    #     map:=/home/mitsuki/smap/my_robot_map.yaml \
    #     clear_amcl_pose:=true \
    #     initial_x:=<X>  initial_y:=<Y>  initial_yaw:=<YAW>
    #
    # To find (X,Y,YAW): during SLAM, drive the robot to its intended start
    # position and read: ros2 topic echo /odom --once
    clear_amcl_pose_arg = DeclareLaunchArgument(
        'clear_amcl_pose', default_value='false',
        description='Set true to wipe ~/.ros/amcl_pose so AMCL re-localises '
                    'from initial_x/y/yaw. Leave false (default) to resume '
                    'from the saved pose of the previous run.')

    # ── 0. Optionally wipe stale AMCL pose ───────────────────────────────────
    clear_amcl_pose = ExecuteProcess(
        cmd=['bash', '-c', 'rm -f ~/.ros/amcl_pose ; echo "[Launch] stale amcl_pose cleared"'],
        name='clear_amcl_pose',
        output='screen',
        condition=IfCondition(LaunchConfiguration('clear_amcl_pose')),
    )

    # ── 1a. micro-ROS agent – robot hardware  (t = 0 s, port 8888) ───────────
    #  Handles: /odom, /scan, /cmd_vel, /servo_s1, /imu  (serial/UDP from MCU)
    uros_robot_agent = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'source ~/uros_ws/install/setup.bash && '
            'ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4'
        ],
        name='uros_robot_agent',
        output='screen',
    )

    # ── 1b. micro-ROS agent – WiFi camera  (t = 0 s, port 9999) ─────────────
    #  Handles: /camera/image_raw, /camera/camera_info  (WiFi-UDP from camera)
    #  MediaPipe and AprilTag both depend on this stream; without it
    #  mediapipe_visible is always false and the robot will never approach
    #  a human picture (camera confirmation is now required for every drop).
    uros_camera_agent = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'source ~/uros_ws/install/setup.bash && '
            'ros2 run micro_ros_agent micro_ros_agent udp4 --port 9999 -v4'
        ],
        name='uros_camera_agent',
        output='screen',
    )

    # ── 2. Nav2 stack  (t = 0 s) ─────────────────────────────────────────────
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map':          LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            'autostart':    'true',
        }.items()
    )

    # ── 2b. Vision node – MediaPipe + AprilTag  (t = 2 s) ───────────────────
    #  Decodes /espRos/esp32camera (CompressedImage from ESP32-CAM) and:
    #    • publishes /mediapipe/points  → consumed by C++ DetectionManager
    #    • republishes /camera/image_raw → consumed by apriltag_ros above
    #    • publishes /vision/latest_at_id → diagnostic String
    #  Delayed 2 s so the micro-ROS camera agent (port 9999) is ready first.
    vision_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='yahboom_nav',
                executable='cam_pose_apriltag',
                name='cam_pose_apriltag_node',
                output='screen',
                parameters=[{
                    'camera_topic':    '/espRos/esp32camera',
                    'mediapipe_topic': '/mediapipe/points',
                    'raw_image_topic': '/camera/image_raw',
                    'at_id_topic':     '/vision/latest_at_id',
                    'apriltag_family': '36h11',
                    'show_window':     True,   # set False on headless robot PC
                }],
            )
        ]
    )

    # ── 3. AprilTag detector  (t = 0 s) ──────────────────────────────────────
    # Reads /camera/image_raw which is republished by cam_pose_apriltag_node
    # (decoded from /espRos/esp32camera compressed stream).
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        remappings=[
            ('image_rect',  '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('detections',  '/apriltag/detections'),
        ],
        parameters=[{
            'family':      '36h11',
            'size':        0.16,
            'max_hamming': 0,
            'detector': {
                'threads':    2,
                'decimate':   1.0,
                'blur':       0.0,
                'refine':     True,
                'sharpening': 0.25,
            },
        }],
    )

    # ── 4. Initial pose publisher  (t = 8 s) ─────────────────────────────────
    # Waits for AMCL to be fully up, then publishes one /initialpose message.
    initial_pose_publisher = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='yahboom_nav',
                executable='publish_initial_pose',
                name='initial_pose_publisher',
                output='screen',
                parameters=[{
                    'x':   LaunchConfiguration('initial_x'),
                    'y':   LaunchConfiguration('initial_y'),
                    'yaw': LaunchConfiguration('initial_yaw'),
                }],
            )
        ]
    )

    # ── 5. auto_nav_node  (t = 20 s) ─────────────────────────────────────────
    # 20 s gives AMCL time to receive /initialpose (t=8 s) and converge on a
    # valid localisation before the FSM starts issuing goals or detections.
    # The FSM also has an internal guard that blocks it from leaving IDLE until
    # getRobotPose() returns a valid pose, as a second layer of safety.
    auto_nav_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='yahboom_nav',
                executable='auto_nav_node',
                name='auto_nav_node',
                output='screen',
                parameters=[
                    os.path.join(pkg_dir, 'config', 'nav_params.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        map_arg,
        use_sim_time_arg,
        initial_x_arg,
        initial_y_arg,
        initial_yaw_arg,
        clear_amcl_pose_arg,
        # Nodes / processes (startup order enforced by TimerAction delays)
        clear_amcl_pose,         # t= 0 s  wipe stale amcl_pose (only if clear_amcl_pose:=true)
        uros_robot_agent,        # t= 0 s  port 8888 – robot hardware
        uros_camera_agent,       # t= 0 s  port 9999 – WiFi camera
        nav2_launch,             # t= 0 s
        apriltag_node,           # t= 0 s
        vision_node,             # t= 2 s  MediaPipe + AprilTag + image_raw bridge
        initial_pose_publisher,  # t= 8 s
        auto_nav_node,           # t=20 s
    ])
