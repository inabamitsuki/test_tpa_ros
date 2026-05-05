from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. ดึง Path ของ Package ต่างๆ
    # เปลี่ยน 'my_robot_slam' เป็นชื่อโฟลเดอร์/package ที่คุณสร้างจริง
    #pkg_name = 'my_robot_slam'
    #pkg_share = get_package_share_directory(pkg_name)

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    yahboom_bringup_dir = get_package_share_directory('yahboomcar_bringup')

    # 2. ระบุตำแหน่งไฟล์ .rviz ในโฟลเดอร์ config ของ package
    #rviz_config_path = os.path.join(pkg_share, 'config', 'my_slam_config.rviz')
    rviz_config_path = os.path.join('rviz','slam_config.rviz')
    return LaunchDescription([
        # 3. รัน Bringup ของ Yahboom (เชื่อมต่อ Micro-ROS + LiDAR)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(yahboom_bringup_dir, 'launch', 'yahboomcar_bringup_launch.py')
            )
        ),
        #รัน SLAM Toolbox (พร้อมพารามิเตอร์ที่ถูกต้องสำหรับ Yahboom)

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

            ),

            launch_arguments={

                'use_sim_time': 'False',

                'base_frame': 'base_link',

                'odom_frame': 'odom',

                'scan_topic': '/scan',

                'resolution': '0.05',# กำหนดความละเอียดเป็น 5cm ต่อพิกเซล

                'map_update_interval': '1.0',

    		'minimum_travel_distance': '0.15', # หุ่นต้องวิ่งไปมากกว่า 15 cm ถึงจะอัปเดตแผนที่

    		'minimum_travel_heading': '0.05', #หุ่นยนต์ต้องหมุนตัวอย่างน้อยประมาณ 2.8 องศา (0.05 rad) ระบบถึงจะอัปเดตแผนที่

    		'throttle_scans': '1',

    		'max_laser_range':'3.0',
    		'use_scan_matching': 'True',
                'use_scan_barycenter': 'True',
                'do_loop_closing': 'true',
                'loop_search_maximum_distance': '3.0',
                'scan_buffer_size': '5',
                'scan_buffer_maximum_scan_distance': '2.0',
                'correlation_search_space_dimension': '0.5',
                'correlation_search_space_resolution': '0.01'

            }.items()

        ),


        # รัน Static TF (สะพานเชื่อมชื่อเฟรม radar_Link -> laser_frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            #arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame']
            arguments=['-0.0046412', '0' , '0.094079','0','0','0','base_link','laser_frame']
        ),



        # เปิด RViz2 พร้อมโหลดไฟล์ Config จากโฟลเดอร์ config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
