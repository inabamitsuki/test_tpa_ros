cmake_minimum_required(VERSION 3.8)
project(yahboom_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# CHANGE: added for MediaPipe pose points (/mediapipe/points)
find_package(yahboomcar_msgs REQUIRED)

set(DEPS
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  nav2_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  apriltag_msgs
  cv_bridge
  image_transport
  pcl_conversions
  yahboomcar_msgs        # CHANGE: added
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ── Main navigation node ──────────────────────────────────────────────────────
add_executable(auto_nav_node
  src/auto_nav_node.cpp
  src/detection_manager.cpp
  src/navigation_manager.cpp
  src/servo_controller.cpp
  src/robot_state_machine.cpp
)
ament_target_dependencies(auto_nav_node ${DEPS})
target_link_libraries(auto_nav_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS auto_nav_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/publish_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME publish_initial_pose
)

install(PROGRAMS
  scripts/cam_pose_apriltag.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cam_pose_apriltag
)

ament_package()
