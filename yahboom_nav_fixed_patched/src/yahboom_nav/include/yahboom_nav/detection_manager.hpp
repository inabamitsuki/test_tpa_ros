#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  yahboom_nav / detection_manager.hpp
//  Fuses LiDAR scan clusters + MediaPipe pose points + AprilTag detections
//  into a single DetectionResult consumed by the state machine.
//
//  CHANGES vs previous version:
//    1. Added yahboomcar_msgs/PointArray include and subscriber for
//       /mediapipe/points.
//    2. Added mediapipeCallback() and human_warning_ counter (the same
//       asymmetric +1/-2 counter from the test scripts, now with proper
//       clamping on both ends).
//    3. human_detected is now set by EITHER LiDAR OR MediaPipe. LiDAR
//       still provides the distance + angle used for actual approach
//       navigation, while MediaPipe provides a more reliable presence
//       confirmation.
// ─────────────────────────────────────────────────────────────────────────────

#include "yahboom_nav/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
// CHANGE 1: MediaPipe message type from Yahboom's own package
#include <yahboomcar_msgs/msg/point_array.hpp>

#include <mutex>
#include <atomic>
#include <vector>

namespace yahboom_nav {

class DetectionManager {
public:
  explicit DetectionManager(rclcpp::Node::SharedPtr node, const NavConfig& cfg);

  // Called each control-loop tick – returns a thread-safe snapshot
  DetectionResult getLatestDetection();

  // FIX: returns true once at least one /mediapipe/points message has arrived.
  // Used by the FSM to decide whether to require camera confirmation for human
  // detection. If the camera has never published, LiDAR-only detection is
  // allowed so the robot can still operate without a working WiFi camera.
  bool isCameraActive() const { return camera_active_; }

private:
  // ── Subscribers ────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                    scan_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr     tag_sub_;
  // CHANGE 1: MediaPipe subscriber
  rclcpp::Subscription<yahboomcar_msgs::msg::PointArray>::SharedPtr               pose_sub_;

  // ── Callbacks ──────────────────────────────────────────────────────────────
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void tagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
  // CHANGE 2: MediaPipe callback
  void mediapipeCallback(const yahboomcar_msgs::msg::PointArray::SharedPtr msg);

  // ── LiDAR clustering ───────────────────────────────────────────────────────
  struct Cluster {
    double cx{0}, cy{0};
    double width{0};
    double distance{0};
    double angle{0};
  };
  std::vector<Cluster> clusterScan(const sensor_msgs::msg::LaserScan& scan);

  // ── State ──────────────────────────────────────────────────────────────────
  std::mutex          mutex_;
  DetectionResult     latest_;
  rclcpp::Node::SharedPtr node_;
  const NavConfig&    cfg_;

  rclcpp::Time last_tag_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_human_seen_{0, 0, RCL_ROS_TIME};
  // FIX 4: timestamp of last mediapipe frame that had keypoints;
  //        used to persist mediapipe_visible for 500 ms so the FSM
  //        does not miss a positive detection between camera frames.
  rclcpp::Time last_mediapipe_seen_{0, 0, RCL_ROS_TIME};

  // CHANGE 2: MediaPipe warning counter (mirrors test_humanDetection logic)
  int human_warning_{0};

  // FIX: true once any /mediapipe/points message has ever been received.
  // Stays true even if the camera stream later drops. The FSM uses this
  // to gate human detection: if camera was never seen, skip mediapipe_visible
  // requirement (LiDAR-only mode). If camera was seen, require it.
  std::atomic<bool> camera_active_{false};
};

}  // namespace yahboom_nav
