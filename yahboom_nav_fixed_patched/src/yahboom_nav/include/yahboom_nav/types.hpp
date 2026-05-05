#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  yahboom_nav / types.hpp
//  Shared types, constants, and configuration for the Yahboom microROS car.
//
//  CHANGES vs previous version:
//    1. Added mediapipe_topic  – /mediapipe/points (yahboomcar_msgs/PointArray)
//    2. Added human_warn_max / human_warn_threshold – counter tuning from
//       the test_humanDetection logic.
//    3. Replaced servo_channel / servo_open_pwm / servo_closed_pwm with
//       servo_open_angle / servo_closed_angle to match the /servo_s1
//       angle-based interface used by the Yahboom firmware on that topic.
//       The old (channel<<12)|pwm_us encoding is no longer used.
// ─────────────────────────────────────────────────────────────────────────────

#include <string>
#include <array>
#include <cmath>

namespace yahboom_nav {

// ── Robot state machine ───────────────────────────────────────────────────────
enum class RobotState {
  IDLE,
  EXPLORING,
  NAVIGATING_TO_GOAL,
  HUMAN_DETECTED,
  APPROACHING_HUMAN,
  APRILTAG_DETECTED,
  APPROACHING_TAG,
  DROPPING_CUBE,
  RECOVERY,
  MISSION_COMPLETE
};

inline const char* stateStr(RobotState s) {
  switch (s) {
    case RobotState::IDLE:               return "IDLE";
    case RobotState::EXPLORING:          return "EXPLORING";
    case RobotState::NAVIGATING_TO_GOAL: return "NAVIGATING_TO_GOAL";
    case RobotState::HUMAN_DETECTED:     return "HUMAN_DETECTED";
    case RobotState::APPROACHING_HUMAN:  return "APPROACHING_HUMAN";
    case RobotState::APRILTAG_DETECTED:  return "APRILTAG_DETECTED";
    case RobotState::APPROACHING_TAG:    return "APPROACHING_TAG";
    case RobotState::DROPPING_CUBE:      return "DROPPING_CUBE";
    case RobotState::RECOVERY:           return "RECOVERY";
    case RobotState::MISSION_COMPLETE:   return "MISSION_COMPLETE";
  }
  return "UNKNOWN";
}

// ── Detection result ──────────────────────────────────────────────────────────
struct DetectionResult {
  bool human_detected{false};
  bool apriltag_detected{false};
  int  apriltag_id{-1};
  double human_distance{999.0};    // metres (from LiDAR cluster)
  double human_angle_rad{0.0};     // relative to robot heading (from LiDAR)
  double tag_x{0.0};               // tag centre in camera frame (pixels)
  double tag_y{0.0};
  double tag_distance{999.0};      // rough depth estimate (metres)
  // True when MediaPipe sees pose keypoints regardless of LiDAR distance
  bool mediapipe_visible{false};
};

// ── Configuration (loaded from ROS params) ────────────────────────────────────
struct NavConfig {
  // --- Topics (must match Yahboom firmware / microROS agent) ---
  std::string cmd_vel_topic     = "/cmd_vel";
  std::string odom_topic        = "/odom";
  std::string scan_topic        = "/scan";
  std::string camera_topic      = "/camera/image_raw";
  std::string servo_topic       = "/servo_s1";          // CHANGE 3: angle-based
  std::string apriltag_topic    = "/apriltag/detections";
  // detection_topic removed – was declared and stored but never subscribed to.
  std::string map_topic         = "/map";
  std::string initialpose_topic = "/initialpose";
  // CHANGE 1: MediaPipe pose topic
  std::string mediapipe_topic   = "/mediapipe/points";

  // --- Navigation ---
  double goal_tolerance_m    = 0.25;
  double approach_distance_m = 0.35;   // closer for wall-mounted prints
  double max_linear_speed    = 0.20;
  double max_angular_speed   = 0.80;
  double recovery_turn_rad   = 1.57;

  // --- Timeouts ---
  double nav_timeout_sec        = 60.0;
  double stuck_timeout_sec      = 5.0;   // tighter – flat wall confuses LiDAR
  double human_lost_timeout_sec = 5.0;

  // --- LiDAR human detection ---
  // Widened cluster range for flat printed images on walls (A3 ≈ 0.42 m wide)
  double lidar_cluster_min_m  = 0.10;
  double lidar_cluster_max_m  = 1.20;
  double lidar_detect_range_m = 3.00;
  bool   scan_front_only      = true;    // only look forward – avoids wall sides
  double scan_front_half_rad  = 1.05;    // ±60° frontal arc

  // CHANGE 2: MediaPipe warning counter tuning
  int human_warn_max       = 10;   // ceiling for the counter
  int human_warn_threshold = 5;    // level that counts as "human present"

  // Fallback drop: if robot has been APPROACHING_HUMAN for this many seconds
  // AND has physically travelled at least human_approach_min_travel_m toward
  // the target AND MediaPipe still sees the person, drop regardless of LiDAR
  // distance. Handles wall-mounted prints where LiDAR reads the wall.
  // Set human_approach_min_travel_m to e.g. 0.30 m so the robot must
  // actually close the gap before the fallback fires.
  double human_approach_timeout_sec   = 8.0;   // seconds (raised from 6)
  double human_approach_min_travel_m  = 0.30;  // metres robot must travel first

  // --- AprilTag ---
  int    target_tag_id       = -1;
  double tag_approach_dist_m = 0.40;
  double tag_size_m          = 0.16;
  double tag_focal_px        = 600.0;

  // --- Mission ---
  int max_cubes = 3;

  // CHANGE 3: angle-based servo interface (/servo_s1)
  // Removed: servo_channel, servo_open_pwm, servo_closed_pwm
  // The old (channel<<12)|pwm_us encoding was for /servo_control.
  // /servo_s1 accepts a plain Int32 angle value.
  int    servo_open_angle   = -50;  // angle sent while dropping (open)
  int    servo_closed_angle =   0;  // angle sent while holding  (closed)
  double servo_hold_sec     =  2.0; // seconds to keep open
};

// ── Utility: Euclidean 2-D distance ──────────────────────────────────────────
inline double dist2d(double x1, double y1, double x2, double y2) {
  return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

// ── Utility: angle wrap to [-pi, pi] ─────────────────────────────────────────
inline double wrapAngle(double a) {
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

}  // namespace yahboom_nav
