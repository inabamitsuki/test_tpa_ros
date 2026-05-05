// ─────────────────────────────────────────────────────────────────────────────
//  yahboom_nav / detection_manager.cpp
//
//  CHANGES vs previous version:
//    1. Added mediapipeCallback() – subscribes to /mediapipe/points
//       (yahboomcar_msgs/PointArray) and runs the same asymmetric +1/-2
//       warning counter from the test scripts, clamped to
//       [0, cfg_.human_warn_max].
//    2. human_detected is now raised by EITHER the LiDAR cluster path OR
//       the MediaPipe warning counter reaching cfg_.human_warn_threshold.
//       Both paths write to the same latest_.human_detected flag under the
//       shared mutex.
//    3. LiDAR still owns human_distance and human_angle_rad, which are
//       the values the FSM uses for proportional approach navigation.
//       MediaPipe contributes only to the presence flag, not to geometry.
//    4. Age-out in getLatestDetection() clears human_detected regardless of
//       which source set it; the warning counter is decayed independently
//       inside mediapipeCallback so it stays in sync with the LiDAR age-out.
//
//  All previous fixes (FIX 1-5 in the earlier version) are retained.
// ─────────────────────────────────────────────────────────────────────────────

#include "yahboom_nav/detection_manager.hpp"
#include <algorithm>
#include <cmath>

namespace yahboom_nav {

DetectionManager::DetectionManager(rclcpp::Node::SharedPtr node, const NavConfig& cfg)
  : node_(node), cfg_(cfg)
{
  // ── LiDAR subscriber ─────────────────────────────────────────────────────
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    cfg_.scan_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){ scanCallback(msg); });

  // ── AprilTag subscriber ───────────────────────────────────────────────────
  tag_sub_ = node_->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    cfg_.apriltag_topic, 10,
    [this](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg){ tagCallback(msg); });

  // CHANGE 1: MediaPipe pose subscriber
  pose_sub_ = node_->create_subscription<yahboomcar_msgs::msg::PointArray>(
    cfg_.mediapipe_topic, 10,
    [this](const yahboomcar_msgs::msg::PointArray::SharedPtr msg){ mediapipeCallback(msg); });

  RCLCPP_INFO(node_->get_logger(),
    "[DetectionManager] Initialised. scan=%s  tag=%s  mediapipe=%s  "
    "warn_threshold=%d/%d  front_only=%s",
    cfg_.scan_topic.c_str(), cfg_.apriltag_topic.c_str(),
    cfg_.mediapipe_topic.c_str(),
    cfg_.human_warn_threshold, cfg_.human_warn_max,
    cfg_.scan_front_only ? "yes" : "no");
}

// ─────────────────────────────────────────────────────────────────────────────
//  getLatestDetection – thread-safe snapshot with age-out
// ─────────────────────────────────────────────────────────────────────────────
DetectionResult DetectionManager::getLatestDetection()
{
  std::lock_guard<std::mutex> lk(mutex_);
  auto now = node_->now();

  // Age-out human (covers both LiDAR and MediaPipe sources)
  if (latest_.human_detected) {
    if ((now - last_human_seen_).seconds() > cfg_.human_lost_timeout_sec) {
      latest_.human_detected = false;
      human_warning_         = 0;   // reset counter so next detection starts fresh
    }
  }
  // Age-out AprilTag – 5 s gives time for brief occlusions / frame drops
  // without aborting an active approach run.
  if (latest_.apriltag_detected) {
    if ((now - last_tag_seen_).seconds() > 5.0) {
      latest_.apriltag_detected = false;
    }
  }
  // FIX 4: persist mediapipe_visible for 500 ms so the FSM does not miss
  // the detection window between camera frames (~200 ms for ESP32-CAM).
  // Without this, mediapipe_visible flips false on the very next empty
  // mediapipe message and the FSM tick may land in the false window.
  if (latest_.mediapipe_visible) {
    if ((now - last_mediapipe_seen_).seconds() > 0.5) {
      latest_.mediapipe_visible = false;
    }
  }
  return latest_;
}

// ─────────────────────────────────────────────────────────────────────────────
//  CHANGE 2: MediaPipe callback
//  Mirrors the warning-counter logic from test_humanDetection.py / test_drop.py
//  but lives inside the shared mutex so it is safe with the LiDAR path.
// ─────────────────────────────────────────────────────────────────────────────
void DetectionManager::mediapipeCallback(
  const yahboomcar_msgs::msg::PointArray::SharedPtr msg)
{
  // +1 when pose points visible, -2 when none (fast decay on loss)
  int delta = msg->points.empty() ? -2 : 1;

  // FIX: mark camera as active on the very first message we receive,
  // regardless of whether keypoints are present. This signals to the FSM
  // that the camera pipeline is running and mediapipe_visible checks are valid.
  // Atomic write is safe without the mutex.
  if (!camera_active_) {
    camera_active_ = true;
    RCLCPP_INFO(node_->get_logger(),
      "[DetectionManager] Camera active – mediapipe confirmation now required for human detection.");
  }

  std::lock_guard<std::mutex> lk(mutex_);

  human_warning_ = std::clamp(human_warning_ + delta, 0, cfg_.human_warn_max);

  // mediapipe_visible: true this tick if keypoints are present right now.
  // The FSM uses this as a "person is in front of camera" signal independent
  // of LiDAR distance, which is unreliable against wall-mounted flat prints.
  // FIX 4: only set mediapipe_visible=true here; age-out in
  // getLatestDetection() clears it after 500 ms of no detections.
  if (!msg->points.empty()) {
    latest_.mediapipe_visible = true;
    last_mediapipe_seen_      = node_->now();
  }
  // (mediapipe_visible is cleared by the age-out in getLatestDetection)

  RCLCPP_DEBUG(node_->get_logger(),
    "[DetectionManager] MediaPipe points=%zu  warning=%d/%d",
    msg->points.size(), human_warning_, cfg_.human_warn_max);

  if (human_warning_ >= cfg_.human_warn_threshold) {
    latest_.human_detected = true;
    last_human_seen_       = node_->now();
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "[DetectionManager] MediaPipe: human confirmed (warning=%d/%d)",
      human_warning_, cfg_.human_warn_max);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  LiDAR callback
// ─────────────────────────────────────────────────────────────────────────────
void DetectionManager::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto clusters = clusterScan(*msg);

  bool found = false;
  double best_dist = cfg_.lidar_detect_range_m;
  Cluster best{};

  for (auto& c : clusters) {
    if (c.width  >= cfg_.lidar_cluster_min_m &&
        c.width  <= cfg_.lidar_cluster_max_m &&
        c.distance < best_dist)
    {
      best_dist = c.distance;
      best = c;
      found = true;
    }
  }

  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (found) {
      latest_.human_detected  = true;
      latest_.human_distance  = best.distance;
      latest_.human_angle_rad = best.angle;
      last_human_seen_ = node_->now();
    } else {
      // Only clear if MediaPipe also has no confidence
      if (human_warning_ < cfg_.human_warn_threshold) {
        latest_.human_detected = false;
      }
      // Always reset LiDAR geometry so stale values don't mislead approach
      latest_.human_distance  = 999.0;
      latest_.human_angle_rad = 0.0;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Segment clustering (unchanged)
// ─────────────────────────────────────────────────────────────────────────────
std::vector<DetectionManager::Cluster>
DetectionManager::clusterScan(const sensor_msgs::msg::LaserScan& scan)
{
  std::vector<Cluster> clusters;
  const double JUMP = 0.20;

  struct Point { double x, y; };
  std::vector<std::vector<Point>> segs;
  segs.emplace_back();

  int n = static_cast<int>(scan.ranges.size());
  for (int i = 0; i < n; ++i) {
    double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;

    double angle = scan.angle_min + i * scan.angle_increment;
    if (cfg_.scan_front_only && std::abs(angle) > cfg_.scan_front_half_rad) continue;

    double x = r * std::cos(angle);
    double y = r * std::sin(angle);

    if (!segs.back().empty()) {
      auto& prev = segs.back().back();
      double dx = x - prev.x, dy = y - prev.y;
      if (std::sqrt(dx*dx + dy*dy) > JUMP) {
        segs.emplace_back();
      }
    }
    segs.back().push_back({x, y});
  }

  for (auto& seg : segs) {
    if (seg.size() < 2) continue;

    double sx = 0, sy = 0;
    for (auto& p : seg) { sx += p.x; sy += p.y; }
    sx /= seg.size(); sy /= seg.size();

    auto& f = seg.front();
    auto& l = seg.back();
    double w    = std::sqrt((f.x-l.x)*(f.x-l.x) + (f.y-l.y)*(f.y-l.y));
    double dist = std::sqrt(sx*sx + sy*sy);
    double ang  = std::atan2(sy, sx);

    clusters.push_back({sx, sy, w, dist, ang});
  }
  return clusters;
}

// ─────────────────────────────────────────────────────────────────────────────
//  AprilTag callback (unchanged)
// ─────────────────────────────────────────────────────────────────────────────
void DetectionManager::tagCallback(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
  if (msg->detections.empty()) return;

  for (auto& det : msg->detections) {
    if (cfg_.target_tag_id != -1 && det.id != cfg_.target_tag_id) continue;

    double cx   = det.centre.x;
    double cy   = det.centre.y;
    double span = 0.0;

    if (det.corners.size() == 4) {
      // Average both diagonals for a more stable pixel span.
      double dx0 = det.corners[0].x - det.corners[2].x;
      double dy0 = det.corners[0].y - det.corners[2].y;
      double dx1 = det.corners[1].x - det.corners[3].x;
      double dy1 = det.corners[1].y - det.corners[3].y;
      span = (std::sqrt(dx0*dx0 + dy0*dy0) + std::sqrt(dx1*dx1 + dy1*dy1)) * 0.5;
    }

    // Lower threshold to 0.5 px so small/distant tags still get a real depth.
    // Clamp to 3.5 m – beyond that the formula is too noisy; robot closes in
    // and gets a better reading on the next frame.
    double depth = (span > 0.5)
      ? std::min(cfg_.tag_size_m * cfg_.tag_focal_px / span, 3.5)
      : 3.5;

    std::lock_guard<std::mutex> lk(mutex_);
    latest_.apriltag_detected = true;
    latest_.apriltag_id       = det.id;
    latest_.tag_x             = cx;
    latest_.tag_y             = cy;
    latest_.tag_distance      = depth;
    last_tag_seen_ = node_->now();
    break;
  }
}

}  // namespace yahboom_nav
