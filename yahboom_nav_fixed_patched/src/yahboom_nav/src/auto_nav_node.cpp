// ─────────────────────────────────────────────────────────────────────────────
//  yahboom_nav / auto_nav_node.cpp
//
//  CHANGES vs previous version:
//    1. Added declare/get for mediapipe_topic.
//    2. Added declare/get for human_warn_max and human_warn_threshold.
//    3. Replaced servo_channel / servo_open_pwm / servo_closed_pwm with
//       servo_open_angle / servo_closed_angle to match the /servo_s1 interface.
// ─────────────────────────────────────────────────────────────────────────────

#include "yahboom_nav/types.hpp"
#include "yahboom_nav/detection_manager.hpp"
#include "yahboom_nav/navigation_manager.hpp"
#include "yahboom_nav/servo_controller.hpp"
#include "yahboom_nav/robot_state_machine.hpp"

#include <rclcpp/rclcpp.hpp>

using namespace yahboom_nav;

class AutoNavNode : public rclcpp::Node {
public:
  AutoNavNode() : Node("auto_nav_node")
  {
    // ── Declare all ROS parameters ─────────────────────────────────────────
    // Topics
    this->declare_parameter("cmd_vel_topic",      cfg_.cmd_vel_topic);
    this->declare_parameter("odom_topic",          cfg_.odom_topic);
    this->declare_parameter("scan_topic",          cfg_.scan_topic);
    this->declare_parameter("camera_topic",        cfg_.camera_topic);
    this->declare_parameter("servo_topic",         cfg_.servo_topic);
    this->declare_parameter("apriltag_topic",      cfg_.apriltag_topic);
    // detection_topic removed – no subscriber ever used it (dead parameter).
    this->declare_parameter("map_topic",           cfg_.map_topic);
    this->declare_parameter("initialpose_topic",   cfg_.initialpose_topic);
    // CHANGE 1: MediaPipe topic
    this->declare_parameter("mediapipe_topic",     cfg_.mediapipe_topic);

    // Navigation
    this->declare_parameter("goal_tolerance_m",    cfg_.goal_tolerance_m);
    this->declare_parameter("approach_distance_m", cfg_.approach_distance_m);
    this->declare_parameter("max_linear_speed",    cfg_.max_linear_speed);
    this->declare_parameter("max_angular_speed",   cfg_.max_angular_speed);
    this->declare_parameter("recovery_turn_rad",   cfg_.recovery_turn_rad);
    this->declare_parameter("nav_timeout_sec",     cfg_.nav_timeout_sec);
    this->declare_parameter("stuck_timeout_sec",   cfg_.stuck_timeout_sec);
    this->declare_parameter("human_lost_timeout_sec", cfg_.human_lost_timeout_sec);

    // LiDAR
    this->declare_parameter("lidar_cluster_min_m",  cfg_.lidar_cluster_min_m);
    this->declare_parameter("lidar_cluster_max_m",  cfg_.lidar_cluster_max_m);
    this->declare_parameter("lidar_detect_range_m", cfg_.lidar_detect_range_m);
    this->declare_parameter("scan_front_only",       cfg_.scan_front_only);
    this->declare_parameter("scan_front_half_rad",   cfg_.scan_front_half_rad);

    // CHANGE 2: MediaPipe warning counter tuning
    this->declare_parameter("human_warn_max",       cfg_.human_warn_max);
    this->declare_parameter("human_warn_threshold", cfg_.human_warn_threshold);
    this->declare_parameter("human_approach_timeout_sec",  cfg_.human_approach_timeout_sec);
    this->declare_parameter("human_approach_min_travel_m", cfg_.human_approach_min_travel_m);

    // AprilTag
    this->declare_parameter("target_tag_id",        cfg_.target_tag_id);
    this->declare_parameter("tag_approach_dist_m",  cfg_.tag_approach_dist_m);
    this->declare_parameter("tag_size_m",           cfg_.tag_size_m);
    this->declare_parameter("tag_focal_px",         cfg_.tag_focal_px);

    // CHANGE 3: angle-based servo params (replaced channel/pwm params)
    this->declare_parameter("servo_open_angle",   cfg_.servo_open_angle);
    this->declare_parameter("servo_closed_angle", cfg_.servo_closed_angle);
    this->declare_parameter("servo_hold_sec",     cfg_.servo_hold_sec);

    // Mission
    this->declare_parameter("max_cubes", cfg_.max_cubes);

    // ── Read parameters back ───────────────────────────────────────────────
    cfg_.cmd_vel_topic      = this->get_parameter("cmd_vel_topic").as_string();
    cfg_.odom_topic         = this->get_parameter("odom_topic").as_string();
    cfg_.scan_topic         = this->get_parameter("scan_topic").as_string();
    cfg_.camera_topic       = this->get_parameter("camera_topic").as_string();
    cfg_.servo_topic        = this->get_parameter("servo_topic").as_string();
    cfg_.apriltag_topic     = this->get_parameter("apriltag_topic").as_string();
    // detection_topic removed – no subscriber ever used it (dead parameter).
    cfg_.map_topic          = this->get_parameter("map_topic").as_string();
    cfg_.initialpose_topic  = this->get_parameter("initialpose_topic").as_string();
    // CHANGE 1
    cfg_.mediapipe_topic    = this->get_parameter("mediapipe_topic").as_string();

    cfg_.goal_tolerance_m    = this->get_parameter("goal_tolerance_m").as_double();
    cfg_.approach_distance_m = this->get_parameter("approach_distance_m").as_double();
    cfg_.max_linear_speed    = this->get_parameter("max_linear_speed").as_double();
    cfg_.max_angular_speed   = this->get_parameter("max_angular_speed").as_double();
    cfg_.recovery_turn_rad   = this->get_parameter("recovery_turn_rad").as_double();
    cfg_.nav_timeout_sec     = this->get_parameter("nav_timeout_sec").as_double();
    cfg_.stuck_timeout_sec   = this->get_parameter("stuck_timeout_sec").as_double();
    cfg_.human_lost_timeout_sec = this->get_parameter("human_lost_timeout_sec").as_double();

    cfg_.lidar_cluster_min_m  = this->get_parameter("lidar_cluster_min_m").as_double();
    cfg_.lidar_cluster_max_m  = this->get_parameter("lidar_cluster_max_m").as_double();
    cfg_.lidar_detect_range_m = this->get_parameter("lidar_detect_range_m").as_double();
    cfg_.scan_front_only      = this->get_parameter("scan_front_only").as_bool();
    cfg_.scan_front_half_rad  = this->get_parameter("scan_front_half_rad").as_double();

    // CHANGE 2
    cfg_.human_warn_max       = (int)this->get_parameter("human_warn_max").as_int();
    cfg_.human_warn_threshold = (int)this->get_parameter("human_warn_threshold").as_int();
    cfg_.human_approach_timeout_sec  = this->get_parameter("human_approach_timeout_sec").as_double();
    cfg_.human_approach_min_travel_m = this->get_parameter("human_approach_min_travel_m").as_double();

    cfg_.target_tag_id       = (int)this->get_parameter("target_tag_id").as_int();
    cfg_.tag_approach_dist_m = this->get_parameter("tag_approach_dist_m").as_double();
    cfg_.tag_size_m          = this->get_parameter("tag_size_m").as_double();
    cfg_.tag_focal_px        = this->get_parameter("tag_focal_px").as_double();

    // CHANGE 3
    cfg_.servo_open_angle   = (int)this->get_parameter("servo_open_angle").as_int();
    cfg_.servo_closed_angle = (int)this->get_parameter("servo_closed_angle").as_int();
    cfg_.servo_hold_sec     = this->get_parameter("servo_hold_sec").as_double();

    cfg_.max_cubes = (int)this->get_parameter("max_cubes").as_int();

    // ── Construct sub-systems ──────────────────────────────────────────────
    auto self = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

    det_   = std::make_shared<DetectionManager>(self, cfg_);
    nav_   = std::make_shared<NavigationManager>(self, cfg_);
    servo_ = std::make_shared<ServoController>(self, cfg_);
    fsm_   = std::make_shared<RobotStateMachine>(self, cfg_, det_, nav_, servo_);

    // ── 10 Hz control loop ─────────────────────────────────────────────────
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this](){ fsm_->tick(); });

    RCLCPP_INFO(this->get_logger(),
      "[AutoNavNode] Started. max_v=%.2f m/s  max_w=%.2f rad/s  "
      "servo_topic=%s  open=%d  closed=%d  "
      "mediapipe=%s  warn=%d/%d  max_cubes=%d",
      cfg_.max_linear_speed, cfg_.max_angular_speed,
      cfg_.servo_topic.c_str(), cfg_.servo_open_angle, cfg_.servo_closed_angle,
      cfg_.mediapipe_topic.c_str(),
      cfg_.human_warn_threshold, cfg_.human_warn_max,
      cfg_.max_cubes);
  }

private:
  NavConfig                              cfg_;
  std::shared_ptr<DetectionManager>     det_;
  std::shared_ptr<NavigationManager>    nav_;
  std::shared_ptr<ServoController>      servo_;
  std::shared_ptr<RobotStateMachine>    fsm_;
  rclcpp::TimerBase::SharedPtr          timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoNavNode>();

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
