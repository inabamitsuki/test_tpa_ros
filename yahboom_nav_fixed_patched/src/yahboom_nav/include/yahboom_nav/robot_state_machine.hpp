#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  yahboom_nav / robot_state_machine.hpp
//  Top-level FSM that drives autonomous navigation + detection + cube drop
// ─────────────────────────────────────────────────────────────────────────────

#include "yahboom_nav/types.hpp"
#include "yahboom_nav/detection_manager.hpp"
#include "yahboom_nav/navigation_manager.hpp"
#include "yahboom_nav/servo_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <set>

namespace yahboom_nav {

class RobotStateMachine {
public:
  RobotStateMachine(
    rclcpp::Node::SharedPtr node,
    const NavConfig& cfg,
    std::shared_ptr<DetectionManager>  det,
    std::shared_ptr<NavigationManager> nav,
    std::shared_ptr<ServoController>   servo);

  // Called at fixed frequency from the main loop timer
  void tick();

  RobotState currentState() const { return state_; }

private:
  // ── State handlers ─────────────────────────────────────────────────────────
  void handleIdle();
  void handleExploring();
  void handleNavigatingToGoal();
  void handleHumanDetected();
  void handleApproachingHuman();
  void handleApriltagDetected();
  void handleApproachingTag();
  void handleDroppingCube();
  void handleRecovery();

  void transitionTo(RobotState next);

  // ── Helpers ────────────────────────────────────────────────────────────────
  bool isStuck();
  void startRecovery();
  void sendApproachVelocity(double target_distance, double angle);

  // ── Members ────────────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr            node_;
  const NavConfig&                   cfg_;
  std::shared_ptr<DetectionManager>  det_;
  std::shared_ptr<NavigationManager> nav_;
  std::shared_ptr<ServoController>   servo_;

  RobotState  state_{RobotState::IDLE};
  rclcpp::Time state_enter_time_;
  rclcpp::Time last_move_time_;
  double       last_x_{0}, last_y_{0};

  // Recovery
  double recovery_target_yaw_{0};
  bool   recovery_turning_{false};

  // Approach tracking
  double approach_target_x_{0}, approach_target_y_{0};
  // Distance the robot has physically travelled since entering APPROACHING_HUMAN.
  // Used to gate the MediaPipe fallback drop so the robot must actually close
  // the gap before it is allowed to drop without a LiDAR confirmation.
  double approach_start_x_{0}, approach_start_y_{0};
  double approach_dist_travelled_{0.0};
  bool   approach_pose_seeded_{false};
  int    cubes_dropped_{0};

  // FIX 5: track whether last_x_/last_y_ have been seeded from a real pose
  bool   pose_initialised_{false};
  double last_yaw_init_{0};

  // FIX 3: debounce counter for human detection confirmation
  int    human_confirm_ticks_{0};

  // Patience counter for brief tag dropouts during approach (camera frame miss)
  int    tag_lost_ticks_{0};

  // Patience counter for brief human dropouts during approach (LiDAR frame miss)
  int    human_lost_ticks_{0};

  // Visited-target memory: avoid re-dropping at the same target
  bool                human_dropped_{false};   // dropped at human image already
  std::set<int>       dropped_tag_ids_;        // set of tag IDs already serviced
};

}  // namespace yahboom_nav
