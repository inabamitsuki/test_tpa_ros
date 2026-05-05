// ─────────────────────────────────────────────────────────────────────────────
//  yahboom_nav / robot_state_machine.cpp
//
//  FIX SUMMARY vs original:
//    1. MAX_CUBES removed as a compile-time constant. Now reads cfg_.max_cubes
//       so it is configurable via YAML/parameter without recompiling.
//    2. tick(): the mission-complete check ran BEFORE the switch, which meant
//       even in MISSION_COMPLETE state it called transitionTo() every tick,
//       spamming the log. Now guarded with `if (state_ != MISSION_COMPLETE)`.
//    3. handleHumanDetected(): immediately transitioned to APPROACHING_HUMAN in
//       the same tick without waiting to confirm the detection was stable. If
//       the LiDAR produced a single-scan glitch the robot would lurch toward
//       nothing. Added a 3-tick confirmation counter before transitioning.
//    4. handleApproachingTag(): linear approach used cfg_.max_linear_speed as
//       a hard cap but multiplied by 0.5*(dist - approach_dist) which could
//       give a very large command if the tag depth estimate was wrong (999 m
//       on startup). Added a secondary clamp of 0.15 m/s during tag approach.
//    5. isStuck(): on the very first call last_x_/last_y_ were 0,0 (default).
//       If the robot started far from origin it would immediately report moved
//       > 0.05 m but then set last_x_/last_y_ to the real position, and on
//       the second call with no movement it would see idle_time growing from
//       node startup rather than from the last real movement. Fixed by
//       initialising last_x_/last_y_ from the first successful getRobotPose.
//    6. handleRecovery(): recovery_target_yaw_ was set to `yaw + recovery_turn_rad`
//       but yaw was fetched before the `if (!recovery_turning_)` block and
//       could differ from the yaw read inside that block on the same tick
//       (two getRobotPose() calls). Unified to a single call per tick.
// ─────────────────────────────────────────────────────────────────────────────

#include "yahboom_nav/robot_state_machine.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace yahboom_nav {

RobotStateMachine::RobotStateMachine(
  rclcpp::Node::SharedPtr node,
  const NavConfig& cfg,
  std::shared_ptr<DetectionManager>  det,
  std::shared_ptr<NavigationManager> nav,
  std::shared_ptr<ServoController>   servo)
: node_(node), cfg_(cfg), det_(det), nav_(nav), servo_(servo)
{
  state_enter_time_ = node_->now();
  last_move_time_   = node_->now();

  // FIX (Bug 4): require TF map→base_link for pose_initialised_, NOT odom.
  // The original code called getRobotPose() which falls back to odometry when
  // TF is not available. Odom is always up, so pose_initialised_ became true
  // immediately even when AMCL hadn't published a valid localised pose yet.
  // The FSM then sent Nav2 goals using odom coordinates as if they were map
  // coordinates → endless wrong-goal loop. Now we only accept a real TF pose.
  pose_initialised_ = nav_->hasTfPose();
  if (pose_initialised_) {
    nav_->getRobotPose(last_x_, last_y_, last_yaw_init_);
  }

  RCLCPP_INFO(node_->get_logger(), "[FSM] Ready. Initial state: IDLE. max_cubes=%d",
              cfg_.max_cubes);
}

// ─────────────────────────────────────────────────────────────────────────────
//  tick
// ─────────────────────────────────────────────────────────────────────────────
void RobotStateMachine::tick()
{
  // FIX (Bug 4): keep retrying with hasTfPose() so we only proceed once AMCL
  // has published a valid map→base_link transform. Using getRobotPose() here
  // would accept odom immediately and send Nav2 goals with wrong coordinates.
  if (!pose_initialised_) {
    if (nav_->hasTfPose()) {
      pose_initialised_ = true;
      nav_->getRobotPose(last_x_, last_y_, last_yaw_init_);
      RCLCPP_INFO(node_->get_logger(),
        "[FSM] AMCL TF ready – starting navigation. Initial pose: (%.2f, %.2f)",
        last_x_, last_y_);
    }
    last_move_time_   = node_->now(); // reset timer while we wait
  }

  // FIX: block all navigation states until we have a valid localised pose.
  // AMCL logs "cannot publish a pose" until /initialpose is received and
  // it has matched a scan.  Running the FSM before that produces garbage
  // goals, false stuck-detections (pose = 0,0), and premature drops.
  // IDLE is exempt so the warm-up spin can still run; only gate the
  // transition OUT of IDLE and all other active states.
  if (!pose_initialised_ && state_ != RobotState::IDLE) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[FSM] Waiting for valid localisation before proceeding…");
    nav_->stopRobot();
    return;
  }

  // FIX 2: only check mission-complete when not already in that state.
  if (state_ != RobotState::MISSION_COMPLETE &&
      cubes_dropped_ >= cfg_.max_cubes)
  {
    transitionTo(RobotState::MISSION_COMPLETE);
  }

  switch (state_) {
    case RobotState::IDLE:               handleIdle();             break;
    case RobotState::EXPLORING:          handleExploring();        break;
    case RobotState::NAVIGATING_TO_GOAL: handleNavigatingToGoal(); break;
    case RobotState::HUMAN_DETECTED:     handleHumanDetected();    break;
    case RobotState::APPROACHING_HUMAN:  handleApproachingHuman(); break;
    case RobotState::APRILTAG_DETECTED:  handleApriltagDetected(); break;
    case RobotState::APPROACHING_TAG:    handleApproachingTag();   break;
    case RobotState::DROPPING_CUBE:      handleDroppingCube();     break;
    case RobotState::RECOVERY:           handleRecovery();         break;
    case RobotState::MISSION_COMPLETE:
      nav_->stopRobot();
      RCLCPP_INFO_ONCE(node_->get_logger(),
        "[FSM] MISSION COMPLETE – %d cubes dropped.", cubes_dropped_);
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void RobotStateMachine::transitionTo(RobotState next)
{
  if (next == state_) return;
  RCLCPP_INFO(node_->get_logger(), "[FSM] %s → %s",
              stateStr(state_), stateStr(next));
  state_             = next;
  state_enter_time_  = node_->now();
  human_confirm_ticks_ = 0;  // reset confirmation counter on any transition
  tag_lost_ticks_      = 0;  // reset tag patience counter on any transition
  human_lost_ticks_    = 0;  // reset human patience counter on any transition
  approach_pose_seeded_    = false;  // reset distance-travelled tracker on any transition
  approach_dist_travelled_ = 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  IDLE
// ─────────────────────────────────────────────────────────────────────────────
void RobotStateMachine::handleIdle()
{
  double age = (node_->now() - state_enter_time_).seconds();
  if (age < 2.0) {
    return;  // wait for Nav2 / SLAM to come up
  } else if (age < 4.0) {
    // FIX: replaced blind forward drive (no obstacle check) with a slow
    // in-place rotation.  Rotation gives SLAM the same initial scan coverage
    // without risking a collision when the robot is close to a wall.
    nav_->cmdVel(0.0, cfg_.max_angular_speed * 0.5);
    return;
  } else {
    nav_->stopRobot();
    transitionTo(RobotState::EXPLORING);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  EXPLORING
// ─────────────────────────────────────────────────────────────────────────────
void RobotStateMachine::handleExploring()
{
  if (isStuck()) { startRecovery(); return; }

  auto det = det_->getLatestDetection();
  if (det.apriltag_detected) {
    // Skip if we already dropped at this tag id
    if (dropped_tag_ids_.count(det.apriltag_id) == 0) {
      transitionTo(RobotState::APRILTAG_DETECTED); return;
    }
  }
  // FIX (Bug 2): mediapipe_visible is only required when the camera pipeline
  // is confirmed running (camera_active_=true). If no /mediapipe/points
  // message has ever arrived the camera is down or not yet connected; in that
  // case LiDAR-only detection is accepted so the robot can still operate.
  // When the camera IS active we still require it to confirm the detection
  // (prevents LiDAR false-positives on walls/furniture).
  bool camera_confirms = det.mediapipe_visible || !det_->isCameraActive();
  if (det.human_detected && !human_dropped_ && camera_confirms) {
    transitionTo(RobotState::HUMAN_DETECTED); return;
  }

  auto ns = nav_->getStatus();
  if (ns == NavStatus::RUNNING) return;

  auto map = nav_->getMap();
  if (!map) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "[FSM] No map yet – waiting…");
    return;
  }

  auto goal = nav_->nextFrontierGoal(*map);
  if (!goal) {
    RCLCPP_INFO(node_->get_logger(), "[FSM] No frontiers left.");
    transitionTo(RobotState::MISSION_COMPLETE);
    return;
  }

  // FIX 8: only transition when the goal was actually queued.
  // If Nav2 isn't up yet, sendGoal() returns false and we stay in
  // EXPLORING to retry next tick rather than spinning into RECOVERY.
  if (nav_->sendGoal(goal->pose.position.x, goal->pose.position.y, 0.0)) {
    transitionTo(RobotState::NAVIGATING_TO_GOAL);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  NAVIGATING_TO_GOAL
// ─────────────────────────────────────────────────────────────────────────────
void RobotStateMachine::handleNavigatingToGoal()
{
  if (isStuck()) { nav_->cancelGoal(); startRecovery(); return; }

  auto det = det_->getLatestDetection();
  if (det.apriltag_detected && dropped_tag_ids_.count(det.apriltag_id) == 0) {
    nav_->cancelGoal();
    transitionTo(RobotState::APRILTAG_DETECTED);
    return;
  }
  // FIX (Bug 2): same camera_active_ guard as handleExploring().
  bool camera_confirms_nav = det.mediapipe_visible || !det_->isCameraActive();
  if (det.human_detected && !human_dropped_ && camera_confirms_nav) {
    nav_->cancelGoal();
    transitionTo(RobotState::HUMAN_DETECTED);
    return;
  }

  if ((node_->now() - state_enter_time_).seconds() > cfg_.nav_timeout_sec) {
    RCLCPP_WARN(node_->get_logger(), "[FSM] Navigation timeout.");
    nav_->cancelGoal();
    transitionTo(RobotState::EXPLORING);
    return;
  }

  auto ns = nav_->getStatus();
  if (ns == NavStatus::SUCCEEDED || ns == NavStatus::FAILED || ns == NavStatus::CANCELLED) {
    transitionTo(RobotState::EXPLORING);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  HUMAN_DETECTED
// ─────────────────────────────────────────────────────────────────────────────
void RobotStateMachine::handleHumanDetected()
{
  nav_->stopRobot();
  auto det = det_->getLatestDetection();

  if (!det.human_detected) {
    human_confirm_ticks_ = 0;
    transitionTo(RobotState::EXPLORING);
    return;
  }

  // FIX 3: require 3 consecutive positive ticks (~300 ms) before committing.
  ++human_confirm_ticks_;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "[FSM] Human @ %.2f m, angle %.1f° (confirm %d/3)",
    det.human_distance, det.human_angle_rad * 180.0 / M_PI,
    human_confirm_ticks_);

  if (human_confirm_ticks_ >= 3) {
    human_confirm_ticks_ = 0;
    transitionTo(RobotState::APPROACHING_HUMAN);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  APPROACHING_HUMAN
// ─────────────────────────────────────────────────────────────────────────────
void RobotStateMachine::handleApproachingHuman()
{
  if (isStuck()) { startRecovery(); return; }

  auto det = det_->getLatestDetection();
  double time_in_state = (node_->now() - state_enter_time_).seconds();

  // ── Seed start pose on first tick so we can measure distance travelled ────
  if (!approach_pose_seeded_) {
    double dummy;
    if (nav_->getRobotPose(approach_start_x_, approach_start_y_, dummy)) {
      approach_pose_seeded_    = true;
      approach_dist_travelled_ = 0.0;
      RCLCPP_INFO(node_->get_logger(),
        "[FSM] APPROACHING_HUMAN start pose seeded at (%.2f, %.2f)",
        approach_start_x_, approach_start_y_);
    }
  }

  // ── Update cumulative distance travelled this approach ────────────────────
  if (approach_pose_seeded_) {
    double cx, cy, dummy;
    if (nav_->getRobotPose(cx, cy, dummy)) {
      approach_dist_travelled_ = dist2d(cx, cy, approach_start_x_, approach_start_y_);
    }
  }

  // ── Human lost: tolerate up to 5 missed ticks (~500 ms) ─────────────────
  if (!det.human_detected) {
    ++human_lost_ticks_;
    if (human_lost_ticks_ > 5) {
      human_lost_ticks_ = 0;
      approach_pose_seeded_ = false;
      nav_->stopRobot();
      RCLCPP_WARN(node_->get_logger(),
        "[FSM] Human lost during approach – returning to EXPLORING.");
      transitionTo(RobotState::EXPLORING);
    }
    return;
  }
  human_lost_ticks_ = 0;

  // ── Drop condition 1: LiDAR close AND camera sees a human ───────────────
  //  Previously lidar_in_range alone was sufficient → any nearby wall/object
  //  within cluster range triggered an immediate drop with no picture in view.
  //  Now both signals must agree: LiDAR measures proximity, MediaPipe confirms
  //  the camera is actually looking at a human image.
  bool lidar_in_range  = (det.human_distance <= cfg_.approach_distance_m + 0.05);
  bool camera_confirms = det.mediapipe_visible;

  // ── Drop condition 2: MediaPipe fallback for wall-mounted flat prints ─────
  //  If the LiDAR reads the wall surface (distance never reaches
  //  approach_distance_m) we allow a fallback ONLY when ALL three hold:
  //    a) MediaPipe currently sees keypoints  (person is in the camera frame)
  //    b) Robot has physically travelled >= human_approach_min_travel_m
  //       since entering this state  (proves it actually closed the gap)
  //    c) Enough time has elapsed  (prevents instant drop on state entry)
  bool mediapipe_close = (det.mediapipe_visible &&
                          approach_dist_travelled_ >= cfg_.human_approach_min_travel_m &&
                          time_in_state >= cfg_.human_approach_timeout_sec);

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "[FSM] Approaching human: dist=%.2f m  angle=%.1f°  mediapipe=%s"
    "  travelled=%.2f m (need %.2f)  t=%.1fs (need %.1f)",
    det.human_distance, det.human_angle_rad * 180.0 / M_PI,
    det.mediapipe_visible ? "yes" : "no",
    approach_dist_travelled_, cfg_.human_approach_min_travel_m,
    time_in_state, cfg_.human_approach_timeout_sec);

  if ((lidar_in_range && camera_confirms) || mediapipe_close) {
    nav_->stopRobot();
    approach_pose_seeded_ = false;
    RCLCPP_INFO(node_->get_logger(),
      "[FSM] Drop triggered: lidar_in_range=%s camera_confirms=%s mediapipe_close=%s"
      " dist=%.2f travelled=%.2f t=%.1fs → DROPPING_CUBE",
      lidar_in_range ? "YES" : "no",
      camera_confirms ? "YES" : "no",
      mediapipe_close ? "YES" : "no",
      det.human_distance, approach_dist_travelled_, time_in_state);
    human_dropped_ = true;
    transitionTo(RobotState::DROPPING_CUBE);
    return;
  }

  // ── Priority 2: divert to AprilTag if one becomes visible ────────────────
  // Only divert if NOT yet close — prevents aborting a nearly-complete approach.
  if (det.apriltag_detected && !lidar_in_range) {
    nav_->stopRobot();
    approach_pose_seeded_ = false;
    transitionTo(RobotState::APRILTAG_DETECTED);
    return;
  }

  // ── Priority 3: keep closing in ───────────────────────────────────────────
  sendApproachVelocity(det.human_distance, det.human_angle_rad);
}

// ─────────────────────────────────────────────────────────────────────────────
//  APRILTAG_DETECTED
//  Wait 3 ticks (~300 ms) to confirm the detection is stable before committing
//  to the approach.  This prevents a single-frame detection from aborting a
//  human approach, and gives the camera pipeline time to deliver a valid depth.
// ─────────────────────────────────────────────────────────────────────────────
void RobotStateMachine::handleApriltagDetected()
{
  nav_->stopRobot();
  auto det = det_->getLatestDetection();
  if (!det.apriltag_detected) {
    human_confirm_ticks_ = 0;
    transitionTo(RobotState::EXPLORING);
    return;
  }

  ++human_confirm_ticks_;  // reuse counter (it resets on every transitionTo)
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
    "[FSM] AprilTag id=%d at ~%.2f m (confirm %d/3)",
    det.apriltag_id, det.tag_distance, human_confirm_ticks_);

  if (human_confirm_ticks_ >= 3) {
    human_confirm_ticks_ = 0;
    transitionTo(RobotState::APPROACHING_TAG);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  APPROACHING_TAG
// ─────────────────────────────────────────────────────────────────────────────
void RobotStateMachine::handleApproachingTag()
{
  if (isStuck()) { startRecovery(); return; }

  auto det = det_->getLatestDetection();

  // Tolerate up to 10 consecutive ticks (~1 s) without a detection before
  // giving up.  Brief camera dropouts (motion blur, lighting) are common.
  if (!det.apriltag_detected) {
    ++tag_lost_ticks_;
    if (tag_lost_ticks_ > 10) {
      tag_lost_ticks_ = 0;
      nav_->stopRobot();
      RCLCPP_WARN(node_->get_logger(), "[FSM] AprilTag lost during approach – returning to EXPLORING.");
      transitionTo(RobotState::EXPLORING);
    }
    // Else: hold last velocity (don't publish anything), wait for next frame.
    return;
  }
  tag_lost_ticks_ = 0;

  const double IMG_CX = 320.0;
  double pixel_err = det.tag_x - IMG_CX;
  double angular   = std::clamp(-0.003 * pixel_err,
                                -cfg_.max_angular_speed, cfg_.max_angular_speed);

  double dist = det.tag_distance;

  // ── Drop condition: check BEFORE computing any velocity ──────────────────
  // Relaxed pixel threshold (60 px ≈ ~10° at 320px half-FOV) so we commit
  // to a drop once close enough rather than hunting for perfect centering.
  if (dist <= cfg_.tag_approach_dist_m + 0.10 && std::abs(pixel_err) < 60.0) {
    nav_->stopRobot();
    RCLCPP_INFO(node_->get_logger(),
      "[FSM] In position for drop (dist=%.2f m, err=%.1f px) → DROPPING_CUBE", dist, pixel_err);
    dropped_tag_ids_.insert(det.apriltag_id);  // mark this tag as serviced
    transitionTo(RobotState::DROPPING_CUBE);
    return;
  }

  // ── Approach velocity ──────────────────────────────────────────────────────
  double linear = 0.0;
  if (dist > cfg_.tag_approach_dist_m + 0.05) {
    // Proportional, hard-capped at 0.15 m/s so depth noise can't runaway.
    linear = std::clamp(0.5 * (dist - cfg_.tag_approach_dist_m), 0.0, 0.15);
  }

  nav_->cmdVel(linear, angular);
}

// ─────────────────────────────────────────────────────────────────────────────
//  DROPPING_CUBE
// ─────────────────────────────────────────────────────────────────────────────
void RobotStateMachine::handleDroppingCube()
{
  // Stop only on the first tick of this state (robot was already stopped
  // before transitioning here, but belt-and-braces).  After that, avoid
  // publishing cmd_vel every tick to keep the micro-ROS serial link clear
  // for the servo angle messages.
  double age = (node_->now() - state_enter_time_).seconds();
  if (age < 0.15) {
    nav_->stopRobot();
  }

  if (servo_->dropCube()) {
    cubes_dropped_++;
    RCLCPP_INFO(node_->get_logger(), "[FSM] Cube dropped! Total: %d / %d",
                cubes_dropped_, cfg_.max_cubes);
    // FIX: reset human_dropped_ so the robot can detect and approach
    // a human target again on the next lap / after an MCU-only reset.
    // Without this, a single successful drop permanently blocks the
    // human-detection branch for the rest of the node's lifetime.
    human_dropped_ = false;
    transitionTo(RobotState::EXPLORING);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  RECOVERY
// ─────────────────────────────────────────────────────────────────────────────
void RobotStateMachine::handleRecovery()
{
  // FIX 6: single getRobotPose call per tick to avoid stale yaw.
  double x, y, yaw;
  if (!nav_->getRobotPose(x, y, yaw)) return;

  if (!recovery_turning_) {
    recovery_target_yaw_ = yaw + cfg_.recovery_turn_rad;
    recovery_turning_    = true;
    RCLCPP_INFO(node_->get_logger(), "[FSM] Recovery: rotating %.0f°",
                cfg_.recovery_turn_rad * 180.0 / M_PI);
  }

  double err = wrapAngle(recovery_target_yaw_ - yaw);
  if (std::abs(err) < 0.05) {
    nav_->stopRobot();
    recovery_turning_ = false;
    last_move_time_   = node_->now();  // FIX 3: reset stuck timer at END of recovery
                                        // (startRecovery resets it at start, so robot
                                        //  only has stuck_timeout minus rotation_time
                                        //  in EXPLORING before getting stuck again).
    RCLCPP_INFO(node_->get_logger(), "[FSM] Recovery complete.");
    transitionTo(RobotState::EXPLORING);
    return;
  }

  double w = (err > 0)
    ? cfg_.max_angular_speed * 0.5
    : -cfg_.max_angular_speed * 0.5;
  nav_->cmdVel(0.0, w);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────
bool RobotStateMachine::isStuck()
{
  double x, y, yaw;
  if (!nav_->getRobotPose(x, y, yaw)) return false;

  double moved = dist2d(x, y, last_x_, last_y_);
  if (moved > 0.05) {
    last_x_ = x;
    last_y_ = y;
    last_move_time_ = node_->now();
  }

  return (node_->now() - last_move_time_).seconds() > cfg_.stuck_timeout_sec;
}

void RobotStateMachine::startRecovery()
{
  nav_->cancelGoal();
  nav_->stopRobot();
  last_move_time_   = node_->now();
  recovery_turning_ = false;
  transitionTo(RobotState::RECOVERY);
}

void RobotStateMachine::sendApproachVelocity(double target_distance, double angle)
{
  double angular = std::clamp(-1.5 * angle, -cfg_.max_angular_speed, cfg_.max_angular_speed);
  double linear  = 0.0;

  if (target_distance > cfg_.approach_distance_m + 0.05) {
    // Proportional, floored at zero so we never back up.
    linear = std::clamp(0.6 * (target_distance - cfg_.approach_distance_m),
                        0.0, cfg_.max_linear_speed);
  }
  // When the human angle is large (> ~45°), reduce forward speed and
  // prioritise turning to face them first – avoids overshooting sideways.
  if (std::abs(angle) > 0.785) {   // 45°
    linear *= 0.3;
  }

  nav_->cmdVel(linear, angular);
}

}  // namespace yahboom_nav
