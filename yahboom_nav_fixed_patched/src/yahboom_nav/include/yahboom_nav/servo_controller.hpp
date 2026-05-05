#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  yahboom_nav / servo_controller.hpp
//
//  CHANGES vs previous version:
//    Switched from the /servo_control + (channel<<12)|pwm_us encoding to the
//    simpler /servo_s1 + plain Int32 angle interface used by the Yahboom
//    firmware on that topic (same as test_drop.py).
//
//    Old interface:  topic=/servo_control  value=(channel<<12)|pwm_us
//    New interface:  topic=/servo_s1       value=angle (e.g. -50 = open, 0 = closed)
//
//  The public API (dropCube / holdCube / isDropping) is unchanged so
//  robot_state_machine.cpp needs no edits.
// ─────────────────────────────────────────────────────────────────────────────

#include "yahboom_nav/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace yahboom_nav {

class ServoController {
public:
  explicit ServoController(rclcpp::Node::SharedPtr node, const NavConfig& cfg);

  // Non-blocking – call every tick; returns true when the drop sequence finishes
  bool dropCube();

  // Park servo at the closed/hold angle
  void holdCube();

  bool isDropping() const { return dropping_; }

private:
  // Publish a plain angle value to /servo_s1
  void publishAngle(int angle);

  rclcpp::Node::SharedPtr  node_;
  const NavConfig&         cfg_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr servo_pub_;

  bool         dropping_{false};
  rclcpp::Time drop_start_{0, 0, RCL_ROS_TIME};
};

}  // namespace yahboom_nav
