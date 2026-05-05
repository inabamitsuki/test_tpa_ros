// ─────────────────────────────────────────────────────────────────────────────
//  yahboom_nav / servo_controller.cpp
//
//  CHANGES vs previous version:
//    Replaced the (channel<<12)|pwm_us PWM encoding with plain Int32 angle
//    values published to /servo_s1, matching the interface in test_drop.py:
//      servo_open_angle   (default -50) → open / drop
//      servo_closed_angle (default   0) → closed / hold
//
//    All previous fixes (debug log, startup hold, periodic refresh) are kept.
// ─────────────────────────────────────────────────────────────────────────────

#include "yahboom_nav/servo_controller.hpp"
#include <algorithm>

namespace yahboom_nav {

ServoController::ServoController(rclcpp::Node::SharedPtr node, const NavConfig& cfg)
  : node_(node), cfg_(cfg)
{
  servo_pub_ = node_->create_publisher<std_msgs::msg::Int32>(cfg_.servo_topic, 10);

  // Park servo closed on startup
  holdCube();

  RCLCPP_INFO(node_->get_logger(),
    "[ServoController] Initialised. topic='%s'  "
    "open_angle=%d  closed_angle=%d  hold=%.1f s",
    cfg_.servo_topic.c_str(),
    cfg_.servo_open_angle,
    cfg_.servo_closed_angle,
    cfg_.servo_hold_sec);
}

// ─────────────────────────────────────────────────────────────────────────────
//  publishAngle – sends a plain angle value to /servo_s1
// ─────────────────────────────────────────────────────────────────────────────
void ServoController::publishAngle(int angle)
{
  std_msgs::msg::Int32 msg;
  msg.data = angle;
  servo_pub_->publish(msg);

  RCLCPP_DEBUG(node_->get_logger(),
    "[Servo] publish angle=%d → topic=%s", angle, cfg_.servo_topic.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  dropCube – non-blocking state machine, returns true when sequence done.
//  Matches the DROPPING / COOLDOWN logic from the fixed test_drop.py.
// ─────────────────────────────────────────────────────────────────────────────
bool ServoController::dropCube()
{
  auto now = node_->now();

  if (!dropping_) {
    RCLCPP_INFO(node_->get_logger(),
      "[Servo] Opening (angle=%d).", cfg_.servo_open_angle);
    publishAngle(cfg_.servo_open_angle);
    dropping_   = true;
    drop_start_ = now;
    return false;
  }

  double elapsed = (now - drop_start_).seconds();

  if (elapsed >= cfg_.servo_hold_sec) {
    RCLCPP_INFO(node_->get_logger(),
      "[Servo] Closing after %.2f s (angle=%d).",
      elapsed, cfg_.servo_closed_angle);
    publishAngle(cfg_.servo_closed_angle);
    dropping_ = false;
    return true;
  }

  // Still holding open – republish periodically (some firmware needs refreshes)
  publishAngle(cfg_.servo_open_angle);
  return false;
}

// ─────────────────────────────────────────────────────────────────────────────
void ServoController::holdCube()
{
  dropping_ = false;
  publishAngle(cfg_.servo_closed_angle);
}

}  // namespace yahboom_nav
