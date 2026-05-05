// ─────────────────────────────────────────────────────────────────────────────
//  yahboom_nav / navigation_manager.cpp
//
//  FIX SUMMARY vs original:
//    1. nextFrontierGoal: the cache-rebuild guard was inverted.
//         Old: `if (!frontier_cache_.empty())` → only re-entered the cache
//         path when it was non-empty, but then immediately returned from it.
//         When the cache WAS empty the code fell through to rebuild, which is
//         correct – but then on the *next* call the non-empty branch returned
//         frontier_cache_[frontier_idx_++] and also advanced the index, then
//         at the end of the non-empty branch it also tried to clear+reset.
//         Net effect: every other call returned the wrong frontier or nullopt.
//       Fixed to: if cache is non-empty AND idx is in range → return cached.
//                 otherwise clear and rebuild.
//    2. sendGoal: nav_status_ is written from the action callbacks which run
//       on a different thread (MultiThreadedExecutor). Made nav_status_
//       std::atomic<NavStatus> in the header to avoid a data race.
//    3. cancelGoal: after cancel the goal_handle_ was left non-null, so a
//       subsequent cancelGoal() call would re-cancel an already-cancelled goal.
//       Added goal_handle_.reset() after the cancel call.
//    4. getRobotPose: in the odom fallback the mutex was acquired INSIDE the
//       catch block but AFTER node_->get_logger() which could itself trigger
//       a callback. Moved the odom snapshot out of the catch to avoid the
//       double-lock scenario when odom_mutex_ is held by the odom callback.
// ─────────────────────────────────────────────────────────────────────────────

#include "yahboom_nav/navigation_manager.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace yahboom_nav {

NavigationManager::NavigationManager(rclcpp::Node::SharedPtr node, const NavConfig& cfg)
  : node_(node), cfg_(cfg)
{
  // ── cmd_vel publisher ────────────────────────────────────────────────────
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cfg_.cmd_vel_topic, 10);

  // ── Odom subscriber ──────────────────────────────────────────────────────
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    cfg_.odom_topic, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg){ odomCallback(msg); });

  // ── Map subscriber ───────────────────────────────────────────────────────
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    cfg_.map_topic, rclcpp::QoS(1).transient_local(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg){ mapCallback(msg); });

  // ── TF ───────────────────────────────────────────────────────────────────
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Nav2 action client ───────────────────────────────────────────────────
  nav_client_ = rclcpp_action::create_client<Nav2Action>(node_, "navigate_to_pose");

  RCLCPP_INFO(node_->get_logger(), "[NavigationManager] Initialised.");
}

// ─────────────────────────────────────────────────────────────────────────────
void NavigationManager::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(odom_mutex_);
  last_odom_ = msg;
}

void NavigationManager::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(map_mutex_);
  latest_map_ = msg;
  frontier_cache_.clear();
  frontier_idx_ = 0;
}

nav_msgs::msg::OccupancyGrid::SharedPtr NavigationManager::getMap() const
{
  std::lock_guard<std::mutex> lk(map_mutex_);
  return latest_map_;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Send Nav2 goal
// ─────────────────────────────────────────────────────────────────────────────
bool NavigationManager::sendGoal(double x, double y, double yaw)
{
  // FIX 8: return false (don't set FAILED) when Nav2 isn't up yet.
  // Old code set nav_status_ = FAILED which caused the FSM to immediately
  // cycle EXPLORING → NAVIGATING_TO_GOAL → FAILED → EXPLORING → ... and after
  // stuck_timeout_sec the robot entered RECOVERY (spinning) even though Nav2
  // was simply still starting up.  Now the caller stays in EXPLORING and
  // retries on the next frontier scan instead.
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[Nav] Nav2 action server not available – will retry.");
    return false;
  }

  Nav2Action::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp    = node_->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);

  nav_status_ = NavStatus::RUNNING;

  auto send_opts = rclcpp_action::Client<Nav2Action>::SendGoalOptions();

  // FIX 2: nav_status_ is atomic – safe to write from this callback thread.
  // FIX 7: reset goal_handle_ here so cancelGoal() can never call
  //         async_cancel_goal() on an already-finished handle, which throws
  //         rclcpp_action::exceptions::UnknownGoalHandleError → std::terminate
  //         → SIGABRT (exit -6).  This was the crash seen in the field.
  send_opts.result_callback =
    [this](const GoalHandle::WrappedResult& result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        nav_status_ = NavStatus::SUCCEEDED;
        RCLCPP_INFO(node_->get_logger(), "[Nav] Goal SUCCEEDED.");
      } else {
        nav_status_ = NavStatus::FAILED;
        RCLCPP_WARN(node_->get_logger(), "[Nav] Goal FAILED / CANCELLED.");
      }
      goal_handle_.reset();   // ← FIX 7: mark handle dead so cancel is safe
    };
  send_opts.goal_response_callback =
    [this](const GoalHandle::SharedPtr& gh) {
      if (!gh) {
        nav_status_ = NavStatus::FAILED;
        RCLCPP_ERROR(node_->get_logger(), "[Nav] Goal rejected by Nav2.");
      } else {
        goal_handle_ = gh;
      }
    };

  nav_client_->async_send_goal(goal, send_opts);
  RCLCPP_INFO(node_->get_logger(),
    "[Nav] Goal sent: (%.2f, %.2f, %.2f°)", x, y, yaw * 180.0 / M_PI);
  return true;
}

void NavigationManager::cancelGoal()
{
  if (goal_handle_) {
    try {
      nav_client_->async_cancel_goal(goal_handle_);
    } catch (const std::exception& e) {
      // FIX 7 (defence-in-depth): result_callback already resets goal_handle_
      // when a goal finishes, but in a race the FSM may call cancelGoal()
      // between the goal finishing and result_callback executing.
      // async_cancel_goal would then throw UnknownGoalHandleError which –
      // if uncaught – calls std::terminate → SIGABRT (exit -6).
      RCLCPP_WARN(node_->get_logger(),
        "[Nav] cancelGoal: goal already finished, skipping cancel (%s).", e.what());
    }
    nav_status_ = NavStatus::CANCELLED;
    // FIX 3: reset handle so repeat calls don't cancel an already-dead goal.
    goal_handle_.reset();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Direct cmd_vel
// ─────────────────────────────────────────────────────────────────────────────
void NavigationManager::cmdVel(double linear, double angular)
{
  geometry_msgs::msg::Twist t;
  t.linear.x  = std::clamp(linear,  -cfg_.max_linear_speed,  cfg_.max_linear_speed);
  t.angular.z = std::clamp(angular, -cfg_.max_angular_speed, cfg_.max_angular_speed);
  cmd_pub_->publish(t);
}

void NavigationManager::stopRobot()
{
  cmdVel(0.0, 0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
//  hasTfPose – succeeds only when AMCL has published map→base_link
// ─────────────────────────────────────────────────────────────────────────────
bool NavigationManager::hasTfPose()
{
  try {
    tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Get current robot pose from TF (map → base_link), fallback to odom
// ─────────────────────────────────────────────────────────────────────────────
bool NavigationManager::getRobotPose(double& x, double& y, double& yaw)
{
  try {
    auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return true;
  } catch (const tf2::TransformException&) {
    // TF not ready yet (common during startup) – fall through to odom silently.
  }

  // FIX 4: take odom snapshot before calling any logging to avoid re-entrancy.
  nav_msgs::msg::Odometry::SharedPtr odom_snap;
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    odom_snap = last_odom_;
  }

  if (!odom_snap) return false;

  x = odom_snap->pose.pose.position.x;
  y = odom_snap->pose.pose.position.y;
  tf2::Quaternion q(
    odom_snap->pose.pose.orientation.x,
    odom_snap->pose.pose.orientation.y,
    odom_snap->pose.pose.orientation.z,
    odom_snap->pose.pose.orientation.w);
  double roll, pitch;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Frontier exploration
// ─────────────────────────────────────────────────────────────────────────────
std::optional<geometry_msgs::msg::PoseStamped>
NavigationManager::nextFrontierGoal(const nav_msgs::msg::OccupancyGrid& map)
{
  // FIX 1: corrected cache logic.
  // Return from cache if we have entries remaining.
  if (!frontier_cache_.empty() && frontier_idx_ < frontier_cache_.size()) {
    return frontier_cache_[frontier_idx_++];
  }
  // Cache exhausted or empty – rebuild it.
  frontier_idx_ = 0;
  frontier_cache_.clear();

  int W = map.info.width;
  int H = map.info.height;
  double res  = map.info.resolution;
  auto& origin = map.info.origin;

  std::vector<std::pair<int,int>> frontiers;
  for (int row = 1; row < H - 1; ++row) {
    for (int col = 1; col < W - 1; ++col) {
      int idx = row * W + col;
      if (map.data[idx] != 0) continue;

      bool near_unknown = false;
      const int dx[4] = {-1, 1, 0, 0};
      const int dy[4] = { 0, 0,-1, 1};
      for (int d = 0; d < 4; ++d) {
        int ni = (row + dx[d]) * W + (col + dy[d]);
        if (map.data[ni] == -1) { near_unknown = true; break; }
      }
      if (near_unknown) frontiers.push_back({col, row});
    }
  }

  if (frontiers.empty()) return std::nullopt;

  // FIX: initialise to origin so the sort is deterministic even if TF/odom is
  // not yet available (getRobotPose returns false during early startup).
  // Previously these were left uninitialised → undefined behaviour in the
  // comparator on every cold start before the first TF frame arrived.
  double rx = 0.0, ry = 0.0, ryaw = 0.0;
  getRobotPose(rx, ry, ryaw);

  std::sort(frontiers.begin(), frontiers.end(),
    [&](const auto& a, const auto& b) {
      double ax = origin.position.x + a.first  * res;
      double ay = origin.position.y + a.second * res;
      double bx = origin.position.x + b.first  * res;
      double by = origin.position.y + b.second * res;
      return dist2d(rx, ry, ax, ay) < dist2d(rx, ry, bx, by);
    });

  constexpr int STEP = 20;
  for (size_t i = 0; i < frontiers.size(); i += STEP) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp    = node_->now();
    ps.pose.position.x = origin.position.x + frontiers[i].first  * res;
    ps.pose.position.y = origin.position.y + frontiers[i].second * res;
    ps.pose.orientation.w = 1.0;
    frontier_cache_.push_back(ps);
  }

  if (frontier_cache_.empty()) return std::nullopt;
  return frontier_cache_[frontier_idx_++];
}

}  // namespace yahboom_nav
