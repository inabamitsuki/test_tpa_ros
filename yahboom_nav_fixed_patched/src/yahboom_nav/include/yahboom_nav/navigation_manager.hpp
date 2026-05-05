#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  yahboom_nav / navigation_manager.hpp
//  Wraps Nav2 NavigateToPose action + direct cmd_vel for close-range manoeuvres
// ─────────────────────────────────────────────────────────────────────────────

#include "yahboom_nav/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <optional>
#include <vector>
#include <mutex>
#include <atomic>

namespace yahboom_nav {

enum class NavStatus { IDLE, RUNNING, SUCCEEDED, FAILED, CANCELLED };

class NavigationManager {
public:
  explicit NavigationManager(rclcpp::Node::SharedPtr node, const NavConfig& cfg);

  // ── Nav2 goal interface ────────────────────────────────────────────────────
  // Returns false if the Nav2 action server is not yet available (caller should
  // stay in EXPLORING rather than transitioning to NAVIGATING_TO_GOAL).
  bool sendGoal(double x, double y, double yaw);
  void cancelGoal();
  NavStatus getStatus() const { return nav_status_; }

  // ── Direct cmd_vel (bypasses Nav2 – used for close approach & recovery) ────
  void cmdVel(double linear, double angular);
  void stopRobot();

  // ── Odometry / pose ────────────────────────────────────────────────────────
  bool getRobotPose(double& x, double& y, double& yaw);

  // FIX: returns true only when a valid map→base_link TF transform exists,
  // meaning AMCL has converged and published a localised pose. Unlike
  // getRobotPose() this never falls back to odometry, so pose_initialised_
  // in the FSM can be gated on AMCL readiness rather than odom availability.
  bool hasTfPose();

  // ── Frontier exploration ───────────────────────────────────────────────────
  // Returns next unexplored frontier goal (or nullopt if map not ready)
  std::optional<geometry_msgs::msg::PoseStamped> nextFrontierGoal(
      const nav_msgs::msg::OccupancyGrid& map);

  // ── Map access ────────────────────────────────────────────────────────────
  nav_msgs::msg::OccupancyGrid::SharedPtr getMap() const;

private:
  using Nav2Action = nav2_msgs::action::NavigateToPose;
  using GoalHandle  = rclcpp_action::ClientGoalHandle<Nav2Action>;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  const NavConfig&        cfg_;

  // Nav2 action client
  rclcpp_action::Client<Nav2Action>::SharedPtr nav_client_;
  GoalHandle::SharedPtr                        goal_handle_;
  std::atomic<NavStatus>                       nav_status_{NavStatus::IDLE};

  // cmd_vel publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  // Odom subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  mutable std::mutex odom_mutex_;

  // Map subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  mutable std::mutex map_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // frontier cache
  std::vector<geometry_msgs::msg::PoseStamped> frontier_cache_;
  size_t frontier_idx_{0};
};

}  // namespace yahboom_nav
