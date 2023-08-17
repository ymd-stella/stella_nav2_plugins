#include "stella_nav2_plugins/plugins/goal_checker.hpp"
#include "angles/angles.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace stella {
GoalChecker::GoalChecker() {}

void GoalChecker::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string &plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name + ".x_tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name + ".yaw_tolerance", rclcpp::ParameterValue(0.5));

  node->get_parameter(plugin_name + ".x_tolerance", x_tolerance_);
  node->get_parameter(plugin_name + ".yaw_tolerance", yaw_tolerance_);
}

void GoalChecker::reset() { rotating = false; }

bool GoalChecker::isGoalReached(const geometry_msgs::msg::Pose &query_pose,
                                const geometry_msgs::msg::Pose &goal_pose,
                                const geometry_msgs::msg::Twist &velocity) {
  double goal_yaw = tf2::getYaw(goal_pose.orientation);
  double query_yaw = tf2::getYaw(query_pose.orientation);
  if (!rotating) {
    double dx = goal_pose.position.x - query_pose.position.x;
    double dy = goal_pose.position.y - query_pose.position.y;
    double distance_to_goal = dx * std::cos(goal_yaw) + dy * std::sin(goal_yaw);
    if (distance_to_goal > x_tolerance_) {
      return false;
    }

    rotating = true;
  }

  return fabs(angles::shortest_angular_distance(query_yaw, goal_yaw)) <
         yaw_tolerance_;
}

bool GoalChecker::getTolerances(geometry_msgs::msg::Pose &pose_tolerance,
                                geometry_msgs::msg::Twist &vel_tolerance) {
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = x_tolerance_;
  pose_tolerance.position.y = invalid_field;
  pose_tolerance.position.z = invalid_field;

  // unused?
  pose_tolerance.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yaw_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

} // namespace stella

PLUGINLIB_EXPORT_CLASS(stella::GoalChecker, nav2_core::GoalChecker)
