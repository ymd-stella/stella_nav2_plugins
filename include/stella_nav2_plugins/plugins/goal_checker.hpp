#pragma once

#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stella {

class GoalChecker : public nav2_core::GoalChecker {
public:
  GoalChecker();
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                  const std::string &plugin_name,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
                      costmap_ros) override;
  void reset() override;
  bool isGoalReached(const geometry_msgs::msg::Pose &query_pose,
                     const geometry_msgs::msg::Pose &goal_pose,
                     const geometry_msgs::msg::Twist &velocity) override;
  bool getTolerances(geometry_msgs::msg::Pose &pose_tolerance,
                     geometry_msgs::msg::Twist &vel_tolerance) override;

private:
  double x_tolerance_; // x_tolerance is used by controllers.
  double yaw_tolerance_;
  bool rotating;
};

} // namespace stella
