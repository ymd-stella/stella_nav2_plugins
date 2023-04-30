#include <limits>
#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"

#include "stella_nav2_plugins/plugins/action/remove_passed_goals_action.hpp"

namespace {
double distance(const geometry_msgs::msg::Pose &pos1,
                const geometry_msgs::msg::Pose &pos2) {
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;
  double yaw1 = tf2::getYaw(pos1.orientation);

  return dx * std::cos(yaw1) + dy * std::sin(yaw1);
}

double distance(const geometry_msgs::msg::PoseStamped &pos1,
                const geometry_msgs::msg::PoseStamped &pos2) {
  return distance(pos1.pose, pos2.pose);
}
} // namespace

namespace stella_nav2_plugins {

RemovePassedGoals2::RemovePassedGoals2(const std::string &name,
                                       const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(name, conf), viapoint_achieved_distance_(0.1) {
  getInput("distance", viapoint_achieved_distance_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);
}

inline BT::NodeStatus RemovePassedGoals2::tick() {
  setStatus(BT::NodeStatus::RUNNING);

  Goals goal_poses;
  getInput("input_goals", goal_poses);

  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_,
                                 robot_base_frame_, transform_tolerance_)) {
    return BT::NodeStatus::FAILURE;
  }

  double dist_to_goal;
  while (goal_poses.size() > 1) {
    dist_to_goal = distance(goal_poses[0].pose, current_pose.pose);

    if (dist_to_goal > viapoint_achieved_distance_) {
      break;
    }

    goal_poses.erase(goal_poses.begin());
  }

  setOutput("output_goals", goal_poses);

  return BT::NodeStatus::SUCCESS;
}

} // namespace stella_nav2_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<stella_nav2_plugins::RemovePassedGoals2>(
      "RemovePassedGoals2");
}
