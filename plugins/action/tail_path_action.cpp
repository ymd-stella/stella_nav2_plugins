#include "stella_nav2_plugins/plugins/action/tail_path_action.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include <limits>
#include <memory>
#include <string>

namespace stella_nav2_plugins {

TailPath::TailPath(const std::string &name, const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(name, conf), distance_(1.0) {
  getInput("distance", distance_);
}

inline BT::NodeStatus TailPath::tick() {
  nav_msgs::msg::Path input_path;
  getInput("input_path", input_path);
  if (input_path.poses.empty()) {
    setOutput("output_path", input_path);
    return BT::NodeStatus::SUCCESS;
  }

  nav_msgs::msg::Path output_path;
  output_path.header = input_path.header;
  geometry_msgs::msg::PoseStamped pose = input_path.poses.back();
  double integrated_distance = 0.0;
  while (integrated_distance < distance_) {
    output_path.poses.push_back(input_path.poses.back());
    input_path.poses.pop_back();
    if (input_path.poses.size() == 0) {
      break;
    }
    integrated_distance = nav2_util::geometry_utils::euclidean_distance(
        input_path.poses.back(), pose);
  }
  std::reverse(output_path.poses.begin(), output_path.poses.end());
  setOutput("output_path", output_path);
  return BT::NodeStatus::SUCCESS;
}

} // namespace stella_nav2_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<stella_nav2_plugins::TailPath>("TailPath");
}
