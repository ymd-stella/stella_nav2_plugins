#include "stella_nav2_plugins/plugins/action/publish_path_action.hpp"
#include <memory>
#include <string>

namespace stella_nav2_plugins {

PublishPath::PublishPath(const std::string &name,
                         const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(name, conf) {
  getInput("topic", topic_);
  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
  pub_ = node_->create_publisher<nav_msgs::msg::Path>(topic_, 10);
}

inline BT::NodeStatus PublishPath::tick() {
  nav_msgs::msg::Path msg;
  getInput("path", msg);
  pub_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

} // namespace stella_nav2_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<stella_nav2_plugins::PublishPath>("PublishPath");
}
