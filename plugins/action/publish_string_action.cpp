#include "stella_nav2_plugins/plugins/action/publish_string_action.hpp"
#include <memory>
#include <string>

namespace stella_nav2_plugins {

PublishString::PublishString(const std::string &name,
                             const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(name, conf) {
  getInput("topic", topic_);
  getInput("data", data_);
  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
  pub_ = node_->create_publisher<std_msgs::msg::String>(topic_, 10);
}

inline BT::NodeStatus PublishString::tick() {
  std_msgs::msg::String msg;
  msg.data = data_;
  pub_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

} // namespace stella_nav2_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<stella_nav2_plugins::PublishString>("PublishString");
}
