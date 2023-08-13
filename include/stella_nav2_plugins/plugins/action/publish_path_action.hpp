#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace stella_nav2_plugins {

class PublishPath : public BT::ActionNodeBase {
public:
  PublishPath(const std::string &xml_tag_name,
              const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("topic", "output topic name"),
        BT::InputPort<nav_msgs::msg::Path>("path", "path message"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  std::string topic_;
  nav_msgs::msg::Path path_;
};

} // namespace stella_nav2_plugins
