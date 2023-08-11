#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace stella_nav2_plugins {

class PublishString : public BT::ActionNodeBase {
public:
  PublishString(const std::string &xml_tag_name,
                const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("topic", "output topic name"),
        BT::InputPort<std::string>("data", "string message"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::string topic_;
  std::string data_;
};

} // namespace stella_nav2_plugins
