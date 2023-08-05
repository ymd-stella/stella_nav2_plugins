#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include <memory>
#include <string>

namespace stella_nav2_plugins {

class TailPath : public BT::ActionNodeBase {
public:
  TailPath(const std::string &xml_tag_name, const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<nav_msgs::msg::Path>("input_path", "input path"),
        BT::OutputPort<nav_msgs::msg::Path>("output_path", "output path"),
        BT::InputPort<double>("distance", 1.0, "distance"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
  double distance_;
};

} // namespace stella_nav2_plugins
