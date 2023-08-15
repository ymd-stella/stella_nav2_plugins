#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include <memory>
#include <string>

namespace stella_nav2_plugins {

class AddTwoFloat : public BT::ActionNodeBase {
public:
  AddTwoFloat(const std::string &xml_tag_name,
              const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("a", "a"),
        BT::InputPort<double>("b", "b"),
        BT::OutputPort<double>("output", "output"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
};

} // namespace stella_nav2_plugins
