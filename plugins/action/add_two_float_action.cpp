#include "stella_nav2_plugins/plugins/action/add_two_float_action.hpp"
#include <memory>
#include <string>

namespace stella_nav2_plugins {

AddTwoFloat::AddTwoFloat(const std::string &name,
                         const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(name, conf) {}

inline BT::NodeStatus AddTwoFloat::tick() {
  double a, b;
  getInput("a", a);
  getInput("b", b);
  setOutput("output", a + b);
  return BT::NodeStatus::SUCCESS;
}

} // namespace stella_nav2_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<stella_nav2_plugins::AddTwoFloat>("AddTwoFloat");
}
