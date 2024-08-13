#include "stella_nav2_plugins/plugins/decorator/time_based_retry.hpp"

namespace stella_nav2_plugins {

TimeBasedRetry::TimeBasedRetry(const std::string &name,
                               const BT::NodeConfiguration &conf)
    : BT::DecoratorNode(name, conf), failed_state_initialized_(false),
      running_state_initialized_(false), duration_(1.0), tolerance_(0.5) {
  getInput("topic", topic_);
  getInput("duration", duration_);
  getInput("tolerance", tolerance_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  pub_ = node_->create_publisher<std_msgs::msg::Float32>(topic_, 10);
}

inline BT::NodeStatus TimeBasedRetry::tick() {
  if (status() == BT::NodeStatus::IDLE) {
    running_state_initialized_ = false;
    failed_state_initialized_ = false;
  }

  const BT::NodeStatus child_state = child_node_->executeTick();

  switch (child_state) {
  case BT::NodeStatus::RUNNING:
    if (failed_state_initialized_) {
      if (!running_state_initialized_) {
        running_start_time_ = node_->now();
        running_state_initialized_ = true;
      }
    }
    if (running_state_initialized_) {
      const auto time_elapsed = (node_->now() - running_start_time_).seconds();
      if (time_elapsed >= tolerance_) {
        failed_state_initialized_ = false;
        running_state_initialized_ = false;
      }
    }
    return child_state;

  case BT::NodeStatus::SUCCESS:
    running_state_initialized_ = false;
    failed_state_initialized_ = false;
    return child_state;

  case BT::NodeStatus::FAILURE: {
    if (!failed_state_initialized_) {
      failed_start_time_ = node_->now();
      failed_state_initialized_ = true;
      running_state_initialized_ = false;
    }
    if (running_state_initialized_) {
      running_state_initialized_ = false;
    }
    const auto time_elapsed = (node_->now() - failed_start_time_).seconds();
    std_msgs::msg::Float32 msg;
    msg.data = duration_ - time_elapsed;
    pub_->publish(msg);
    if (time_elapsed < duration_) {
      return BT::NodeStatus::RUNNING;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }

  default:
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace stella_nav2_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<stella_nav2_plugins::TimeBasedRetry>(
      "TimeBasedRetry");
}
