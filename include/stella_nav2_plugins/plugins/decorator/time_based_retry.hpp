#ifndef STELLA_NAV2_PLUGINS__PLUGINS__DECORATOR__TIME_BASED_RETRY_HPP_
#define STELLA_NAV2_PLUGINS__PLUGINS__DECORATOR__TIME_BASED_RETRY_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace stella_nav2_plugins {

class TimeBasedRetry : public BT::DecoratorNode {
public:
  TimeBasedRetry(const std::string &name, const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("topic", "time_left", "output topic name"),
        BT::InputPort<double>("duration", 1.0,
                              "Duration until FAILURE is returned if "
                              "child's status is FAILURE"),
        BT::InputPort<double>("tolerance", 0.5,
                              "Duration until measurement is interrupted "
                              "when child's status is RUNNING")};
  }

private:
  BT::NodeStatus tick() override;

  double duration_;
  double tolerance_;
  bool failed_state_initialized_;
  bool running_state_initialized_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time failed_start_time_;
  rclcpp::Time running_start_time_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  std::string topic_;
};

} // namespace stella_nav2_plugins

#endif // STELLA_NAV2_PLUGINS__PLUGINS__DECORATOR__TIME_BASED_RETRY_HPP_
