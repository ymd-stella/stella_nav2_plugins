#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "nav2_behaviors/timed_behavior.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "stella_interfaces/action/wait_for_button.hpp"

namespace stella {
using WaitForButtonAction = stella_interfaces::action::WaitForButton;

class WaitForButton
    : public nav2_behaviors::TimedBehavior<WaitForButtonAction> {
public:
  using WaitActionGoal = WaitForButtonAction::Goal;

  WaitForButton();
  ~WaitForButton();

  void onConfigure() override;

  void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr &joy_msg);

  nav2_behaviors::Status
  onRun(const std::shared_ptr<const WaitActionGoal> command) override;

  nav2_behaviors::Status onCycleUpdate() override;

protected:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  int button_id_ = -1;
  bool pressed_ = false;
};

} // namespace stella
