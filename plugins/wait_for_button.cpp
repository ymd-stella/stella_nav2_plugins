#include <chrono>
#include <memory>

#include "wait_for_button.hpp"

namespace stella {

WaitForButton::WaitForButton()
    : nav2_behaviors::TimedBehavior<WaitForButtonAction>() {}

WaitForButton::~WaitForButton() = default;

void WaitForButton::onConfigure() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&WaitForButton::joy_callback, this, std::placeholders::_1));
}

void WaitForButton::joy_callback(
    const sensor_msgs::msg::Joy::ConstSharedPtr &joy_msg) {
  assert(button_id_ > 0);
  if (joy_msg->buttons[button_id_]) {
    pressed_ = true;
  }
}

nav2_behaviors::Status WaitForButton::onRun(
    const std::shared_ptr<const WaitForButtonAction::Goal> command) {
  button_id_ = command->button_id;
  pressed_ = false;

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status WaitForButton::onCycleUpdate() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  assert(button_id_ > 0);
  if (pressed_) {
    return nav2_behaviors::Status::SUCCEEDED;
  }
  return nav2_behaviors::Status::RUNNING;
}

} // namespace stella

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(stella::WaitForButton, nav2_core::Behavior)
