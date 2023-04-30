#ifndef STELLA_NAV2_PLUGINS__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
#define STELLA_NAV2_PLUGINS__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace stella_nav2_plugins {

class RemovePassedGoals2 : public BT::ActionNodeBase {
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  RemovePassedGoals2(const std::string &xml_tag_name,
                     const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<Goals>("input_goals",
                             "Original goals to remove viapoints from"),
        BT::OutputPort<Goals>("output_goals",
                              "Goals with passed viapoints removed"),
        BT::InputPort<double>(
            "distance", 0.1,
            "distance to goal for it to be considered for removal"),
        BT::InputPort<std::string>("global_frame", std::string("map"),
                                   "Global frame"),
        BT::InputPort<std::string>("robot_base_frame", std::string("base_link"),
                                   "Robot base frame"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  double viapoint_achieved_distance_;
  std::string robot_base_frame_, global_frame_;
  double transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

} // namespace stella_nav2_plugins

#endif // STELLA_NAV2_PLUGINS__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
