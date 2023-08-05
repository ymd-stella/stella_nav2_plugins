cmake_minimum_required(VERSION 3.5)
project(stella_nav2_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stella_interfaces REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  stella_interfaces
  nav2_common
  nav2_core
  nav2_msgs
  nav2_util
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
)

include_directories(
  include
)

add_library(stella_nav2_remove_passed_goals_action_bt_node SHARED plugins/action/remove_passed_goals_action.cpp)
ament_target_dependencies(stella_nav2_remove_passed_goals_action_bt_node ${dependencies})
target_compile_definitions(stella_nav2_remove_passed_goals_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(stella_wait_for_button_behavior SHARED
  plugins/wait_for_button.cpp
)
ament_target_dependencies(stella_wait_for_button_behavior
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  stella_nav2_remove_passed_goals_action_bt_node
  stella_wait_for_button_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  stella_nav2_remove_passed_goals_action_bt_node
  stella_wait_for_button_behavior
)

ament_export_dependencies(${dependencies})

ament_package()
