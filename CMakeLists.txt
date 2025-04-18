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
find_package(nav2_behaviors REQUIRED)
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
  nav2_behaviors
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

add_library(stella_nav2_tail_path_action_bt_node SHARED plugins/action/tail_path_action.cpp)
ament_target_dependencies(stella_nav2_tail_path_action_bt_node ${dependencies})
target_compile_definitions(stella_nav2_tail_path_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(stella_nav2_publish_string_action_bt_node SHARED plugins/action/publish_string_action.cpp)
ament_target_dependencies(stella_nav2_publish_string_action_bt_node ${dependencies})
target_compile_definitions(stella_nav2_publish_string_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(stella_nav2_publish_path_action_bt_node SHARED plugins/action/publish_path_action.cpp)
ament_target_dependencies(stella_nav2_publish_path_action_bt_node ${dependencies})
target_compile_definitions(stella_nav2_publish_path_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(stella_nav2_add_two_float_action_bt_node SHARED plugins/action/add_two_float_action.cpp)
ament_target_dependencies(stella_nav2_add_two_float_action_bt_node ${dependencies})
target_compile_definitions(stella_nav2_add_two_float_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(stella_nav2_time_based_retry_bt_node SHARED plugins/decorator/time_based_retry.cpp)
ament_target_dependencies(stella_nav2_time_based_retry_bt_node ${dependencies})
target_compile_definitions(stella_nav2_time_based_retry_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(stella_wait_for_button_behavior SHARED
  plugins/wait_for_button.cpp
)
ament_target_dependencies(stella_wait_for_button_behavior
  ${dependencies}
)

add_library(stella_goal_checker SHARED
  plugins/goal_checker.cpp
)
ament_target_dependencies(stella_goal_checker
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  stella_nav2_tail_path_action_bt_node
  stella_nav2_publish_string_action_bt_node
  stella_nav2_publish_path_action_bt_node
  stella_nav2_add_two_float_action_bt_node
  stella_nav2_time_based_retry_bt_node
  stella_wait_for_button_behavior
  stella_goal_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  stella_nav2_tail_path_action_bt_node
  stella_nav2_publish_string_action_bt_node
  stella_nav2_publish_path_action_bt_node
  stella_nav2_add_two_float_action_bt_node
  stella_nav2_time_based_retry_bt_node
  stella_wait_for_button_behavior
  stella_goal_checker
)

ament_export_dependencies(${dependencies})

ament_package()
