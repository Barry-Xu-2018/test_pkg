cmake_minimum_required(VERSION 3.8)
project(test_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
)

add_library(${PROJECT_NAME}_timer_publish_composable_node SHARED src/timer_publish.cpp)
ament_target_dependencies(${PROJECT_NAME}_timer_publish_composable_node ${dependencies})
rclcpp_components_register_node(${PROJECT_NAME}_timer_publish_composable_node
  PLUGIN "${PROJECT_NAME}::TimerPublishComposableNode"
  EXECUTABLE timer_publish_node
)

add_library(${PROJECT_NAME}_subscribe_composable_node SHARED src/subscribe.cpp)
ament_target_dependencies(${PROJECT_NAME}_subscribe_composable_node ${dependencies})
rclcpp_components_register_node(${PROJECT_NAME}_subscribe_composable_node
  PLUGIN "${PROJECT_NAME}::SubscribeComposableNode"
  EXECUTABLE subscribe_node
)

install(TARGETS
  ${PROJECT_NAME}_timer_publish_composable_node
  ${PROJECT_NAME}_subscribe_composable_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
