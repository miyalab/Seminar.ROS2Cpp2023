cmake_minimum_required(VERSION 3.8)
project(tutorial_component)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/subscriber.cpp
  src/publisher.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_nodes(${PROJECT_NAME} "ROS2TutorialComponent::Subscriber")
rclcpp_components_register_nodes(${PROJECT_NAME} "ROS2TutorialComponent::Publisher")

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# cfg
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/)

# launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
