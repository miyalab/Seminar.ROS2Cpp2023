cmake_minimum_required(VERSION 3.8)
project(tutorial_opencv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/image_subscriber.cpp
  src/image_publisher.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  OpenCV
  cv_bridge
)

rclcpp_components_register_nodes(${PROJECT_NAME} "ROS2Tutorial::ImageSubscriber")
rclcpp_components_register_nodes(${PROJECT_NAME} "ROS2Tutorial::ImagePublisher")

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# cfg
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/)

# launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
