cmake_minimum_required(VERSION 3.8)
project(turtle_locator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

set(NODES
  locator
)

foreach(target IN LISTS NODES)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} rclcpp turtlesim geometry_msgs)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
