cmake_minimum_required(VERSION 3.8)
project(semi_robot_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# msg定義
set(MSG_FILE
  "msg/RobotState.msg"
)

# msg型生成
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILE}
  DEPENDENCIES sensor_msgs geometry_msgs nav_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
