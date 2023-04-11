cmake_minimum_required(VERSION 3.8)
project(tutorial_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# msg定義
set(MSG_FILE
  "msg/Int32Stamped.msg"
)

# msg型生成
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILE}
  DEPENDENCIES std_msgs
)

ament_package()
