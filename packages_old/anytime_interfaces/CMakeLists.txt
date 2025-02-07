cmake_minimum_required(VERSION 3.8)
project(anytime_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Anytime.action"
)

ament_package()
