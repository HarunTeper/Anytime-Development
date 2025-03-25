cmake_minimum_required(VERSION 3.8)
project(anytime_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "action/MonteCarlo.action"
  "action/Yolo.action"
)

set(dependencies
  vision_msgs
  sensor_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES ${dependencies}
)

ament_package()
