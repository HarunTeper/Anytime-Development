cmake_minimum_required(VERSION 3.5)
project(yolo_action)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH /home/vscode/libraries/libtorch)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(anytime_interfaces REQUIRED)
find_package(Torch REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

add_library(yolo_rt_model SHARED src/yolo_model.cpp)
set(yolo_rt_dependencies rclcpp Torch nlohmann_json sensor_msgs)
ament_target_dependencies(yolo_rt_model ${yolo_rt_dependencies})
target_link_libraries(yolo_rt_model "${TORCH_LIBRARIES}")

set(anytime_dependencies "anytime_interfaces" "rclcpp" "rclcpp_action" "rclcpp_components")

add_library(anytime_client SHARED src/anytime_client.cpp)
# target_include_directories(anytime_client PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

ament_target_dependencies(anytime_client ${anytime_dependencies})
rclcpp_components_register_node(anytime_client PLUGIN "AnytimeActionClient" EXECUTABLE anytime_action_client)
install(TARGETS
  anytime_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(anytime_server SHARED src/anytime_server.cpp)
# target_include_directories(anytime_server PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
ament_target_dependencies(anytime_server ${anytime_dependencies})
rclcpp_components_register_node(anytime_server PLUGIN "AnytimeActionServer" EXECUTABLE anytime_action_server)
target_link_libraries(anytime_server yolo_rt_model)
install(TARGETS
  anytime_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY
  launch
  weights
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
