cmake_minimum_required(VERSION 3.8)
project(test_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TestAction.action"
)

# Include Cpp "include" directory
include_directories(include)

set(dependencies
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)

# Action Server
add_executable(test_action_server 
  src/action_server.cpp
  src/test_waitable.cpp
)
ament_target_dependencies(test_action_server ${dependencies})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(test_action_server "${cpp_typesupport_target}")

# Action Client
add_executable(test_action_client
  src/action_client.cpp
)
ament_target_dependencies(test_action_client ${dependencies})
target_link_libraries(test_action_client "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  test_action_server
  test_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
