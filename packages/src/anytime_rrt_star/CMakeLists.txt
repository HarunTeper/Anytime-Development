cmake_minimum_required(VERSION 3.5)
project(anytime_rrt_star)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(anytime_interfaces REQUIRED)
find_package(anytime_core REQUIRED)
find_package(anytime_tracing REQUIRED)

# Include Cpp "include" directory
include_directories(include)

set(anytime_dependencies "anytime_interfaces" "rclcpp" "rclcpp_action" "rclcpp_components" "anytime_core" "anytime_tracing")

# Add libraries
add_library(anytime_rrt_client_lib SHARED src/anytime_client.cpp)
add_library(anytime_rrt_server_lib SHARED src/anytime_server.cpp)

# Include directories
target_include_directories(anytime_rrt_client_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(anytime_rrt_server_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link dependencies
ament_target_dependencies(anytime_rrt_client_lib ${anytime_dependencies})
ament_target_dependencies(anytime_rrt_server_lib ${anytime_dependencies})

# Register components
rclcpp_components_register_node(anytime_rrt_client_lib
  PLUGIN "AnytimeRrtActionClient"
  EXECUTABLE anytime_rrt_client_component)

rclcpp_components_register_node(anytime_rrt_server_lib
  PLUGIN "AnytimeRrtActionServer"
  EXECUTABLE anytime_rrt_server_component)

# Create executables from main files
add_executable(anytime_rrt_client src/anytime_client_main.cpp)
add_executable(anytime_rrt_server src/anytime_server_main.cpp)

# Link executables with their respective libraries
target_link_libraries(anytime_rrt_client anytime_rrt_client_lib)
target_link_libraries(anytime_rrt_server anytime_rrt_server_lib)

# Install Cpp executables
install(TARGETS
  anytime_rrt_client
  anytime_rrt_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  anytime_rrt_client_lib
  anytime_rrt_server_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rrt_star_management test/test_rrt_star_management.cpp)
  ament_target_dependencies(test_rrt_star_management
    rclcpp rclcpp_action anytime_interfaces anytime_core anytime_tracing)
  target_include_directories(test_rrt_star_management PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(anytime_rrt_client_lib anytime_rrt_server_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${anytime_dependencies})

ament_package()
