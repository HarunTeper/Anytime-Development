cmake_minimum_required(VERSION 3.5)
project(anytime_monte_carlo)

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

# Include Cpp "include" directory
include_directories(include)

set(anytime_dependencies "anytime_interfaces" "rclcpp" "rclcpp_action" "rclcpp_components")

# Add libraries
add_library(anytime_client SHARED src/anytime_client.cpp)
add_library(anytime_server SHARED src/anytime_server.cpp)

# Include directories
target_include_directories(anytime_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(anytime_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link dependencies
ament_target_dependencies(anytime_client ${anytime_dependencies})
ament_target_dependencies(anytime_server ${anytime_dependencies})

# Register nodes
rclcpp_components_register_node(anytime_client PLUGIN "AnytimeActionClient" EXECUTABLE anytime_action_client)
rclcpp_components_register_node(anytime_server PLUGIN "AnytimeActionServer" EXECUTABLE anytime_action_server)

# Install targets
install(TARGETS
  anytime_client
  anytime_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# # Add libraries
# add_library(anytime_node SHARED src/anytime_node.cpp)

# # Create Cpp executable
# add_executable(anytime src/anytime_main.cpp)

# set(anytime_dependencies rclcpp )
# ament_target_dependencies(anytime_node ${anytime_dependencies})

# # Link libraries
# target_link_libraries(anytime anytime_node)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# # Install Cpp executables
# install(TARGETS
#   anytime
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS 
#   anytime_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# # Install Python modules
# ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
