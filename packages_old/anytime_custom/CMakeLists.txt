cmake_minimum_required(VERSION 3.5)
project(anytime_custom)

# Default to C++14
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

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(anytime_node SHARED src/anytime_node.cpp)

# Create Cpp executable
add_executable(anytime src/anytime_main.cpp)

set(anytime_dependencies "rclcpp")
ament_target_dependencies(anytime_node ${anytime_dependencies})

target_link_libraries(anytime anytime_node)

install(TARGETS
  anytime_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  anytime
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  anytime_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
