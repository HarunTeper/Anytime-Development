cmake_minimum_required(VERSION 3.5)
project(waitable_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(anytime_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

set(anytime_dependencies "anytime_interfaces" "rclcpp" "rclcpp_action")

# Add libraries
add_library(cuda_waitable SHARED src/cuda_waitable.cpp)
ament_target_dependencies(cuda_waitable ${anytime_dependencies})

# Create Cpp executable
add_executable(basic_publisher src/basic_publisher.cpp)
ament_target_dependencies(basic_publisher rclcpp std_msgs)

# Link libraries
target_link_libraries(basic_publisher cuda_waitable)

# Export libraries
ament_export_targets(cuda_waitable HAS_LIBRARY_TARGET)

# Install Cpp executables
install(TARGETS
  basic_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS cuda_waitable
  EXPORT cuda_waitable
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
