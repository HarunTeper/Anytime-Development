cmake_minimum_required(VERSION 3.5)
project(anytime_yolo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Detect architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(ARCHITECTURE "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(ARCHITECTURE "x86_64")
else()
  message(WARNING "Unknown architecture ${CMAKE_SYSTEM_PROCESSOR}, defaulting to x86_64")
  set(ARCHITECTURE "x86_64")
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
find_package(sensor_msgs REQUIRED)
find_package(CUDA REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(TENSORRT_INCLUDE_DIR /usr/include/x86_64-linux-gnu)
set(TENSORRT_LIB_DIR /usr/lib/x86_64-linux-gnu)

include_directories(${TENSORRT_INCLUDE_DIR})
link_directories(${TENSORRT_LIB_DIR})

# Include Cpp "include" directory
include_directories(include /usr/include/${ARCHITECTURE}-linux-gnu ${CUDA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(anytime_dependencies "anytime_interfaces" "rclcpp" "rclcpp_action" "rclcpp_components" "sensor_msgs" "cv_bridge")

# Add libraries
add_library(anytime_client SHARED src/anytime_client.cpp)
add_library(anytime_server SHARED src/anytime_server.cpp)


# Include directories
target_include_directories(anytime_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(anytime_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(anytime_server 
  nvinfer
  nvonnxparser
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBS}
)


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
  weights_32
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
