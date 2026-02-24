cmake_minimum_required(VERSION 3.5)
project(anytime_yolo)

# Default to C++20
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

# libtorch path (if available)
if(DEFINED ENV{HOME})
  list(APPEND CMAKE_PREFIX_PATH $ENV{HOME}/libraries/libtorch)
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
find_package(sensor_msgs REQUIRED)
find_package(CUDA REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(TENSORRT_INCLUDE_DIR /usr/include/x86_64-linux-gnu)
set(TENSORRT_LIB_DIR /usr/lib/x86_64-linux-gnu)

include_directories(${TENSORRT_INCLUDE_DIR})
link_directories(${TENSORRT_LIB_DIR})
include_directories(include /usr/include/${ARCHITECTURE}-linux-gnu ${CUDA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(anytime_dependencies "anytime_interfaces" "rclcpp" "rclcpp_action" "rclcpp_components" "anytime_core" "anytime_tracing" "sensor_msgs" "cv_bridge")

# Add libraries
add_library(anytime_yolo_client_lib SHARED src/anytime_client.cpp)
add_library(anytime_yolo_server_lib SHARED src/anytime_server.cpp)

# Include directories
target_include_directories(anytime_yolo_client_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(anytime_yolo_server_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(anytime_yolo_server_lib 
  nvinfer
  nvonnxparser
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBS}
)

# Link dependencies
ament_target_dependencies(anytime_yolo_client_lib ${anytime_dependencies})
ament_target_dependencies(anytime_yolo_server_lib ${anytime_dependencies})

# Register components
rclcpp_components_register_node(anytime_yolo_client_lib
  PLUGIN "AnytimeActionClient"
  EXECUTABLE anytime_yolo_client_component)

rclcpp_components_register_node(anytime_yolo_server_lib
  PLUGIN "AnytimeActionServer"
  EXECUTABLE anytime_yolo_server_component)

# Create executables from main files
add_executable(anytime_yolo_client src/anytime_client_main.cpp)
add_executable(anytime_yolo_server src/anytime_server_main.cpp)

# Link executables with their respective libraries
target_link_libraries(anytime_yolo_client anytime_yolo_client_lib)
target_link_libraries(anytime_yolo_server anytime_yolo_server_lib)

# Install targets
install(TARGETS
  anytime_yolo_client_lib
  anytime_yolo_server_lib
  anytime_yolo_client
  anytime_yolo_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  weights_32
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(anytime_yolo_client_lib anytime_yolo_server_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${anytime_dependencies})

ament_package()
