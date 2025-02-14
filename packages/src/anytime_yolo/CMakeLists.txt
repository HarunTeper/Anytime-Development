cmake_minimum_required(VERSION 3.5)
project(anytime_yolo)

# Default to C++17
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
find_package(CUDA REQUIRED)
# find_package(Torch REQUIRED)
# find_package(nlohmann_json REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(vision_msgs REQUIRED)
# find_package(waitable_test REQUIRED)
# find_package(anytime_interfaces REQUIRED)
# find_package(rclcpp_action REQUIRED)

# Include Cpp "include" directory
include_directories(include)
include_directories(include /usr/include/aarch64-linux-gnu ${CUDA_INCLUDE_DIRS})

# Add libraries
add_library(anytime_yolo_node SHARED src/yolo_node.cpp)
add_library(anytime_yolo_model SHARED src/yolo_model.cpp)

# Create Cpp executable
add_executable(anytime_yolo src/yolo_main.cpp)

set(yolo_rt_dependencies "rclcpp" )
ament_target_dependencies(anytime_yolo_node ${yolo_rt_dependencies})
ament_target_dependencies(anytime_yolo_model ${yolo_rt_dependencies})

# Link libraries
target_link_libraries(anytime_yolo anytime_yolo_node)
target_link_libraries(anytime_yolo_node anytime_yolo_model)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(anytime_yolo_model /usr/lib/aarch64-linux-gnu/libnvinfer.so ${CUDA_LIBRARIES})
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(anytime_yolo_model /usr/lib/x86_64-linux-gnu/libnvinfer.so ${CUDA_LIBRARIES})
endif()

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  anytime_yolo
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  anytime_yolo_node
  anytime_yolo_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# ament_python_install_package(yolov7)
# ament_python_install_package(models)
# ament_python_install_package(utils)

# # Install Python executables
# install(PROGRAMS
#   scripts/yolo.py
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
