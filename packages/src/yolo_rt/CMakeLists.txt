cmake_minimum_required(VERSION 3.5)
project(yolo_rt)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH /home/ls12-gpuhost/libtorch)
# list(APPEND CMAKE_PREFIX_PATH ~/.local/lib/python3.8/site-packages/torch)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Torch REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(yolo_rt_node SHARED src/yolo_node.cpp)
add_library(yolo_rt_model SHARED src/yolo_model.cpp)

# Create Cpp executable
add_executable(yolo_rt src/yolo_main.cpp)

set(yolo_rt_dependencies rclcpp Torch nlohmann_json sensor_msgs)
ament_target_dependencies(yolo_rt_node ${yolo_rt_dependencies})

# Link libraries
target_link_libraries(yolo_rt yolo_rt_node)
target_link_libraries(yolo_rt_node yolo_rt_model)
target_link_libraries(yolo_rt_node "${TORCH_LIBRARIES}")
target_link_libraries(yolo_rt_model "${TORCH_LIBRARIES}")

install(DIRECTORY 
  launch
  weights
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  yolo_rt
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  yolo_rt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
ament_python_install_package(yolov7)
ament_python_install_package(models)
ament_python_install_package(utils)

# Install Python executables
install(PROGRAMS
  scripts/yolo.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
