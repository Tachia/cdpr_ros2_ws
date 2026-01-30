cmake_minimum_required(VERSION 3.8)
project(cdpr_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Find Eigen3 - corrected approach
find_package(Eigen3 REQUIRED)

# Add executable
add_executable(cdpr_controller src/cdpr_controller_simple.cpp)

# Include directories - CRITICAL: Add Eigen3 directories
target_include_directories(cdpr_controller PUBLIC
  ${Eigen3_INCLUDE_DIRS}
)

# Add dependencies
ament_target_dependencies(cdpr_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  visualization_msgs)

# Install executables
install(TARGETS cdpr_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install RViz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

# Install worlds
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS scripts/cdpr_visualizer.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
