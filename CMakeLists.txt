cmake_minimum_required(VERSION 3.14)
project(global_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Library
add_library(globalMap SHARED
  src/global_map.cpp 
)

target_link_libraries(globalMap
  ${PCL_LIBRARIES}
  -loctomap
  -loctomath 
)

ament_target_dependencies(globalMap
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
  octomap_msgs
  octomap
  std_msgs
)

# Executable
add_executable(global_map_node src/main.cpp src/global_map.cpp) 
target_link_libraries(global_map_node globalMap -loctomath  )

ament_target_dependencies(global_map_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
  octomap_msgs
  octomap
  std_msgs
)

# Install library
install(TARGETS globalMap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# Install executable
install(TARGETS global_map_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Export include directories
ament_export_include_directories(include)
ament_export_libraries(globalMap)
ament_export_dependencies(rclcpp sensor_msgs  pcl_conversions nav_msgs geometry_msgs octomap_msgs octomap)

ament_package()