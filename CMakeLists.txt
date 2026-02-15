cmake_minimum_required(VERSION 3.8)
project(lidar_filtering)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED) 
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 1. Build as a Shared Library (Component)
add_library(lidar_filtering_component SHARED
  src/lidar_filtering.cpp
)

# Required for modern ROS 2 component linking
target_include_directories(lidar_filtering_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 2. Link Dependencies
target_link_libraries(lidar_filtering_component
  ${PCL_LIBRARIES}
)

ament_target_dependencies(lidar_filtering_component
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
)

# 3. Register the Component
rclcpp_components_register_nodes(lidar_filtering_component
  "lidar_filtering::SlopeGroundFilter"
)

# 4. Install
install(TARGETS
  lidar_filtering_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(lidar_filtering_component)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs pcl_conversions)

ament_package()