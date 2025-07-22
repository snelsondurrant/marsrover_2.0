cmake_minimum_required(VERSION 3.8)
project(coug_gtsam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the executable for the pose graph node
add_executable(gtsam_dvl_localizer_node src/gtsam_dvl_localizer_node.cpp)
ament_target_dependencies(gtsam_dvl_localizer_node
  rclcpp 
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Manually add the include paths for GTSAM and its bundled Eigen dependency
target_include_directories(gtsam_dvl_localizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${GTSAM_INCLUDE_DIRS}
  /usr/local/include/gtsam/3rdparty/Eigen
)

# Link against GTSAM libraries
target_link_libraries(gtsam_dvl_localizer_node
  ${GTSAM_LIBRARIES}
  gtsam
  # The IncrementalFixedLagSmoother requires the unstable GTSAM library
  gtsam_unstable
)

# Install the executable
install(TARGETS
  gtsam_dvl_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()